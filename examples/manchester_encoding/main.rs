#![no_std]
#![no_main]

use bsp::entry;
use defmt::*;
use defmt_rtt as _;
use panic_probe as _;
use rp2040_project_template::{PioStateCopy, SmStateCopy, SM0_BASE, SM1_BASE};

use rp_pico as bsp;

use bsp::hal::gpio::{FunctionPio0, Pin};
use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac,
    sio::Sio,
    watchdog::Watchdog,
};
use rp_pico::hal::pio::PIOExt;

const EXPECTED_SM0: &'static str = r###"{
  "sm_clkdiv": "00000000000000010000000000000000",
  "sm_execctrl": "01000000000000011111110100000000",
  "sm_shiftctrl": "01000000000011100000000000000000",
  "sm_addr": "00000000000000000000000000011110",
  "sm_instr": "00000000000000000110000000100001",
  "sm_pinctrl": "01000000000000000000100000000000"
}"###;

const EXPECTED_SM1: &'static str = r###"{
  "sm_clkdiv": "00000000000000010000000000000000",
  "sm_execctrl": "10000011000000011001101110000000",
  "sm_shiftctrl": "10000000000011010000000000000000",
  "sm_addr": "00000000000000000000000000010100",
  "sm_instr": "00000000000000000010000000100000",
  "sm_pinctrl": "00000000000000011000000000000000"
}"###;

const EXPECTED_PIO: &'static str = r###"{
  "ctrl": "00000000000000000000000000000001",
  "fstat": "00001111000000100000111100000001",
  "fdebug": "00000001000000000000000000000000",
  "flevel": "00000000000000000000000000000000",
  "irq": "00000000000000000000000000000000",
  "dbg_padout": "00000000000000000000000000000000",
  "dbg_padoe": "00000000000000000000000000000100",
  "dbg_cfginfo": "00000000001000000000010000000100"
}"###;

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    let pin_tx: rp_pico::hal::gpio::Pin<
        rp_pico::hal::gpio::bank0::Gpio2,
        FunctionPio0,
        rp_pico::hal::gpio::PullDown,
    > = pins.gpio2.into_function();
    let pin_rx: rp_pico::hal::gpio::Pin<
        rp_pico::hal::gpio::bank0::Gpio3,
        FunctionPio0,
        rp_pico::hal::gpio::PullDown,
    > = pins.gpio3.into_function();

    let program_with_defines_tx = pio_proc::pio_file!(
        "examples/manchester_encoding/manchester_encoding.pio",
        select_program("manchester_tx"),
        options(max_program_size = 32)
    );
    let program_tx = program_with_defines_tx.program;
    let start = program_with_defines_tx.public_defines.start;

    let program_with_defines_rx = pio_proc::pio_file!(
        "examples/manchester_encoding/manchester_encoding.pio",
        select_program("manchester_rx"),
        options(max_program_size = 32)
    );
    let program_rx = program_with_defines_rx.program;

    let (mut pio, sm0, sm1, _, _) = pac.PIO0.split(&mut pac.RESETS);

    let program = pio.install(&program_tx).unwrap();
    let offset = program.offset();
    let (mut sm_tx, _, mut tx) = rp_pico::hal::pio::PIOBuilder::from_program(program)
        .autopull(true)
        .pull_threshold(32)
        .out_shift_direction(rp_pico::hal::pio::ShiftDirection::Right)
        .buffers(rp_pico::hal::pio::Buffers::OnlyTx)
        .autopush(false)
        .in_shift_direction(rp_pico::hal::pio::ShiftDirection::Right)
        .out_pins(0, 0)
        .set_pins(0, 0)
        .side_set_pin_base(pin_tx.id().num)
        .build(sm0);
    sm_tx.set_pins([(pin_tx.id().num, rp_pico::hal::pio::PinState::Low)]);
    sm_tx.set_clock_divisor(1f32);
    sm_tx.set_pindirs([(pin_tx.id().num, rp_pico::hal::pio::PinDir::Output)]);

    sm_tx.exec_instruction(pio::Instruction {
        operands: pio::InstructionOperands::JMP {
            condition: pio::JmpCondition::Always,
            address: offset + start as u8,
        },
        delay: 0,
        side_set: None,
    });
    SmStateCopy::assert_eq(SM0_BASE, EXPECTED_SM0);
    let sm_tx = sm_tx.start();

    let (mut sm_rx, mut rx, _) =
        rp_pico::hal::pio::PIOBuilder::from_program(pio.install(&program_rx).unwrap())
            .autopull(false)
            .out_shift_direction(rp_pico::hal::pio::ShiftDirection::Right)
            .push_threshold(32)
            .autopush(true)
            .in_shift_direction(rp_pico::hal::pio::ShiftDirection::Right)
            .out_pins(0, 0)
            .set_pins(0, 0)
            .in_pin_base(pin_rx.id().num)
            .jmp_pin(pin_rx.id().num)
            .buffers(rp_pico::hal::pio::Buffers::OnlyRx)
            .build(sm1);
    sm_rx.set_clock_divisor(1f32);
    sm_rx.set_pindirs([(pin_rx.id().num, rp_pico::hal::pio::PinDir::Input)]);

    sm_rx.exec_instruction(pio::Instruction {
        operands: pio::InstructionOperands::SET {
            destination: pio::SetDestination::X,
            data: 1,
        },
        delay: 0,
        side_set: None,
    });
    sm_rx.exec_instruction(pio::Instruction {
        operands: pio::InstructionOperands::SET {
            destination: pio::SetDestination::Y,
            data: 0,
        },
        delay: 0,
        side_set: None,
    });
    sm_rx.exec_instruction(pio::Instruction {
        operands: pio::InstructionOperands::WAIT {
            polarity: 1,
            source: pio::WaitSource::PIN,
            index: 0,
            relative: false,
        },
        delay: 2,
        side_set: None,
    });

    SmStateCopy::assert_eq(SM1_BASE, EXPECTED_SM1);
    PioStateCopy::assert_eq(EXPECTED_PIO);

    sm_rx.start();

    let mut sm_tx = sm_tx.stop();
    sm_tx.drain_tx_fifo();
    while tx.is_full() {}
    tx.write(0);
    while tx.is_full() {}
    tx.write(0x0ff0a55a);
    while tx.is_full() {}
    tx.write(0x12345678);

    sm_tx.start();

    for _ in 0..3 {
        while rx.is_empty() {}
        info!("{:#x}\n", rx.read().unwrap());
    }

    loop {
        delay.delay_ms(10);
    }
}
