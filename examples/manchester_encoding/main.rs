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

const EXPECTED_PIO: &'static str = "{\"ctrl\":0,\"fstat\":251793153,\"fdebug\":16777216,\"flevel\":0,\"irq\":0,\"dbg_padout\":0,\"dbg_padoe\":4,\"dbg_cfginfo\":2098180}";

const EXPECTED_SM_0: &'static str = "{\"sm_clkdiv\":65536,\"sm_execctrl\":1073872128,\"sm_shiftctrl\":1074659328,\"sm_addr\":30,\"sm_instr\":24609,\"sm_pinctrl\":1073743872}";
const EXPECTED_SM_1: &'static str = "{\"sm_clkdiv\":65536,\"sm_execctrl\":50436992,\"sm_shiftctrl\":2148335616,\"sm_addr\":20,\"sm_instr\":8224,\"sm_pinctrl\":98304}";

#[entry]
fn main() -> ! {
    info!("Program start");
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
    delay.delay_ms(2000);
    let start = program_with_defines_tx.public_defines.start;
    info!("{:?}", start);

    let program_with_defines_rx = pio_proc::pio_file!(
        "examples/manchester_encoding/manchester_encoding.pio",
        select_program("manchester_rx"),
        options(max_program_size = 32)
    );
    let program_rx = program_with_defines_rx.program;
    info!("Transmit program loaded\n");
    info!("Receive program loaded\n");

    let (mut pio, sm0, sm1, _, _) = pac.PIO0.split(&mut pac.RESETS);

    let (mut sm_tx, _, mut tx) =
        rp_pico::hal::pio::PIOBuilder::from_program(pio.install(&program_tx).unwrap())
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

    // sm_tx.exec_instruction(pio::Instruction {
    //     operands: pio::InstructionOperands::JMP {
    //         condition: pio::JmpCondition::Always,
    //         address: start as u8
    //     },
    //     delay: 0,
    //     side_set: None,
    // });
    let mut sm_tx = sm_tx.start();

    let (mut sm_rx, mut rx, _) =
        rp_pico::hal::pio::PIOBuilder::from_program(pio.install(&program_rx).unwrap())
            .autopull(false)
            .out_shift_direction(rp_pico::hal::pio::ShiftDirection::Right)
            .buffers(rp_pico::hal::pio::Buffers::OnlyRx)
            .push_threshold(32)
            .autopush(true)
            .in_shift_direction(rp_pico::hal::pio::ShiftDirection::Right)
            .out_pins(0, 0)
            .set_pins(0, 0)
            .in_pin_base(pin_rx.id().num)
            .jmp_pin(pin_rx.id().num)
            .build(sm1);

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

    sm_rx.set_clock_divisor(1f32);

    let mut sm_tx = sm_tx.stop();
    sm_tx.set_pindirs([(pin_tx.id().num, rp_pico::hal::pio::PinDir::Output)]);
    sm_rx.set_pindirs([(pin_rx.id().num, rp_pico::hal::pio::PinDir::Input)]);

    let mut sm_tx = sm_tx.start();
    sm_rx.start();

    delay.delay_ms(2000);

    sm_tx.drain_tx_fifo();
    tx.write(0);
    tx.write(0x0ff0a55a);
    tx.write(0x12345678);

    for _ in 0..3 {
        while rx.is_empty() {}
        info!("{:#x}\n", rx.read().unwrap());
    }

    loop {
        delay.delay_ms(10);
    }
}
