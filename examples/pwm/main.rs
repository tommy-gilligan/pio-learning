#![no_std]
#![no_main]

use bsp::entry;
use defmt::*;
use defmt_rtt as _;
use panic_probe as _;
use rp2040_project_template::{PioStateCopy, SmStateCopy, SM0_BASE};

use rp_pico as bsp;

use bsp::hal::gpio::{FunctionPio0, Pin};
use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac,
    sio::Sio,
    watchdog::Watchdog,
};
use rp_pico::hal::pio::PIOExt;

const EXPECTED_SM: &'static str = r###"{
  "sm_clkdiv": "00000000000000010000000000000000",
  "sm_execctrl": "01000000000000011111110010000000",
  "sm_shiftctrl": "00000000000011000000000000000000",
  "sm_addr": "00000000000000000000000000011001",
  "sm_instr": "00000000000000001001000010000000",
  "sm_pinctrl": "01000000000000000110010000000000"
}"###;

const EXPECTED_PIO: &'static str = r###"{
  "ctrl": "00000000000000000000000000000000",
  "fstat": "00001111000000000000111100000000",
  "fdebug": "00000000000000000000000000000000",
  "flevel": "00000000000000000000000000000000",
  "irq": "00000000000000000000000000000000",
  "dbg_padout": "00000000000000000000000000000000",
  "dbg_padoe": "00000010000000000000000000000000",
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

    let program_with_defines = pio_proc::pio_file!(
        "examples/pwm/pwm.pio",
        select_program("pwm"),
        options(max_program_size = 32)
    );
    let program = program_with_defines.program;

    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);

    let pin: rp_pico::hal::gpio::Pin<
        rp_pico::hal::gpio::bank0::Gpio25,
        FunctionPio0,
        rp_pico::hal::gpio::PullDown
    > = pins.led.into_function();
    let (mut sm, _, mut tx) =
        rp_pico::hal::pio::PIOBuilder::from_program(pio.install(&program).unwrap())
            .autopush(false)
            .autopull(false)
            .out_shift_direction(rp_pico::hal::pio::ShiftDirection::Right)
            .in_shift_direction(rp_pico::hal::pio::ShiftDirection::Right)
            .set_pins(0, 0)
            .side_set_pin_base(pin.id().num)
            .build(sm0);

    sm.set_pindirs([(pin.id().num, rp_pico::hal::pio::PinDir::Output)]);

    while tx.is_full() {}
    tx.write((1 << 16) - 1);

    sm.exec_instruction(pio::Instruction {
        operands: pio::InstructionOperands::PULL {
            if_empty: false,
            block: false,
        },
        delay: 0,
        side_set: None,
    });
    sm.exec_instruction(pio::Instruction {
        operands: pio::InstructionOperands::OUT {
            destination: pio::OutDestination::ISR,
            bit_count: 32,
        },
        delay: 0,
        side_set: None,
    });

    PioStateCopy::assert_eq(EXPECTED_PIO);
    SmStateCopy::assert_eq(SM0_BASE, EXPECTED_SM);

    sm.start();

    loop {
        for level in 0..256 {
            info!("Level = {}\n", level);
            while tx.is_full() {}
            tx.write(level * level);
            delay.delay_ms(10);
        }
    }
}
