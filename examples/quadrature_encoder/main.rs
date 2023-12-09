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
  "sm_execctrl": "00001010000000010111011110000000",
  "sm_shiftctrl": "00000000000010000000000000000000",
  "sm_addr": "00000000000000000000000000000000",
  "sm_instr": "00000000000000000000000000001111",
  "sm_pinctrl": "00000000000001010000000000000000"
}"###;

const EXPECTED_PIO: &'static str = r###"{
  "ctrl": "00000000000000000000000000000000",
  "fstat": "00001111000000000000111100000000",
  "fdebug": "00000000000000000000000000000000",
  "flevel": "00000000000000000000000000000000",
  "irq": "00000000000000000000000000000000",
  "dbg_padout": "00000000000000000000000000000000",
  "dbg_padoe": "00000000000000000000000000000000",
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
        "examples/quadrature_encoder/quadrature_encoder.pio",
        select_program("quadrature_encoder"),
        options(max_program_size = 32)
    );
    let program = program_with_defines.program;

    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);

    let pin_a: rp_pico::hal::gpio::Pin<
        rp_pico::hal::gpio::bank0::Gpio10,
        FunctionPio0,
        rp_pico::hal::gpio::PullDown,
    > = pins.gpio10.into_function();
    let pin_b: rp_pico::hal::gpio::Pin<
        rp_pico::hal::gpio::bank0::Gpio11,
        FunctionPio0,
        rp_pico::hal::gpio::PullDown,
    > = pins.gpio11.into_function();

    let (mut sm, mut rx, _) =
        rp_pico::hal::pio::PIOBuilder::from_program(pio.install(&program).unwrap())
            .buffers(rp_pico::hal::pio::Buffers::RxTx)
            .autopush(false)
            .in_shift_direction(rp_pico::hal::pio::ShiftDirection::Left)
            .push_threshold(32)
            .in_pin_base(pin_a.id().num)
            .jmp_pin(pin_a.id().num)
            .autopull(false)
            .out_shift_direction(rp_pico::hal::pio::ShiftDirection::Right)
            .set_pins(0, 0)
            .build(sm0);

    sm.set_pindirs([
        (pin_a.id().num, rp_pico::hal::pio::PinDir::Input),
        (pin_b.id().num, rp_pico::hal::pio::PinDir::Input),
    ]);
    sm.set_clock_divisor(1.0);

    PioStateCopy::assert_eq(EXPECTED_PIO);
    SmStateCopy::assert_eq(SM0_BASE, EXPECTED_SM);

    sm.start();

    let mut old_value: i32 = 0;

    loop {
        while rx.is_empty() {}
        let ret: i32 = rx.read().unwrap() as i32;
        let delta: i32 = ret - old_value;
        old_value = ret;

        if delta != 0 {
            println!("position {}, delta {}", ret, delta);
        }
    }
}
