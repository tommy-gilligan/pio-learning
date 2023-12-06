#![no_std]
#![no_main]

use bsp::entry;
use defmt::*;
use defmt_rtt as _;
use panic_probe as _;
use rp2040_project_template::{PioStateCopy, SmStateCopy, SM0_BASE};

use rp_pico as bsp;

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac,
    sio::Sio,
    watchdog::Watchdog,
};
use rand::rngs::SmallRng;
use rand::RngCore;
use rand::{Rng, SeedableRng};
use rp_pico::hal::pio::PIOExt;

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

const EXPECTED_SM0: &'static str = r###"{
  "sm_clkdiv": "00000000000000010000000000000000",
  "sm_execctrl": "00000000000000011111101110000000",
  "sm_shiftctrl": "00000000000011000000000000000000",
  "sm_addr": "00000000000000000000000000010111",
  "sm_instr": "00000000000000001000000010100000",
  "sm_pinctrl": "00000000000000000000000000000000"
}"###;

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

    let program_with_defines = pio_proc::pio_file!(
        "examples/addition/addition.pio",
        select_program("addition"),
        options(max_program_size = 32)
    );
    let program = program_with_defines.program;

    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let installed = pio.install(&program).unwrap();

    let (mut sm, mut rx, mut tx) = rp_pico::hal::pio::PIOBuilder::from_program(installed)
        .autopush(false)
        .autopull(false)
        .out_shift_direction(rp_pico::hal::pio::ShiftDirection::Right)
        .in_shift_direction(rp_pico::hal::pio::ShiftDirection::Right)
        .set_pins(0, 0)
        .out_pins(0, 0)
        .build(sm0);
    sm.set_pindirs([]);

    PioStateCopy::assert_eq(EXPECTED_PIO);
    SmStateCopy::assert_eq(SM0_BASE, EXPECTED_SM0);

    sm.start();

    let mut small_rng = SmallRng::seed_from_u64(0x69420);

    info!("Doing some random additions:\n");

    for _ in 0..10 {
        let a = small_rng.next_u32() % 100;
        let b = small_rng.next_u32() % 100;

        while tx.is_full() {}
        tx.write(a);
        while tx.is_full() {}
        tx.write(b);
        while rx.is_empty() {}
        info!("{} + {} = {}\n", a, b, rx.read().unwrap());
    }

    loop {}
}
