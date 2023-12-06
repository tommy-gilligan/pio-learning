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

const EXPECTED_SM: &'static str = r###"{
  "sm_clkdiv": "00000000000000101000000000000000",
  "sm_execctrl": "00000000000000011111000000000000",
  "sm_shiftctrl": "00000000000011000000000000000000",
  "sm_addr": "00000000000000000000000000000000",
  "sm_instr": "00000000000000001110000010000001",
  "sm_pinctrl": "00000100000000000000000001000000"
}"###;

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    let external_xtal_freq_hz = 12_000_000u32;
    init_clocks_and_plls(
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

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    let pin: rp_pico::hal::gpio::Pin<
        rp_pico::hal::gpio::bank0::Gpio25,
        FunctionPio0,
        rp_pico::hal::gpio::PullDown,
    > = pins.led.into_function();

    let program_with_defines = pio_proc::pio_file!(
        "examples/squarewave/squarewave.pio",
        select_program("squarewave"),
        options(max_program_size = 32)
    );
    let program = program_with_defines.program;

    unsafe {
        core::ptr::write_volatile(0x50200000 as *mut u32, 0);
    }

    for (index, instruction) in program.code.iter().enumerate() {
        unsafe {
            core::ptr::write_volatile(
                (0x50200048 + index * 4) as *mut u32,
                (*instruction) as u32
            );
        }
    }

    unsafe {
        core::ptr::write_volatile(0x502000c8 as *mut u32, (2.5f32 * (1 << 16) as f32) as u32);
        core::ptr::write_volatile(
            0x502000dc as *mut u32,
            (1 << 26) | ((pin.id().num as u32) << 5),
        );
    }

    // SmStateCopy::assert_eq(SM0_BASE, EXPECTED_SM);
    // PioStateCopy::assert_eq(EXPECTED_PIO);

    unsafe {
        core::ptr::write_volatile(
            0x50200000 as *mut u32,
            core::ptr::read_volatile(0x50200000 as *mut u32) | 1,
        );
    }

    loop {}
}
