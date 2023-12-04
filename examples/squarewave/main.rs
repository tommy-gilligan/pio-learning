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

const EXPECTED_SM: &'static str = "{\"sm_clkdiv\":163840,\"sm_execctrl\":126976,\"sm_shiftctrl\":786432,\"sm_addr\":0,\"sm_instr\":57473,\"sm_pinctrl\":67108928}";
const EXPECTED_PIO: &'static str = "{\"ctrl\":0,\"fstat\":251662080,\"fdebug\":0,\"flevel\":0,\"irq\":0,\"dbg_padout\":0,\"dbg_padoe\":0,\"dbg_cfginfo\":2098180}";

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
    let pin: rp_pico::hal::gpio::Pin<
        rp_pico::hal::gpio::bank0::Gpio2,
        FunctionPio0,
        rp_pico::hal::gpio::PullDown,
    > = pins.gpio2.into_function();

    let program_with_defines = pio_proc::pio_file!(
        "examples/squarewave/squarewave.pio",
        select_program("squarewave"),
        options(max_program_size = 32)
    );
    let program = program_with_defines.program;

    for (index, instruction) in program.code.iter().enumerate() {
        unsafe {
            core::ptr::write_volatile((0x50200048 + index * 4) as *mut u32, (*instruction) as u32);
        }
    }

    unsafe {
        core::ptr::write_volatile(0x502000c8 as *mut u32, (2.5f32 * (1 << 16) as f32) as u32);
        core::ptr::write_volatile(
            0x502000dc as *mut u32,
            (1 << 26) | ((pin.id().num as u32) << 5),
        );
    }

    unsafe {
        core::ptr::write_volatile(
            0x50200000 as *mut u32,
            core::ptr::read_volatile(0x50200000 as *mut u32) | 1,
        );
    }
    // PioStateCopy::assert_eq(EXPECTED_PIO);
    // SmStateCopy::assert_eq(SM0_BASE, EXPECTED_SM);

    loop {}
}
