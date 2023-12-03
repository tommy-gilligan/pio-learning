//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

use bsp::entry;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::v2::OutputPin;
use panic_probe as _;

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use rp_pico as bsp;
// use sparkfun_pro_micro_rp2040 as bsp;

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac,
    sio::Sio,
    watchdog::Watchdog,
};
use bsp::hal::gpio::{Pin, FunctionPio0};
use pio_proc::pio_file;
use rp_pico::hal::pio::PIOExt;

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
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

    // This is the correct pin on the Raspberry Pico board. On other boards, even if they have an
    // on-board LED, it might need to be changed.
    // Notably, on the Pico W, the LED is not connected to any of the RP2040 GPIOs but to the cyw43 module instead. If you have
    // a Pico W and want to toggle a LED with a simple GPIO output pin, you can connect an external
    // LED to one of the GPIO pins, and reference that pin here.
    let led_pin: Pin<_, FunctionPio0, _> = pins.led.into_function();
    let led_pin_id = led_pin.id().num;

    // Initialize and start PIO
    let program_with_defines = pio_proc::pio_file!(
        "examples/hello_pio/hello.pio",
        select_program("hello"), // Optional if only one program in the file
        options(max_program_size = 32) // Optional, defaults to 32
    );
    let program = program_with_defines.program;

    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let installed = pio.install(&program).unwrap();

    let (mut sm, _, mut tx) = rp_pico::hal::pio::PIOBuilder::from_program(installed)
        .out_pins(led_pin_id, 1)
        .autopull(true)
        .out_shift_direction(rp_pico::hal::pio::ShiftDirection::Right)
        .build(sm0);
    sm.set_pindirs([(led_pin_id, rp_pico::hal::pio::PinDir::Output)]);
    sm.start();

    loop {
        info!("on! {} ", tx.write(1));
        delay.delay_ms(500);

        info!("off! {}", tx.write(0));
        delay.delay_ms(500);
    }
}
