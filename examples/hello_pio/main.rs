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

const EXPECTED_PIO: &'static str = "{\"ctrl\":1,\"fstat\":251662080,\"fdebug\":16777216,\"flevel\":0,\"irq\":0,\"dbg_padout\":0,\"dbg_padoe\":33554432,\"dbg_cfginfo\":2098180}";
const EXPECTED_SM: &'static str = "{\"sm_clkdiv\":65536,\"sm_execctrl\":130688,\"sm_shiftctrl\":786432,\"sm_addr\":29,\"sm_instr\":32928,\"sm_pinctrl\":1048601}";

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

    let led_pin: Pin<_, FunctionPio0, _> = pins.led.into_function();
    let led_pin_id = led_pin.id().num;

    let program_with_defines = pio_proc::pio_file!(
        "examples/hello_pio/hello.pio",
        select_program("hello"),
        options(max_program_size = 32)
    );
    let program = program_with_defines.program;

    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let installed = pio.install(&program).unwrap();

    let (mut sm, _, mut tx) = rp_pico::hal::pio::PIOBuilder::from_program(installed)
        .out_pins(led_pin_id, 1)
        .set_pins(0, 0)
        .out_shift_direction(rp_pico::hal::pio::ShiftDirection::Right)
        .in_shift_direction(rp_pico::hal::pio::ShiftDirection::Right)
        .build(sm0);

    sm.set_pindirs([(led_pin_id, rp_pico::hal::pio::PinDir::Output)]);
    sm.start();

    PioStateCopy::assert_eq(EXPECTED_PIO);
    SmStateCopy::assert_eq(SM0_BASE, EXPECTED_SM);

    loop {
        tx.write(1);
        delay.delay_ms(500);

        tx.write(0);
        delay.delay_ms(500);
    }
}
