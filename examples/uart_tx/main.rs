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
  "fstat": "00001111000000000000111100000001",
  "fdebug": "00000000000000000000000000000000",
  "flevel": "00000000000000000000000000000000",
  "irq": "00000000000000000000000000000000",
  "dbg_padout": "00000000000000010000000000000000",
  "dbg_padoe": "00000000000000010000000000000000",
  "dbg_cfginfo": "00000000001000000000010000000100"
}"###;

const EXPECTED_SM: &'static str = r###"{
  "sm_clkdiv": "00000000100001111010001000000000",
  "sm_execctrl": "01000000000000011111111000000000",
  "sm_shiftctrl": "01000000000011000000000000000000",
  "sm_addr": "00000000000000000000000000011100",
  "sm_instr": "00000000000000001001111110100000",
  "sm_pinctrl": "01000000000100000100000000010000"
}"###;

const HELLO_WORLD: &'static str = "Hello, world! (from PIO!)\n";

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
    let pin: rp_pico::hal::gpio::Pin<
        rp_pico::hal::gpio::bank0::Gpio16,
        FunctionPio0,
        rp_pico::hal::gpio::PullDown,
    > = pins.gpio16.into_function();

    let program_with_defines = pio_proc::pio_file!(
        "examples/uart_tx/uart_tx.pio",
        select_program("uart_tx"),
        options(max_program_size = 32)
    );
    let program = program_with_defines.program;

    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);

    let (mut sm, _, mut tx) =
        rp_pico::hal::pio::PIOBuilder::from_program(pio.install(&program).unwrap())
            .buffers(rp_pico::hal::pio::Buffers::OnlyTx)
            .autopull(false)
            .pull_threshold(32)
            .out_shift_direction(rp_pico::hal::pio::ShiftDirection::Right)
            .out_pins(pin.id().num, 1)
            .set_pins(0, 0)
            .side_set_pin_base(pin.id().num)
            .in_shift_direction(rp_pico::hal::pio::ShiftDirection::Right)
            .autopush(false)
            .build(sm0);

    // should this be 125???
    sm.set_clock_divisor(125_000_000f32 / (8f32 * 115200f32));
    sm.set_pindirs([(pin.id().num, rp_pico::hal::pio::PinDir::Output)]);
    sm.set_pins([(pin.id().num, rp_pico::hal::pio::PinState::High)]);

    SmStateCopy::assert_eq(SM0_BASE, EXPECTED_SM);
    PioStateCopy::assert_eq(EXPECTED_PIO);

    sm.start();

    loop {
        for byte in HELLO_WORLD.bytes() {
            while tx.is_full() {}
            tx.write(byte as u32);
        }
        delay.delay_ms(1000);
    }
}
