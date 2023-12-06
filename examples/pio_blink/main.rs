#![no_std]
#![no_main]

use bsp::entry;
use defmt_rtt as _;
use panic_probe as _;
use rp2040_project_template::{PioStateCopy, SmStateCopy, SM0_BASE};

use rp_pico as bsp;

use bsp::hal::gpio::FunctionPio0;
use bsp::hal::{clocks::init_clocks_and_plls, pac, sio::Sio, watchdog::Watchdog};
use rp_pico::hal::pio::PIOExt;

const EXPECTED_PIO: &'static str = r###"{
  "ctrl":        "00000000000000000000000000000000",
  "fstat":       "00001111000000000000111100000000",
  "fdebug":      "00000000000000000000000000000000",
  "flevel":      "00000000000000000000000000000000",
  "irq":         "00000000000000000000000000000000",
  "dbg_padout":  "00000000000000000000000000000000",
  "dbg_padoe":   "00000000000000000000000000000100",
  "dbg_cfginfo": "00000000001000000000010000000100"
}"###;

const EXPECTED_SM_0: &'static str = r###"{
  "sm_clkdiv":    "00000000000000010000000000000000",
  "sm_execctrl":  "00000000000000011111110100000000",
  "sm_shiftctrl": "00000000000011000000000000000000",
  "sm_addr":      "00000000000000000000000000011000",
  "sm_instr":     "00000000000000001000000010100000",
  "sm_pinctrl":   "00000100000000000000000001000000"
}"###;

#[entry]
fn main() -> ! {
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

    let program_with_defines = pio_proc::pio_file!(
        "examples/pio_blink/blink.pio",
        select_program("blink"),
        options(max_program_size = 32)
    );
    let program = program_with_defines.program;

    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let pin: rp_pico::hal::gpio::Pin<
        rp_pico::hal::gpio::bank0::Gpio2,
        FunctionPio0,
        rp_pico::hal::gpio::PullDown,
    > = pins.gpio2.into_function();

    let (mut sm, _, mut tx) =
        rp_pico::hal::pio::PIOBuilder::from_program(pio.install(&program).unwrap())
            .autopush(false)
            .autopull(false)
            .out_shift_direction(rp_pico::hal::pio::ShiftDirection::Right)
            .in_shift_direction(rp_pico::hal::pio::ShiftDirection::Right)
            .set_pins(pin.id().num, 1)
            .build(sm0);

    sm.set_pindirs([(pin.id().num, rp_pico::hal::pio::PinDir::Output)]);

    SmStateCopy::assert_eq(SM0_BASE, EXPECTED_SM_0);
    PioStateCopy::assert_eq(EXPECTED_PIO);

    sm.start();

    let freq: u32 = 3;
    tx.write((133_000_000 / (2 * freq)) - 3);

    loop {}
}
