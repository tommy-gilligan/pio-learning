#![no_std]
#![no_main]

use bsp::entry;
use defmt::*;
use defmt_rtt as _;
use panic_probe as _;
use rp2040_project_template::{
    PioStateCopy,
    SmStateCopy,
    SM0_BASE,
    SM1_BASE,
    SM2_BASE
};

use rp_pico as bsp;

use bsp::hal::gpio::{FunctionPio0, Pin};
use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac,
    sio::Sio,
    watchdog::Watchdog,
};
use rp_pico::hal::pio::PIOExt;

const EXPECTED_SM: &'static str = "{\"sm_clkdiv\":65536,\"sm_execctrl\":130304,\"sm_shiftctrl\":786432,\"sm_addr\":28,\"sm_instr\":92,\"sm_pinctrl\":67108928}";
const EXPECTED_PIO: &'static str = "{\"ctrl\":7,\"fstat\":251662080,\"fdebug\":117440512,\"flevel\":0,\"irq\":0,\"dbg_padout\":2116,\"dbg_padoe\":2116,\"dbg_cfginfo\":2098180}";

fn blink_pin_forever<T, S>(
    pio: &mut rp_pico::hal::pio::PIO<rp_pico::pac::PIO0>,
    program: &pio::Program<32>,
    sm: rp_pico::hal::pio::UninitStateMachine<(rp_pico::pac::PIO0, S)>,
    pin: rp_pico::hal::gpio::Pin<T, FunctionPio0, rp_pico::hal::gpio::PullDown>,
    freq: u32
) where T: rp_pico::hal::gpio::PinId, S: rp_pico::hal::pio::StateMachineIndex {
    let (mut sm, _, mut tx) = rp_pico::hal::pio::PIOBuilder::from_program(pio.install(&program).unwrap())
        .autopush(false)
        .autopull(false)
        .out_shift_direction(rp_pico::hal::pio::ShiftDirection::Right)
        .in_shift_direction(rp_pico::hal::pio::ShiftDirection::Right)
        .out_pins(0, 0)
        .set_pins(pin.id().num, 1)
        .build(sm);

    sm.set_pindirs([(pin.id().num, rp_pico::hal::pio::PinDir::Output)]);
    sm.start();

    tx.write((133_000_000 / (2 * freq)) - 3);
}

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

    let program_with_defines = pio_proc::pio_file!(
        "examples/pio_blink/blink.pio",
        select_program("blink"),
        options(max_program_size = 32)
    );
    let program = program_with_defines.program;

    let (mut pio, sm0, sm1, sm2, _) = pac.PIO0.split(&mut pac.RESETS);
    info!("Loaded pio program\n");

    blink_pin_forever(&mut pio, &program, sm0, pins.gpio2.into_function(), 3);
    blink_pin_forever(&mut pio, &program, sm1, pins.gpio6.into_function(), 4);
    blink_pin_forever(&mut pio, &program, sm2, pins.gpio11.into_function(), 1);

    PioStateCopy::assert_eq(EXPECTED_PIO);
    SmStateCopy::assert_eq(SM0_BASE, EXPECTED_SM);
    // SmStateCopy::assert_eq(SM1_BASE, EXPECTED_SM);
    // SmStateCopy::assert_eq(SM2_BASE, EXPECTED_SM);

    loop {}
}
