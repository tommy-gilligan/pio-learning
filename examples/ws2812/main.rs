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

const EXPECTED_SM: &'static str = "{\"sm_clkdiv\":983040,\"sm_execctrl\":130560,\"sm_shiftctrl\":1879441408,\"sm_addr\":28,\"sm_instr\":25121,\"sm_pinctrl\":536872960}";
const EXPECTED_PIO: &'static str = "{\"ctrl\":1,\"fstat\":251662081,\"fdebug\":16777216,\"flevel\":0,\"irq\":0,\"dbg_padout\":0,\"dbg_padoe\":4,\"dbg_cfginfo\":2098180}";

#[inline]
fn put_pixel<P, S>(tx: &mut rp_pico::hal::pio::Tx<(P, S)>, pixel_grb: u32) where P: rp_pico::hal::pio::PIOExt, S: rp_pico::hal::pio::StateMachineIndex {
    tx.write(pixel_grb << 8);
}

#[inline]
fn urgb_u32(r: u8, g: u8, b: u8) -> u32 {
    ((r as u32) << 8) | ((g as u32) << 16) | (b as u32)
}

fn pattern_greys<P, S>(tx: &mut rp_pico::hal::pio::Tx<(P, S)>, len: usize, mut t: usize) where P: rp_pico::hal::pio::PIOExt, S: rp_pico::hal::pio::StateMachineIndex {
    let max: usize = 100; // let's not draw too much current!
    t %= max;

    for i in 0..len {
        put_pixel(tx, t as u32 * 0x10101);

        if t >= max {
            t = 0
        } else {
            t += 1;
        };
    }
}

fn pattern_snakes<P, S>(tx: &mut rp_pico::hal::pio::Tx<(P, S)>, len: usize, t: usize) where P: rp_pico::hal::pio::PIOExt, S: rp_pico::hal::pio::StateMachineIndex {
    for i in 0..len {
        let x: usize = (i + (t >> 1)) % 64;

        if x < 10 {
            put_pixel(tx, urgb_u32(0xff, 0, 0));
        } else if x >= 15 && x < 25 {
            put_pixel(tx, urgb_u32(0, 0xff, 0));
        } else if x >= 30 && x < 40 {
            put_pixel(tx, urgb_u32(0, 0, 0xff));
        } else {
            put_pixel(tx, 0);
        }
    }
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
    let pin: rp_pico::hal::gpio::Pin<
        rp_pico::hal::gpio::bank0::Gpio2,
        FunctionPio0,
        rp_pico::hal::gpio::PullDown
    > = pins.gpio2.into_function();

    let program_with_defines = pio_proc::pio_file!(
        "examples/ws2812/ws2812.pio",
        select_program("ws2812"),
        options(max_program_size = 32)
    );
    let program = program_with_defines.program;

    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);

    let (mut sm, _, mut tx) = rp_pico::hal::pio::PIOBuilder::from_program(pio.install(&program).unwrap())
        .autopush(false)
        .autopull(true)
        .pull_threshold(24)
        .out_shift_direction(rp_pico::hal::pio::ShiftDirection::Left)
        .in_shift_direction(rp_pico::hal::pio::ShiftDirection::Right)
        .out_pins(0, 0)
        .set_pins(0, 0)
        .side_set_pin_base(pin.id().num)
        .buffers(rp_pico::hal::pio::Buffers::OnlyTx)
        .build(sm0);

    // let cycles_per_bit: i32 = program_with_defines.public_defines.T1 + program_with_defines.public_defines.T2 + program_with_defines.public_defines.T3;
    sm.clock_divisor_fixed_point(15, 0);
    sm.set_pindirs([(pin.id().num, rp_pico::hal::pio::PinDir::Output)]);
    sm.start();

    PioStateCopy::assert_eq(EXPECTED_PIO);
    SmStateCopy::assert_eq(SM0_BASE, EXPECTED_SM);

    loop {
        for i in 0..1000 {
            pattern_snakes(&mut tx, 24, i);
            delay.delay_ms(10);
        }
    }
}
