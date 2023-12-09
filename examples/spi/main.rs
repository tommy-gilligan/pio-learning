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
  "dbg_padoe": "00000000000001010000000000000000",
  "dbg_cfginfo": "00000000001000000000010000000100"
}"###;

const EXPECTED_SM: &'static str = r###"{
  "sm_clkdiv": "00000000000111110100000000000000",
  "sm_execctrl": "00000000000000011111111100000000",
  "sm_shiftctrl": "00010000100000110000000000000000",
  "sm_addr": "00000000000000000000000000011110",
  "sm_instr": "00000000000000000110000100000001",
  "sm_pinctrl": "00100000000110000100100000010000"
}"###;

const BUF_SIZE: usize = 20;

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
    let pin_mosi_miso: rp_pico::hal::gpio::Pin<
        rp_pico::hal::gpio::bank0::Gpio16,
        FunctionPio0,
        rp_pico::hal::gpio::PullDown,
    > = pins.gpio16.into_function();
    let mut pin_sck: rp_pico::hal::gpio::Pin<
        rp_pico::hal::gpio::bank0::Gpio18,
        FunctionPio0,
        rp_pico::hal::gpio::PullDown,
    > = pins.gpio18.into_function();

    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);

    let cpha = 0;
    let cpol = 0;

    println!("CPHA = {}, CPOL = {}\n", cpha, cpol);
    let cpha_with_defines = pio_proc::pio_file!(
        "examples/spi/spi.pio",
        select_program("spi_cpha1"),
        options(max_program_size = 32)
    );
    let no_cpha_with_defines = pio_proc::pio_file!(
        "examples/spi/spi.pio",
        select_program("spi_cpha0"),
        options(max_program_size = 32)
    );

    let program = if cpha != 0 {
        cpha_with_defines.program
    } else {
        no_cpha_with_defines.program
    };

    let (mut sm, mut rx, mut tx) =
        rp_pico::hal::pio::PIOBuilder::from_program(pio.install(&program).unwrap())
            .pull_threshold(8)
            .push_threshold(8)
            .autopush(true)
            .autopull(true)
            .out_shift_direction(rp_pico::hal::pio::ShiftDirection::Left)
            .in_shift_direction(rp_pico::hal::pio::ShiftDirection::Left)
            .in_pin_base(pin_mosi_miso.id().num)
            .out_pins(pin_mosi_miso.id().num, 1)
            .side_set_pin_base(pin_sck.id().num)
            .set_pins(0, 0)
            .buffers(rp_pico::hal::pio::Buffers::RxTx)
            .build(sm0);

    sm.set_clock_divisor(31.25f32);
    sm.set_pindirs(
        [
            (pin_mosi_miso.id().num, rp_pico::hal::pio::PinDir::Output),
            (pin_sck.id().num, rp_pico::hal::pio::PinDir::Output),
        ]
    );
    sm.set_pins(
        [
            (pin_mosi_miso.id().num, rp_pico::hal::pio::PinState::Low),
            (pin_sck.id().num, rp_pico::hal::pio::PinState::Low),
        ]
    );

    if cpol != 0 {
        pin_sck.set_output_override(rp_pico::hal::gpio::OutputOverride::Invert);
    } else {
        pin_sck.set_output_override(rp_pico::hal::gpio::OutputOverride::DontInvert);
    }

    PioStateCopy::assert_eq(EXPECTED_PIO);
    SmStateCopy::assert_eq(SM0_BASE, EXPECTED_SM);

    unsafe {
        core::ptr::write_volatile(0x50200038 as *mut u32, 1 << pin_mosi_miso.id().num);
    }
    sm.start();

    let mut txbuf: [u8; BUF_SIZE] = [0; BUF_SIZE];
    let mut rxbuf: [u8; BUF_SIZE] = [0; BUF_SIZE];

    let mut small_rng = SmallRng::seed_from_u64(0x69420);

    for i in 0..BUF_SIZE {
        txbuf[i] = small_rng.next_u32() as u8;
        rxbuf[i] = 0;
    }
    println!("TX: {:?}", txbuf);

    for (s, d) in txbuf.iter().zip(rxbuf.iter_mut()) {
        while tx.is_full() {
        }
        tx.write(*s as u32);

        while rx.is_empty() {
        }
        *d = rx.read().unwrap() as u8;
    }

    println!("RX: {:?}", rxbuf);

    if rxbuf.iter().zip(txbuf.iter()).any(|(r, t)| r != t) {
        println!("\nNope\n");
    } else {
        println!("\nOK\n");
    }

    loop {
    }
}
