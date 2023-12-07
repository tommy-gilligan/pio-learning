#![no_std]
#![no_main]

use bsp::entry;
use defmt::*;
use defmt_rtt as _;
use panic_probe as _;
use rp2040_project_template::{PioStateCopy, SmStateCopy, SM0_BASE, SM1_BASE};

use cortex_m::prelude::*;
use rp_pico as bsp;

use bsp::hal::gpio::{FunctionPio0, FunctionSpi, Pin};
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

const BUF_SIZE: usize = 8;

const EXPECTED_SM0: &'static str = r###"{
  "sm_clkdiv": "00000000000000010000000000000000",
  "sm_execctrl": "00000000000000011111111010000000",
  "sm_shiftctrl": "10000000100010010000000000000000",
  "sm_addr": "00000000000000000000000000011101",
  "sm_instr": "00000000000000000010000000100001",

  "sm_pinctrl": "00000000000000100000000000000000"
}"###;

const EXPECTED_PIO: &'static str = r###"{
  "ctrl": "00000000000000000000000000000000",
  "fstat": "00001111000000010000111100000000",
  "fdebug": "00000000000000000000000000000000",
  "flevel": "00000000000000000000000000000000",
  "irq": "00000000000000000000000000000000",
  "dbg_padout": "00000000000000000000000000000000",
  "dbg_padoe": "00000000000000000000000000000000",
  "dbg_cfginfo": "00000000001000000000010000000100"
}"###;

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
    let mut small_rng = SmallRng::seed_from_u64(0x69420);

    // These are implicitly used by the spi driver if they are in the correct mode
    let spi_mosi = pins
        .gpio3
        .into_function::<rp_pico::hal::gpio::FunctionSpi>();
    let spi_miso = pins
        .gpio16
        .into_function::<rp_pico::hal::gpio::FunctionSpi>();
    let spi_sclk = pins
        .gpio2
        .into_function::<rp_pico::hal::gpio::FunctionSpi>();
    let spi = rp_pico::hal::spi::Spi::<_, _, _, 8>::new(pac.SPI0, (spi_mosi, spi_miso, spi_sclk));
    use rp_pico::hal::fugit::RateExtU32;

    let pin_4: rp_pico::hal::gpio::Pin<
        rp_pico::hal::gpio::bank0::Gpio4,
        FunctionPio0,
        rp_pico::hal::gpio::PullDown,
    > = pins.gpio4.into_function();
    let pin_5: rp_pico::hal::gpio::Pin<
        rp_pico::hal::gpio::bank0::Gpio5,
        FunctionPio0,
        rp_pico::hal::gpio::PullDown,
    > = pins.gpio5.into_function();

    // Exchange the uninitialised SPI driver for an initialised one
    let mut spi = spi.init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        1.MHz(),
        embedded_hal::spi::MODE_0,
    );

    let program_with_defines = pio_proc::pio_file!(
        "examples/clocked_input/clocked_input.pio",
        select_program("clocked_input"),
        options(max_program_size = 32)
    );
    let program = program_with_defines.program;
    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let program = pio.install(&program).unwrap();
    let (mut sm, mut rx, _) = rp_pico::hal::pio::PIOBuilder::from_program(program)
        .autopush(true)
        .push_threshold(8)
        .out_shift_direction(rp_pico::hal::pio::ShiftDirection::Right)
        .in_shift_direction(rp_pico::hal::pio::ShiftDirection::Left)
        .autopull(false)
        .buffers(rp_pico::hal::pio::Buffers::OnlyTx)
        .in_pin_base(pin_4.id().num)
        .set_pins(0, 0)
        .build(sm0);

    sm.set_pindirs([
        (pin_4.id().num, rp_pico::hal::pio::PinDir::Input),
        (pin_5.id().num, rp_pico::hal::pio::PinDir::Input),
    ]);

    SmStateCopy::assert_eq(SM0_BASE, EXPECTED_SM0);
    PioStateCopy::assert_eq(EXPECTED_PIO);
    sm.start();

    let mut txbuf: [u8; BUF_SIZE] = [23; BUF_SIZE];
    println!("Data to transmit:");
    for i in 0..BUF_SIZE {
        txbuf[i] = small_rng.next_u32() as u8;
        println!("{:x}", txbuf[i]);
    }

    spi.write(&txbuf).unwrap();

    println!("Reading back from RX FIFO:");
    for i in 0..BUF_SIZE {
        while rx.is_empty() {}
        let rxdata = rx.read().unwrap();
        if rxdata as u8 == txbuf[i] {
            println!("{:x} OK", rxdata);
        } else {
            println!("{:x} FAIL", rxdata);
        }
    }
    println!("Done.");

    loop {
        delay.delay_ms(10);
    }
}
