#![no_std]
#![no_main]
#![feature(ascii_char)]

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
use rp_pico::hal::multicore::{Multicore, Stack};
use rp_pico::hal::pio::PIOExt;

use rp_pico::hal::fugit::RateExtU32;
use rp_pico::hal::uart::{DataBits, StopBits, UartConfig};

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

const EXPECTED_SM: &'static str = r###"{
  "sm_clkdiv": "00000000100001111010001000000000",
  "sm_execctrl": "00000011000000011111101110000000",
  "sm_shiftctrl": "10000000000011000000000000000000",
  "sm_addr": "00000000000000000000000000010111",
  "sm_instr": "00000000000000000010000000100000",
  "sm_pinctrl": "00000000000000011000000000000000"
}"###;

const HELLO_WORLD: &'static [u8] =
    b"Hello, world from PIO! (Plus 2 UARTs and 2 cores, for complex reasons)\n";

static mut CORE1_STACK: Stack<4096> = Stack::new();

fn core1_task(sys_freq: u32) -> ! {
    let mut pac = unsafe { pac::Peripherals::steal() };
    let core = unsafe { pac::CorePeripherals::steal() };

    let mut sio = Sio::new(pac.SIO);
    let external_xtal_freq_hz = 12_000_000u32;
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
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
    let pins = rp_pico::hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let uart_pins = (
        // UART TX (characters sent from RP2040) (GPIO4)
        pins.gpio4.into_function(),
        // UART RX (characters received by RP2040) (GPIO5)
        pins.gpio5.into_function(),
    );
    let uart = rp_pico::hal::uart::UartPeripheral::new(pac.UART1, uart_pins, &mut pac.RESETS)
        .enable(
            UartConfig::new(115200.Hz(), DataBits::Eight, None, StopBits::One),
            clocks.peripheral_clock.freq(),
        )
        .unwrap();

    let address = sio.fifo.read_blocking() as *const &'static [u8];
    unsafe {
        uart.write_full_blocking(*address);
    }

    loop {}
}

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let mut sio = Sio::new(pac.SIO);

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
    let sys_freq = clocks.system_clock.freq().to_Hz();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    let pin: rp_pico::hal::gpio::Pin<
        rp_pico::hal::gpio::bank0::Gpio3,
        FunctionPio0,
        rp_pico::hal::gpio::PullDown,
    > = pins.gpio3.into_function();

    let program_with_defines = pio_proc::pio_file!(
        "examples/uart_rx/uart_rx.pio",
        select_program("uart_rx"),
        options(max_program_size = 32)
    );
    let program = program_with_defines.program;

    println!("Starting PIO UART RX example");

    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);

    let (mut sm, mut rx, _) =
        rp_pico::hal::pio::PIOBuilder::from_program(pio.install(&program).unwrap())
            .buffers(rp_pico::hal::pio::Buffers::OnlyRx)
            .in_shift_direction(rp_pico::hal::pio::ShiftDirection::Right)
            .autopush(false)
            .push_threshold(32)
            .autopull(false)
            .out_shift_direction(rp_pico::hal::pio::ShiftDirection::Right)
            .in_pin_base(pin.id().num)
            .jmp_pin(pin.id().num)
            .set_pins(0, 0)
            .out_pins(0, 0)
            .build(sm0);

    // should this be 125???
    sm.set_clock_divisor(125_000_000f32 / (8f32 * 115200f32));
    sm.set_pindirs([(pin.id().num, rp_pico::hal::pio::PinDir::Input)]);

    SmStateCopy::assert_eq(SM0_BASE, EXPECTED_SM);
    PioStateCopy::assert_eq(EXPECTED_PIO);

    sm.start();

    let mut mc = Multicore::new(&mut pac.PSM, &mut pac.PPB, &mut sio.fifo);
    let cores = mc.cores();
    let core1 = &mut cores[1];
    let _test = core1.spawn(unsafe { &mut CORE1_STACK.mem }, move || {
        core1_task(sys_freq)
    });

    let address = &HELLO_WORLD as *const &'static [u8];
    sio.fifo.write_blocking(address as u32);
    let mut s: heapless::String<512> = heapless::String::new();

    loop {
        while rx.is_empty() {}
        let bytes = rx.read().unwrap().to_ne_bytes();
        let char = core::str::from_utf8(&bytes)
            .unwrap()
            .chars()
            .last()
            .unwrap();
        if char == '\n' {
            println!("{}", s);
        } else {
            s.push(char).unwrap();
        }
    }
}
