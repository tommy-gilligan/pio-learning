#![no_std]
#![no_main]

use bsp::entry;
use defmt::*;
use defmt_rtt as _;
use panic_probe as _;
use rp2040_project_template::{PioStateCopy, SmStateCopy, SM0_BASE, SM1_BASE, SM2_BASE};

use rp_pico as bsp;

use bsp::hal::gpio::FunctionPio0;
use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac,
    sio::Sio,
    watchdog::Watchdog,
};
use rp_pico::hal::pio::PIOExt;

const EXPECTED_PIO: &'static str = r###"{
  "ctrl": "00000000000000000000000000000011",
  "fstat": "00001111000001000000111100000010",
  "fdebug": "00000010000000000000000000000000",
  "flevel": "00000000000000000000000000000000",
  "irq": "00000000000000000000000000000000",
  "dbg_padout": "00000000000000000000000000000000",
  "dbg_padoe": "00000000000000000100000000000000",
  "dbg_cfginfo": "00000000001000000000010000000100"
}"###;

const EXPECTED_SM_0: &'static str = r###"{
  "sm_clkdiv": "00000011001100011001011100000000",
  "sm_execctrl": "00000000000000011111110110000000",
  "sm_shiftctrl": "00000000000011000000000000000000",
  "sm_addr": "00000000000000000000000000011011",
  "sm_instr": "00000000000000001110000000110100",
  "sm_pinctrl": "00000100000000000000000111000000"
}"###;

const EXPECTED_SM_1: &'static str = r###"{
  "sm_clkdiv": "10001001010101000100000000000000",
  "sm_execctrl": "00000000000000011010100000000000",
  "sm_shiftctrl": "01000000000011000000000000000000",
  "sm_addr": "00000000000000000000000000010000",
  "sm_instr": "00000000000000001000000010100000",
  "sm_pinctrl": "00000000000000000000000000000000"
}"###;

const EXPECTED_SM_2: &'static str = r###"{
  "sm_clkdiv": "00011011011101110100000000000000",
  "sm_execctrl": "00001111000000001111001110000000",
  "sm_shiftctrl": "10000000000011010000000000000000",
  "sm_addr": "00000000000000000000000000000111",
  "sm_instr": "00000000000000001110000000111110",
  "sm_pinctrl": "00000000000001111000000000000000"
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
    let tx_pin: rp_pico::hal::gpio::Pin<
        rp_pico::hal::gpio::bank0::Gpio14,
        FunctionPio0,
        rp_pico::hal::gpio::PullDown,
    > = pins.gpio14.into_function();

    // TODO: needs to disable pull
    let rx_pin: rp_pico::hal::gpio::Pin<
        rp_pico::hal::gpio::bank0::Gpio15,
        FunctionPio0,
        rp_pico::hal::gpio::PullDown,
    > = pins.gpio15.into_function();

    let program_with_defines = pio_proc::pio_file!(
        "examples/ir_nec/nec_carrier_burst.pio",
        select_program("nec_carrier_burst"),
        options(max_program_size = 32)
    );
    let program = program_with_defines.program;

    let (mut pio, sm0, sm1, sm2, _) = pac.PIO0.split(&mut pac.RESETS);

    let (mut sm, _, _) =
        rp_pico::hal::pio::PIOBuilder::from_program(pio.install(&program).unwrap())
            .out_shift_direction(rp_pico::hal::pio::ShiftDirection::Right)
            .in_shift_direction(rp_pico::hal::pio::ShiftDirection::Right)
            .set_pins(tx_pin.id().num, 1)
            .build(sm0);

    sm.set_clock_divisor(
        125_000_000.0 / (38.222e3 * program_with_defines.public_defines.TICKS_PER_LOOP as f32),
    );
    sm.set_pindirs([(tx_pin.id().num, rp_pico::hal::pio::PinDir::Output)]);
    sm.set_pins([(tx_pin.id().num, rp_pico::hal::pio::PinState::Low)]);

    SmStateCopy::assert_eq(SM0_BASE, EXPECTED_SM_0);
    // PioStateCopy::assert_eq(EXPECTED_PIO);

    sm.start();

    let program_with_defines = pio_proc::pio_file!(
        "examples/ir_nec/nec_carrier_control.pio",
        select_program("nec_carrier_control"),
        options(max_program_size = 32)
    );
    let program = program_with_defines.program;

    let (mut sm, _, mut tx) =
        rp_pico::hal::pio::PIOBuilder::from_program(pio.install(&program).unwrap())
            .buffers(rp_pico::hal::pio::Buffers::OnlyTx)
            .pull_threshold(32)
            .autopull(false)
            .out_shift_direction(rp_pico::hal::pio::ShiftDirection::Right)
            .in_shift_direction(rp_pico::hal::pio::ShiftDirection::Right)
            .set_pins(0, 0)
            .build(sm1);

    sm.set_clock_divisor(35156.25);

    SmStateCopy::assert_eq(SM1_BASE, EXPECTED_SM_1);
    // PioStateCopy::assert_eq(EXPECTED_PIO);
    sm.start();

    let program_with_defines = pio_proc::pio_file!(
        "examples/ir_nec/nec_receive.pio",
        select_program("nec_receive"),
        options(max_program_size = 32)
    );
    let program = program_with_defines.program;

    let (mut sm, mut rx, _) =
        rp_pico::hal::pio::PIOBuilder::from_program(pio.install(&program).unwrap())
            .buffers(rp_pico::hal::pio::Buffers::OnlyRx)
            .push_threshold(32)
            .autopush(true)
            .out_shift_direction(rp_pico::hal::pio::ShiftDirection::Right)
            .in_shift_direction(rp_pico::hal::pio::ShiftDirection::Right)
            .in_pin_base(rx_pin.id().num)
            .jmp_pin(rx_pin.id().num)
            .set_pins(0, 0)
            .build(sm2);

    sm.set_clock_divisor(7031.25);
    sm.set_pindirs([(rx_pin.id().num, rp_pico::hal::pio::PinDir::Input)]);

    SmStateCopy::assert_eq(SM2_BASE, EXPECTED_SM_2);
    PioStateCopy::assert_eq(EXPECTED_PIO);
    sm.start();

    let tx_address: u8 = 0x00;
    let mut tx_data: u8 = 0x00;

    loop {
        // create a 32-bit frame and add it to the transmit FIFO
        let tx_frame: u32 = (tx_address as u32)
            | (((tx_address as u32 ^ 0xff) << 8) as u32)
            | (((tx_data as u32) << 16) as u32)
            | (((tx_data as u32 ^ 0xff) << 24) as u32);

        tx.write(tx_frame);
        println!("\nsent: {:x}, {:x}", tx_address, tx_data);
        delay.delay_ms(100);

        while !rx.is_empty() {
            let rx_frame: RxFrame = unsafe {
                RxFrameWrapper {
                    raw: rx.read().unwrap(),
                }
                .decoded
            };

            if rx_frame.address != (rx_frame.inverted_address ^ 0xff)
                || rx_frame.data != (rx_frame.inverted_data ^ 0xff)
            {
                println!("\treceived: {:x}", rx_frame);
            } else {
                println!("\treceived: {:x}, {:x}", rx_frame.address, rx_frame.data);
            }
        }

        delay.delay_ms(900);
        tx_data += 1;
    }
}

use defmt::Format;

#[derive(Format, Debug, Copy, Clone)]
#[repr(C)]
struct RxFrame {
    pub address: u8,
    pub inverted_address: u8,
    pub data: u8,
    pub inverted_data: u8,
}

#[repr(C)]
union RxFrameWrapper {
    pub raw: u32,
    pub decoded: RxFrame,
}
