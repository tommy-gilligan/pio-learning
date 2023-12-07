#![no_std]
#![no_main]

use bsp::entry;
use defmt::*;
use defmt_rtt as _;
use panic_probe as _;
use rp2040_project_template::{PioStateCopy, SmStateCopy, SM0_BASE, SM1_BASE};
use cortex_m::prelude::*;

use cortex_m::singleton;

use rp_pico as bsp;

use bsp::hal::gpio::{FunctionPio0, FunctionPwm, Pin};
use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac,
    sio::Sio,
    watchdog::Watchdog,
};
use rp_pico::hal::pio::PIOExt;
use rp_pico::hal::dma::{double_buffer, single_buffer, DMAExt};

const EXPECTED_SM0: &'static str = r###"{
  "sm_clkdiv": "00000000000000010000000000000000",
  "sm_execctrl": "10000000000000011111111110000000",
  "sm_shiftctrl": "10000000000011010000000000000000",
  "sm_addr": "00000000000000000000000000011111",
  "sm_instr": "00000000000000000100000000000010",
  "sm_pinctrl": "00000000000010000000000000000000"
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

#[inline]
fn bits_packed_per_word(pin_count: u32) -> u32 {
    32 - (32 % pin_count)
}

const CAPTURE_PIN_BASE: u32 = 16;
const CAPTURE_PIN_COUNT: u8 = 2;
const CAPTURE_N_SAMPLES: usize = 96;
const TOTAL_SAMPLE_BITS: u32 = (CAPTURE_N_SAMPLES as u32 * CAPTURE_PIN_COUNT as u32) + (32 - (32 % CAPTURE_PIN_COUNT as u32)) - 1;
const BUF_SIZE_WORDS: usize = (TOTAL_SAMPLE_BITS / 32 - (32 % CAPTURE_PIN_COUNT as u32)) as usize;

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

    let mut capture_buf = singleton!(: [u32; BUF_SIZE_WORDS] = [0; BUF_SIZE_WORDS]).unwrap();

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    let pin_a: rp_pico::hal::gpio::Pin<
        rp_pico::hal::gpio::bank0::Gpio16,
        FunctionPwm,
        rp_pico::hal::gpio::PullDown,
    > = pins.gpio16.into_function();
    let pin_b: rp_pico::hal::gpio::Pin<
        rp_pico::hal::gpio::bank0::Gpio17,
        FunctionPwm,
        rp_pico::hal::gpio::PullDown,
    > = pins.gpio17.into_function();

    println!("Starting PWM example");
    let mut pwm_slices = rp_pico::hal::pwm::Slices::new(pac.PWM, &mut pac.RESETS);
    let pwm = &mut pwm_slices.pwm0;
    pwm.set_top(3);
    pwm.set_div_int(4);
    let channel_a = &mut pwm.channel_a;
    channel_a.set_duty(1);
    let channel_b = &mut pwm.channel_b;
    channel_b.set_duty(3);
    pwm.enable();

    let mut a = pio::Assembler::<{ pio::RP2040_MAX_PROGRAM_SIZE }>::new();
    a.r#in(pio::InSource::PINS, CAPTURE_PIN_COUNT);
    let program = a.assemble_program();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);

    let program = pio.install(&program).unwrap();
    let program = program.set_wrap(pio::Wrap { source: 0, target: 0 }).unwrap();
    let offset = program.offset();
    let (mut sm, rx, _) =
        rp_pico::hal::pio::PIOBuilder::from_program(program)
            .buffers(rp_pico::hal::pio::Buffers::OnlyRx)
            .autopush(true)
            .in_shift_direction(rp_pico::hal::pio::ShiftDirection::Right)
            .push_threshold(bits_packed_per_word(CAPTURE_PIN_COUNT as u32) as u8)
            .in_pin_base(CAPTURE_PIN_BASE as u8)
            .autopull(false)
            .out_shift_direction(rp_pico::hal::pio::ShiftDirection::Right)
            .out_pins(0, 0)
            .set_pins(0, 0)
            .build(sm0);

    sm.set_clock_divisor(1f32);

    println!("Arming trigger");
    sm.clear_fifos();
    sm.drain_tx_fifo();
    let dma = pac.DMA.split(&mut pac.RESETS);
    let d = single_buffer::Config::new(dma.ch0, rx, capture_buf).start();

    // sm.exec_instruction(pio::Instruction {
    //     operands: pio::InstructionOperands::JMP {
    //         condition: pio::JmpCondition::Always,
    //         address: offset
    //     },
    //     delay: 0,
    //     side_set: None,
    // });
    sm.exec_instruction(pio::Instruction {
        operands: pio::InstructionOperands::WAIT {
            polarity: 1,
            source: pio::WaitSource::GPIO,
            index: CAPTURE_PIN_BASE as u8,
            relative: false,
        },
        delay: 0,
        side_set: None,
    });

    // SmStateCopy::assert_eq(SM0_BASE, EXPECTED_SM0);
    PioStateCopy::assert_eq(EXPECTED_PIO);
    sm.start();

    let (_, _, rx_buf) = d.wait();

    // Display the capture buffer in text form, like this:
    // 00: __--__--__--__--__--__--
    // 01: ____----____----____----
    println!("Capture:");
    // Each FIFO record may be only partially filled with bits, depending on
    // whether pin_count is a factor of 32.
    let record_size_bits: usize = bits_packed_per_word(CAPTURE_PIN_COUNT as u32) as usize;
    for pin in 0..CAPTURE_PIN_COUNT {
        let mut sparkline: heapless::String<CAPTURE_N_SAMPLES> = heapless::String::new();

        println!("{}: ", pin + CAPTURE_PIN_BASE as u8);
        for sample in 0..CAPTURE_N_SAMPLES {
            let bit_index: usize = pin as usize + sample as usize * CAPTURE_PIN_COUNT as usize;
            let word_index: usize = bit_index / record_size_bits;
            // Data is left-justified in each FIFO entry, hence the (32 - record_size_bits) offset
            let word_mask: u32 = 1 << (bit_index % record_size_bits + 32 - record_size_bits);
            if (rx_buf[word_index] & word_mask) != 0 {
                sparkline.push('-').unwrap();
            } else {
                sparkline.push('_').unwrap();
            }
        }

        println!("{:?}", sparkline);
    }

    loop {
        delay.delay_ms(10);
    }
}
