#![no_std]
#![no_main]

use bsp::entry;
use core::fmt;
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

const EXPECTED_SM: &'static str = r###"{
  "sm_clkdiv": "00000000001001110001000000000000",
  "sm_execctrl": "01110000000000011111110100000000",
  "sm_shiftctrl": "00100000100000110000000000000000",
  "sm_addr": "00000000000000000000000000011010",
  "sm_instr": "00000000000000000110000000100110",
  "sm_pinctrl": "01000100000110000100011000010000"
}"###;

const EXPECTED_PIO: &'static str = r###"{
  "ctrl": "00000000000000000000000000000000",
  "fstat": "00001111000000000000111100000000",
  "fdebug": "00000000000000000000000000000000",
  "flevel": "00000000000000000000000000000000",
  "irq": "00000000000000000000000000000000",
  "dbg_padout": "00000000000000000000000000000000",
  "dbg_padoe": "00000000000000110000000000000000",
  "dbg_cfginfo": "00000000001000000000010000000100"
}"###;

const PIO_I2C_ICOUNT_LSB: u16 = 10;
const PIO_I2C_FINAL_LSB: u16 = 9;
const PIO_I2C_DATA_LSB: u16 = 1;
const PIO_I2C_NAK_LSB: u16 = 0;

const I2C_SC0_SD0: u16 = 63360;
const I2C_SC0_SD1: u16 = 63361;
const I2C_SC1_SD0: u16 = 65408;
const I2C_SC1_SD1: u16 = 65409;

#[entry]
fn main() -> ! {
    let code = pio_proc::pio_file!(
        "examples/i2c/i2c.pio",
        select_program("set_scl_sda"),
        options(max_program_size = 32)
    )
    .program
    .code;
    defmt::assert_eq!(I2C_SC0_SD0, code[0]);
    defmt::assert_eq!(I2C_SC0_SD1, code[1]);
    defmt::assert_eq!(I2C_SC1_SD0, code[2]);
    defmt::assert_eq!(I2C_SC1_SD1, code[3]);

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
        "examples/i2c/i2c.pio",
        select_program("i2c"),
        options(max_program_size = 32)
    );
    let i2c_offset_entry_point = program_with_defines.public_defines.entry_point;
    let program = program_with_defines.program;

    let (mut p, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);

    let mut pin_sda: rp_pico::hal::gpio::Pin<
        rp_pico::hal::gpio::bank0::Gpio16,
        FunctionPio0,
        rp_pico::hal::gpio::PullDown,
    > = pins.gpio16.into_function();
    let mut pin_scl: rp_pico::hal::gpio::Pin<
        rp_pico::hal::gpio::bank0::Gpio17,
        FunctionPio0,
        rp_pico::hal::gpio::PullDown,
    > = pins.gpio17.into_function();
    let installed_program = p.install(&program).unwrap();
    let offset = installed_program.offset();
    let (mut sm, mut rx, mut tx) = rp_pico::hal::pio::PIOBuilder::from_program(installed_program)
        .buffers(rp_pico::hal::pio::Buffers::RxTx)
        .autopush(true)
        .autopull(true)
        .in_shift_direction(rp_pico::hal::pio::ShiftDirection::Left)
        .out_shift_direction(rp_pico::hal::pio::ShiftDirection::Left)
        .pull_threshold(16)
        .push_threshold(8)
        .in_pin_base(pin_sda.id().num)
        .jmp_pin(pin_sda.id().num)
        .set_pins(pin_sda.id().num, 1)
        .out_pins(pin_sda.id().num, 1)
        .side_set_pin_base(pin_scl.id().num)
        .build(sm0);

    sm.set_pindirs([
        (pin_sda.id().num, rp_pico::hal::pio::PinDir::Output),
        (pin_scl.id().num, rp_pico::hal::pio::PinDir::Output),
    ]);
    sm.set_pins([
        (pin_sda.id().num, rp_pico::hal::pio::PinState::Low),
        (pin_scl.id().num, rp_pico::hal::pio::PinState::Low),
    ]);
    pin_sda.set_output_override(rp_pico::hal::gpio::OutputOverride::Invert);
    pin_scl.set_output_override(rp_pico::hal::gpio::OutputOverride::Invert);

    sm.set_clock_divisor(39.0625);

    p.irq0().disable_sm_interrupt(0);
    p.irq1().disable_sm_interrupt(0);

    p.clear_irq(0xff);

    sm.exec_instruction(pio::Instruction {
        operands: pio::InstructionOperands::JMP {
            condition: pio::JmpCondition::Always,
            address: offset + i2c_offset_entry_point as u8,
        },
        delay: 0,
        side_set: None,
    });

    SmStateCopy::assert_eq(SM0_BASE, EXPECTED_SM);
    PioStateCopy::assert_eq(EXPECTED_PIO);

    let mut sm = sm.start();

    println!("\nPIO I2C Bus Scan");
    println!("   0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F");

    let mut line: heapless::String<128> = heapless::String::new();
    for addr in 0..((1 << 7) as u8) {
        if addr % 16 == 0 {
            fmt::write(&mut line, format_args!(" {:2x}", addr)).unwrap();
        }

        let result = if (addr & 0x78) == 0 || (addr & 0x78) == 0x78 {
            -1
        } else {
            let mut err: i32 = 0;
            pio_i2c_start(&p, &mut tx);

            while !rx.is_empty() {
                rx.read().unwrap();
            }

            pio_i2c_put16(&mut tx, ((addr as u16) << 2) | 3);
            let mut tx_remain: u32 = 0;

            let mut first: bool = true;

            while tx_remain != 0 && p.get_irq_raw() == 0 {
                if tx_remain != 0 && !tx.is_full() {
                    tx_remain -= 1;

                    if tx_remain == 0 {
                        pio_i2c_put16(
                            &mut tx,
                            (0xff << 1) | (1 << PIO_I2C_FINAL_LSB) | (1 << PIO_I2C_NAK_LSB),
                        );
                    } else {
                        pio_i2c_put16(&mut tx, (0xff << 1) | (0));
                    }
                }
                if !rx.is_empty() {
                    if first {
                        rx.read().unwrap();
                        first = false;
                    }
                }
            }
            pio_i2c_stop(&p, &mut tx);
            pio_i2c_wait_idle(&p, &mut tx);

            if p.get_irq_raw() != 0 {
                err = -1;
                pio_i2c_resume_after_error(&p, &mut tx, &mut sm);
                pio_i2c_stop(&p, &mut tx);
            }

            err
        };

        if result < 0 {
            line.push('.').unwrap();
        } else {
            line.push('@').unwrap();
        }

        if addr % 16 == 15 {
            println!("{}", line);
            line.clear();
        } else {
            line.push_str("  ").unwrap();
        }
    }

    println!("Done.\n");
    loop {}
}

fn pio_i2c_stop<SM>(
    pio: &rp_pico::hal::pio::PIO<rp_pico::pac::PIO0>,
    tx: &mut rp_pico::hal::pio::Tx<SM>,
) where
    SM: rp_pico::hal::pio::ValidStateMachine,
{
    pio_i2c_put_or_err(pio, tx, 2 << PIO_I2C_ICOUNT_LSB);
    pio_i2c_put_or_err(pio, tx, I2C_SC1_SD0);
    pio_i2c_put_or_err(pio, tx, I2C_SC0_SD0);
    pio_i2c_put_or_err(pio, tx, I2C_SC1_SD1);
}

fn pio_i2c_start<SM>(
    p: &rp_pico::hal::pio::PIO<rp_pico::pac::PIO0>,
    tx: &mut rp_pico::hal::pio::Tx<SM>,
) where
    SM: rp_pico::hal::pio::ValidStateMachine,
{
    pio_i2c_put_or_err(p, tx, 1 << PIO_I2C_ICOUNT_LSB);
    pio_i2c_put_or_err(p, tx, I2C_SC1_SD0);
    pio_i2c_put_or_err(p, tx, I2C_SC0_SD0);
}

fn pio_i2c_put16<SM>(tx: &mut rp_pico::hal::pio::Tx<SM>, data: u16)
where
    SM: rp_pico::hal::pio::ValidStateMachine,
{
    while tx.is_full() {}
    tx.write(data as u32);
}

fn pio_i2c_wait_idle<SM>(
    pio: &rp_pico::hal::pio::PIO<rp_pico::pac::PIO0>,
    tx: &mut rp_pico::hal::pio::Tx<SM>,
) where
    SM: rp_pico::hal::pio::ValidStateMachine,
{
    // Finished when TX runs dry or SM hits an IRQ
    tx.clear_stalled_flag();

    while !(tx.has_stalled() || pio.get_irq_raw() != 0) {}
}

// If I2C is ok, block and push data. Otherwise fall straight through.
fn pio_i2c_put_or_err<SM>(
    pio: &rp_pico::hal::pio::PIO<rp_pico::pac::PIO0>,
    tx: &mut rp_pico::hal::pio::Tx<SM>,
    data: u16,
) where
    SM: rp_pico::hal::pio::ValidStateMachine,
{
    while tx.is_full() {
        if pio.get_irq_raw() != 0 {
            return;
        }
    }
    if pio.get_irq_raw() != 0 {
        return;
    }

    tx.write(data.into());
}

fn pio_i2c_resume_after_error<SM, State>(
    pio: &rp_pico::hal::pio::PIO<rp_pico::pac::PIO0>,
    tx: &mut rp_pico::hal::pio::Tx<SM>,
    sm: &mut rp_pico::hal::pio::StateMachine<SM, State>,
) where
    SM: rp_pico::hal::pio::ValidStateMachine,
{
    sm.drain_tx_fifo();
    // sm.exec_instruction(0);
    // pio_sm_exec(
    //     pio,
    //     sm,
    //     (pio->sm[sm].execctrl & PIO_SM0_EXECCTRL_WRAP_BOTTOM_BITS) >> PIO_SM0_EXECCTRL_WRAP_BOTTOM_LSB
    // );
    pio.clear_irq(0);
}
