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

const EXPECTED_SM: &'static str = "{\"sm_clkdiv\":65536,\"sm_execctrl\":1073872000,\"sm_shiftctrl\":786432,\"sm_addr\":28,\"sm_instr\":41026,\"sm_pinctrl\":1073767424}";
const EXPECTED_PIO: &'static str = "{\"ctrl\":1,\"fstat\":251662080,\"fdebug\":16777216,\"flevel\":0,\"irq\":0,\"dbg_padout\":0,\"dbg_padoe\":33554432,\"dbg_cfginfo\":2098180}";

fn pio_pwm_set_period<T, S>(
    pio: &mut rp_pico::hal::pio::PIO<rp_pico::pac::PIO0>,
    program: &pio::Program<32>,
    sm: rp_pico::hal::pio::UninitStateMachine<(rp_pico::pac::PIO0, S)>,
    pin: rp_pico::hal::gpio::Pin<T, FunctionPio0, rp_pico::hal::gpio::PullDown>,
    freq: u32,
) -> rp_pico::hal::pio::Tx<(rp_pico::pac::PIO0, S)>
where
    T: rp_pico::hal::gpio::PinId,
    S: rp_pico::hal::pio::StateMachineIndex,
{
    let (mut sm, _, mut tx) =
        rp_pico::hal::pio::PIOBuilder::from_program(pio.install(&program).unwrap())
            .autopush(false)
            .autopull(false)
            .out_shift_direction(rp_pico::hal::pio::ShiftDirection::Right)
            .in_shift_direction(rp_pico::hal::pio::ShiftDirection::Right)
            .out_pins(0, 0)
            .set_pins(pin.id().num, 1)
            .side_set_pin_base(pin.id().num)
            .build(sm);
    sm.set_pindirs([(pin.id().num, rp_pico::hal::pio::PinDir::Output)]);

    while tx.is_full() {}
    tx.write(freq);

    sm.exec_instruction(pio::Instruction {
        operands: pio::InstructionOperands::PULL {
            if_empty: false,
            block: false,
        },
        delay: 0,
        side_set: None,
    });
    sm.exec_instruction(pio::Instruction {
        operands: pio::InstructionOperands::OUT {
            destination: pio::OutDestination::ISR,
            bit_count: 32,
        },
        delay: 0,
        side_set: None,
    });
    sm.start();

    tx
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
        "examples/pwm/pwm.pio",
        select_program("pwm"),
        options(max_program_size = 32)
    );
    let program = program_with_defines.program;

    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    info!("Loaded pio program\n");

    let mut tx = pio_pwm_set_period(
        &mut pio,
        &program,
        sm0,
        pins.led.into_function(),
        (1 << 16) - 1,
    );
    PioStateCopy::assert_eq(EXPECTED_PIO);
    // SmStateCopy::assert_eq(SM0_BASE, EXPECTED_SM);

    loop {
        for level in 0..256 {
            info!("Level = {}\n", level);
            while tx.is_full() {}
            tx.write(level * level);
            delay.delay_ms(10);
        }
    }
}
