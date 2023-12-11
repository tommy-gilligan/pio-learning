#![no_std]
#![no_main]

use bsp::entry;
use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    gpio::{FunctionPio0, Pin, PinId, PullDown},
    pac,
    pio::{
        Buffers, PIOBuilder, PIOExt, PinDir, PinState, Rx, ShiftDirection, StateMachineIndex, Tx,
        UninitStateMachine, PIO,
    },
    sio::Sio,
    watchdog::Watchdog,
};
use defmt::*;
use defmt_rtt as _;
use embedded_io::{Error, ErrorKind, ErrorType, Read, ReadReady, Write, WriteReady};
use panic_probe as _;
use rp_pico as bsp;

struct Reader<P, SM>
where
    P: PIOExt,
    SM: StateMachineIndex,
{
    rx: Rx<(P, SM)>,
}

struct Writer<P, SmControl>
where
    P: PIOExt,
    SmControl: StateMachineIndex,
{
    tx: Tx<(P, SmControl)>,
}

impl<P, SmControl> ErrorType for Writer<P, SmControl>
where
    P: PIOExt,
    SmControl: StateMachineIndex,
{
    type Error = MyError;
}

impl<P, SmControl> WriteReady for Writer<P, SmControl>
where
    P: PIOExt,
    SmControl: StateMachineIndex,
{
    fn write_ready(&mut self) -> Result<bool, Self::Error> {
        Ok(!self.tx.is_full())
    }
}

impl<P, SmControl> Write for Writer<P, SmControl>
where
    P: PIOExt,
    SmControl: StateMachineIndex,
{
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        if buf.len() >= 2 {
            // create a 32-bit frame and add it to the transmit FIFO
            let tx_frame: u32 = (buf[0] as u32)
                | (((buf[0] as u32 ^ 0xff) << 8) as u32)
                | (((buf[1] as u32) << 16) as u32)
                | (((buf[1] as u32 ^ 0xff) << 24) as u32);

            self.tx.write(tx_frame);
            Ok(2)
        } else {
            Err(MyError)
        }
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        while !self.tx.is_empty() {}
        Ok(())
    }
}

impl<P, SM> Reader<P, SM>
where
    P: PIOExt,
    SM: StateMachineIndex,
{
    pub fn new<PIN>(
        pio: &mut PIO<P>,
        sm: UninitStateMachine<(P, SM)>,
        pin: Pin<PIN, FunctionPio0, PullDown>,
    ) -> Self
    where
        P: PIOExt,
        SM: StateMachineIndex,
        PIN: PinId,
    {
        let program_with_defines = pio_proc::pio_file!(
            "examples/ir_nec/nec_receive.pio",
            select_program("nec_receive"),
            options(max_program_size = 32)
        );
        let program = program_with_defines.program;
        let (mut sm, rx, _) = PIOBuilder::from_program(pio.install(&program).unwrap())
            .buffers(Buffers::OnlyRx)
            .push_threshold(32)
            .autopush(true)
            .out_shift_direction(ShiftDirection::Right)
            .in_shift_direction(ShiftDirection::Right)
            .in_pin_base(pin.id().num)
            .jmp_pin(pin.id().num)
            .set_pins(0, 0)
            .build(sm);

        sm.set_clock_divisor(7031.25);
        sm.set_pindirs([(pin.id().num, PinDir::Input)]);
        sm.start();

        Self { rx }
    }
}

#[derive(Debug)]
struct MyError;

impl Error for MyError {
    fn kind(&self) -> ErrorKind {
        ErrorKind::Other
    }
}

impl<P, SM> ErrorType for Reader<P, SM>
where
    P: PIOExt,
    SM: StateMachineIndex,
{
    type Error = MyError;
}

impl<P, SM> Read for Reader<P, SM>
where
    P: PIOExt,
    SM: StateMachineIndex,
{
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        let rx_frame: RxFrame = unsafe {
            RxFrameWrapper {
                raw: self.rx.read().unwrap(),
            }
            .decoded
        };

        if rx_frame.address == (rx_frame.inverted_address ^ 0xff)
            && rx_frame.data == (rx_frame.inverted_data ^ 0xff)
        {
            buf[0] = rx_frame.address;
            buf[1] = rx_frame.data;
            Ok(2)
        } else {
            Err(MyError)
        }
    }
}

impl<P, SM> ReadReady for Reader<P, SM>
where
    P: PIOExt,
    SM: StateMachineIndex,
{
    fn read_ready(&mut self) -> Result<bool, Self::Error> {
        Ok(!self.rx.is_empty())
    }
}

impl<P, SmControl> Writer<P, SmControl>
where
    P: PIOExt,
    SmControl: StateMachineIndex,
{
    pub fn new<PIN, SmBurst>(
        pio: &mut PIO<P>,
        sm_burst: UninitStateMachine<(P, SmBurst)>,
        sm_control: UninitStateMachine<(P, SmControl)>,
        pin: Pin<PIN, FunctionPio0, PullDown>,
    ) -> Self
    where
        P: PIOExt,
        SmBurst: StateMachineIndex,
        SmControl: StateMachineIndex,
        PIN: PinId,
    {
        let program_with_defines = pio_proc::pio_file!(
            "examples/ir_nec/nec_carrier_burst.pio",
            select_program("nec_carrier_burst"),
            options(max_program_size = 32)
        );
        let program = program_with_defines.program;
        let (mut sm, _, _) = PIOBuilder::from_program(pio.install(&program).unwrap())
            .out_shift_direction(ShiftDirection::Right)
            .in_shift_direction(ShiftDirection::Right)
            .set_pins(pin.id().num, 1)
            .build(sm_burst);

        sm.set_clock_divisor(
            125_000_000.0 / (40.222e3 * program_with_defines.public_defines.TICKS_PER_LOOP as f32),
        );
        sm.set_pindirs([(pin.id().num, PinDir::Output)]);
        sm.set_pins([(pin.id().num, PinState::Low)]);
        sm.start();

        let program_with_defines = pio_proc::pio_file!(
            "examples/ir_nec/nec_carrier_control.pio",
            select_program("nec_carrier_control"),
            options(max_program_size = 32)
        );
        let program = program_with_defines.program;
        let (mut sm, _, tx) = PIOBuilder::from_program(pio.install(&program).unwrap())
            .buffers(Buffers::OnlyTx)
            .pull_threshold(32)
            .autopull(false)
            .out_shift_direction(ShiftDirection::Right)
            .in_shift_direction(ShiftDirection::Right)
            .set_pins(0, 0)
            .build(sm_control);
        sm.set_clock_divisor(35156.25);
        sm.start();

        Self { tx }
    }
}

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
    let tx_pin = pins.gpio14.into_function();
    let rx_pin = pins.gpio15.into_function();

    let (mut pio, sm0, sm1, sm2, _) = pac.PIO0.split(&mut pac.RESETS);

    let mut reader = Reader::new(&mut pio, sm2, rx_pin);
    let mut writer = Writer::new(&mut pio, sm0, sm1, tx_pin);

    let tx_address: u8 = 0x00;
    let mut tx_data: u8 = 0x00;

    loop {
        let buf = [tx_address, tx_data];
        writer.write(&buf).unwrap();
        println!("sent: {:x}, {:x}", tx_address, tx_data);

        while !reader.read_ready().unwrap() {}
        let mut buf: [u8; 2] = [0; 2];
        reader.read(&mut buf).unwrap();
        println!("received: {:x}, {:x}", buf[0], buf[1]);

        delay.delay_ms(30);
        tx_data = tx_data.wrapping_add(1);
    }
}

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
