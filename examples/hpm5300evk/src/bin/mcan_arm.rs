//! Xiaomi CyberGear CAN moter driver

#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]
#![feature(impl_trait_in_assoc_type)]
#![feature(abi_riscv_interrupt)]

use core::ptr::addr_of_mut;

// use defmt::println;
use embassy_executor::Spawner;
use embassy_sync::waitqueue::AtomicWaker;
use embassy_time::Timer;
use embedded_io::Write as _;
use hal::gpio::{Level, Output, Pin};
use hal::interrupt::InterruptExt;
use hal::mcan::Dependencies;
use hal::mode::Blocking;
use hal::{pac, peripherals};
use hpm_hal::gpio::{Input, Pull};
use hpm_hal::mcan::{RxPin, TxPin};
use hpm_hal::Peripheral;
use mcan::bus::CanConfigurable;
use {defmt_rtt as _, hpm_hal as hal};

const BOARD_NAME: &str = "HPM5300EVK";
const BANNER: &str = include_str!("../../../assets/BANNER");

use mcan::config::Mode;
use mcan::core::fugit::HertzU32;
use mcan::embedded_can::{self as ecan, StandardId};
use mcan::filter::{Action, ExtFilter, Filter};
use mcan::generic_array::typenum::consts::*;
use mcan::interrupt::{Interrupt, InterruptLine, OwnedInterruptSet};
use mcan::message::{rx, tx};
use mcan::messageram::SharedMemory;
use mcan::prelude::*;
use mcan::rx_fifo::{Fifo0, Fifo1, RxFifo};

pub struct Capacities;

impl mcan::messageram::Capacities for Capacities {
    type StandardFilters = U1;
    type ExtendedFilters = U1;
    type RxBufferMessage = rx::Message<64>;
    type DedicatedRxBuffers = U0;
    type RxFifo0Message = rx::Message<64>;
    type RxFifo0 = U64;
    type RxFifo1Message = rx::Message<64>;
    type RxFifo1 = U64;
    type TxMessage = tx::Message<64>;
    type TxBuffers = U32;
    type DedicatedTxBuffers = U0;
    type TxEventFifo = U32;
}

type RxFifo0 = RxFifo<'static, Fifo0, peripherals::MCAN3, <Capacities as mcan::messageram::Capacities>::RxFifo0Message>;
type RxFifo1 = RxFifo<'static, Fifo1, peripherals::MCAN3, <Capacities as mcan::messageram::Capacities>::RxFifo1Message>;
type Tx = mcan::tx_buffers::Tx<'static, peripherals::MCAN3, Capacities>;
type TxEventFifo = mcan::tx_event_fifo::TxEventFifo<'static, peripherals::MCAN3>;
type Aux = mcan::bus::Aux<'static, peripherals::MCAN3, Dependencies<'static, peripherals::MCAN3>>;

#[link_section = ".can"]
static mut CAN_MEMORY: SharedMemory<Capacities> = SharedMemory::new();

static mut UART: Option<hal::uart::Uart<'static, Blocking>> = None;
macro_rules! println {
    ($($arg:tt)*) => {
        let _ = writeln!(unsafe {(&mut *(&raw mut UART)).as_mut().unwrap()}, $($arg)*);
    };
}

static CAN_RX_WAKER: AtomicWaker = AtomicWaker::new();

// The type alias for the CAN peripheral
type CanPeripheral = peripherals::MCAN3;

static mut LINE_INTERRUPTS: Option<OwnedInterruptSet<CanPeripheral>> = None;

#[allow(non_snake_case)]
#[no_mangle]
unsafe extern "riscv-interrupt-m" fn MCAN3() {
    println!("MCAN3 interrupt");

    let line_interrupts = unsafe { LINE_INTERRUPTS.as_mut().unwrap() };
    for interrupt in line_interrupts.iter_flagged() {
        match interrupt {
            // Classic CAN message
            Interrupt::RxFifo0NewMessage => {
                CAN_RX_WAKER.wake();
            }
            _ => {
                println!("interrupt: {:?}", interrupt);
            }
        }
    }
    //   line_interrupts.clear_interrupts(flags);
    //    panic!("fuck");

    hal::interrupt::MCAN3.complete();
}

#[derive(Clone, Copy)]
pub struct ArmState {
    pub hand: u16,
    pub motor1: u16,
    pub motor2: u16,
    pub motor3: u16,
    pub motor4: u16,
}

impl ArmState {
    pub const fn new() -> ArmState {
        ArmState {
            hand: 2100,
            motor1: 2000,
            motor2: 2100,
            motor3: 2100,
            motor4: 2000,
        }
    }
}

pub struct CruzrRobotArm {
    pub can: mcan::bus::Can<'static, CanPeripheral, Dependencies<'static, CanPeripheral>, Capacities>,
    pub state: ArmState,
}

impl CruzrRobotArm {
    // init from MCAN3
    pub fn new(
        can: impl Peripheral<P = CanPeripheral> + 'static,
        rx: impl Peripheral<P = impl RxPin<CanPeripheral>> + 'static,
        tx: impl Peripheral<P = impl TxPin<CanPeripheral>> + 'static,
    ) -> Self {
        let dependencies = hal::mcan::Dependencies::new(can, rx, tx);

        let mut can: CanConfigurable<'static, CanPeripheral, Dependencies<CanPeripheral>, Capacities> =
            CanConfigurable::new(HertzU32::kHz(1_000), dependencies, unsafe {
                &mut *addr_of_mut!(CAN_MEMORY)
            })
            .unwrap();

        can.config().mode = Mode::Classic;

        let interrupts_to_be_enabled = can
            .interrupts()
            .split(
                [
                    Interrupt::RxFifo0NewMessage,
                    Interrupt::RxFifo0Full,
                    Interrupt::RxFifo0MessageLost,
                    Interrupt::RxFifo1NewMessage,
                    Interrupt::RxFifo1Full,
                    Interrupt::RxFifo1MessageLost,
                ]
                .into_iter()
                .collect(),
            )
            .unwrap();
        // HPM chips do not expose separate NVIC lines to MCAN
        // InterruptLine::Line0 and InterruptLine::Line1 are wired
        // together in the hardware.
        let line_interrupts = can
            .interrupt_configuration()
            .enable(interrupts_to_be_enabled, InterruptLine::Line0);
        unsafe {
            LINE_INTERRUPTS = Some(line_interrupts);
        }

        // Example filters configuration
        // This filter will put all messages with a standard ID into RxFifo0
        can.filters_standard()
            .push(Filter::Classic {
                action: Action::StoreFifo0,
                filter: ecan::StandardId::MAX,
                mask: ecan::StandardId::ZERO,
            })
            .unwrap_or_else(|_| panic!("Standard filter application failed"));
        // This filter will put all messages with a extended ID into RxFifo1
        can.filters_extended()
            .push(ExtFilter::Classic {
                action: Action::StoreFifo1,
                filter: ecan::ExtendedId::MAX,
                mask: ecan::ExtendedId::ZERO,
            })
            .unwrap_or_else(|_| panic!("Extended filter application failed"));

        let can = can.finalize().unwrap();

        Self {
            can,
            state: ArmState::new(),
        }
    }

    pub fn handle_messages(&mut self) {
        while let Some(message) = self.can.rx_fifo_0.next() {
            match message.id() {
                ecan::Id::Standard(id) if id.as_raw() == 0x6E => {
                    continue;
                }
                ecan::Id::Standard(id) => {
                    let motor_id = id.as_raw();

                    let msg_type = message.data()[0];

                    match msg_type {
                        0x02 => {
                            let angle = u16::from_le_bytes([message.data()[1], message.data()[2]]);
                        }
                        _ => {}
                    }
                }
                _ => (),
            }
        }
    }
}

#[embassy_executor::main(entry = "hpm_hal::entry")]
async fn main(spawner: Spawner) -> ! {
    let p = hal::init(Default::default());
    // let button = Input::new(p.PA03, Pull::Down); // hpm5300evklite, BOOT1_KEY
    let uart = hal::uart::Uart::new_blocking(p.UART0, p.PA01, p.PA00, Default::default()).unwrap();
    unsafe { UART = Some(uart) }

    let mut user_button = Input::new(p.PA09, Pull::None);

    println!("\n{}", BANNER);
    println!("{} init OK!", BOARD_NAME);

    println!("Clock summary:");
    println!("  CPU0:\t{}Hz", hal::sysctl::clocks().cpu0.0);
    println!("  AHB:\t{}Hz", hal::sysctl::clocks().ahb.0);
    println!(
        "  XPI0:\t{}Hz",
        hal::sysctl::clocks().get_clock_freq(hal::pac::clocks::XPI0).0
    );
    println!(
        "  MTMR:\t{}Hz",
        hal::sysctl::clocks().get_clock_freq(pac::clocks::MCT0).0
    );
    println!(
        "  CAN3:\t{}Hz",
        hal::sysctl::clocks().get_clock_freq(pac::clocks::CAN3).0
    );

    println!("==============================");

    println!("Hello, world!");

    p.PY04.set_as_ioc_gpio();
    p.PY05.set_as_ioc_gpio();
    let dependencies = hal::mcan::Dependencies::new(p.MCAN3, p.PY04, p.PY05);
    //let dependencies = hal::mcan::Dependencies::new(p.MCAN1, p.PB04, p.PB05);

    let mut can: CanConfigurable<'static, CanPeripheral, Dependencies<CanPeripheral>, Capacities> =
        mcan::bus::CanConfigurable::new(HertzU32::kHz(1_000), dependencies, unsafe { &mut CAN_MEMORY }).unwrap();

    can.config().mode = Mode::Classic;

    // Example interrupt configuration
    let interrupts_to_be_enabled = can
        .interrupts()
        .split(
            [
                Interrupt::RxFifo0NewMessage,
                Interrupt::RxFifo0Full,
                Interrupt::RxFifo0MessageLost,
                Interrupt::RxFifo1NewMessage,
                Interrupt::RxFifo1Full,
                Interrupt::RxFifo1MessageLost,
            ]
            .into_iter()
            .collect(),
        )
        .unwrap();
    // HPM chips do not expose separate NVIC lines to MCAN
    // InterruptLine::Line0 and InterruptLine::Line1 are wired
    // together in the hardware.
    let line_interrupts = can
        .interrupt_configuration()
        .enable(interrupts_to_be_enabled, InterruptLine::Line0);
    unsafe {
        LINE_INTERRUPTS = Some(line_interrupts);
    }

    // Example filters configuration
    // This filter will put all messages with a standard ID into RxFifo0
    can.filters_standard()
        .push(Filter::Classic {
            action: Action::StoreFifo0,
            filter: ecan::StandardId::MAX,
            mask: ecan::StandardId::ZERO,
        })
        .unwrap_or_else(|_| panic!("Standard filter application failed"));
    // This filter will put all messages with a extended ID into RxFifo1
    can.filters_extended()
        .push(ExtFilter::Classic {
            action: Action::StoreFifo1,
            filter: ecan::ExtendedId::MAX,
            mask: ecan::ExtendedId::ZERO,
        })
        .unwrap_or_else(|_| panic!("Extended filter application failed"));

    let can = can.finalize().unwrap();

    // `can` object can be split into independent pieces
    let mut rx_fifo_0 = can.rx_fifo_0;
    let mut rx_fifo_1 = can.rx_fifo_1;
    let mut tx = can.tx;
    let mut tx_event_fifo = can.tx_event_fifo;
    let aux = can.aux;

    defmt::info!("operational? {}", aux.is_operational());

    let mut led = Output::new(p.PA23, Level::Low, Default::default());

    let mut buf = [0u8; 8];

    //    buf[1..3].copy_from_slice(&u16::to_le_bytes(2000));
    //  buf[3..5].copy_from_slice(&u16::to_le_bytes(10));
    // buf[1] = 8;

    const MOT_ID: u16 = 0x00;

    let mut a = 230;

    buf[0] = 0x02;

    buf[1] = 30;
    buf[2] = 0xa0;
    // 速度低位
    buf[3] = 0; // time?
                // 200 slow
                // 100 rather slow
                // 50 more slow
                // 10 slowest
                // 0 fastest
                //???
    buf[4] = 1; // 速度高位, > 0 时候噪音大

    let mut a = 0;

    buf.fill(0x00);

    buf[0] = 0x0E;

    buf[1] = 0;
    buf[3] = 0;
    buf[4] = 0;

    let mut cmd = 0x10;
    //        let mut  = 0x3;

    println!("user btn {}", user_button.is_high());
    loop {
        while let Some(message) = rx_fifo_0.next() {
            match message.id() {
                ecan::Id::Standard(id) if id.as_raw() == 0x6E => {
                    continue;
                }
                _ => (),
            }
            println!("id:                 {:0X?}", message.id());
            println!("data:               {:0X?}", message.data());
            let data = message.data();

            if data[0] == 0xB {
                let angle = u16::from_le_bytes([data[1], data[2]]);
                println!("angle: {}", angle);

                let speed = u16::from_le_bytes([data[3], data[4]]);
                println!("speed: {}", speed);
            }
        }

        if user_button.is_low() {
            //  buf[0] = cmd;
            println!("user btn {} do logic a={}", user_button.is_high(), cmd);
            cmd += 1;

            tx.transmit_queued(
                tx::MessageBuilder {
                    id: ecan::Id::Standard(StandardId::new(MOT_ID).unwrap()), // id=16
                    frame_type: tx::FrameType::Classic(tx::ClassicFrameType::Data(&buf)),
                    store_tx_event: None,
                }
                .build()
                .unwrap(),
            )
            .unwrap();
        }

        led.toggle();
        defmt::info!("tick");

        Timer::after_millis(1000).await;
    }

    loop {
        while let Some(message) = rx_fifo_1.next() {
            match message.id() {
                ecan::Id::Standard(id) if id.as_raw() == 0x6E => {
                    continue;
                }
                ecan::Id::Standard(id) => {
                    let motor_id = id.as_raw();

                    println!("MSG: {:02X?} {:02X?}", motor_id, message.data());

                    //  if motor_id == 0x0b {
                    //      min = min.min(u16::from_le_bytes([message.data()[1], message.data()[2]]));
                    //     max = max.max(u16::from_le_bytes([message.data()[1], message.data()[2]]));
                    // }
                }
                _ => (),
            }
        }
    }

    // 0x6E

    {
        buf[0] = 0x03; // motion control
        buf[1..3].copy_from_slice(&u16::to_le_bytes(1000));
        buf[3..5].copy_from_slice(&u16::to_le_bytes(10)); // spped, no more than 255. 20

        tx.transmit_queued(
            tx::MessageBuilder {
                id: ecan::Id::Standard(StandardId::new(0x0b).unwrap()), // id=16
                frame_type: tx::FrameType::Classic(tx::ClassicFrameType::Data(&buf)),
                store_tx_event: None,
            }
            .build()
            .unwrap(),
        )
        .unwrap();
    }

    let mut min = 0xFFFF;
    let mut max = 0;

    loop {
        tx.transmit_queued(
            tx::MessageBuilder {
                id: ecan::Id::Standard(StandardId::new(0x0a).unwrap()), // id=16
                frame_type: tx::FrameType::Classic(tx::ClassicFrameType::Data(&[
                    0x25, 0x00, // 0x7005
                    0x00, 0x00, // pad
                    0x00, 0x00, // ?
                    0x00, 0x00, // runmode = 2
                ])),
                store_tx_event: None,
            }
            .build()
            .unwrap(),
        )
        .unwrap();
        Timer::after_millis(100).await;
        while let Some(message) = rx_fifo_0.next() {
            match message.id() {
                ecan::Id::Standard(id) if id.as_raw() == 0x6E => {
                    continue;
                }
                ecan::Id::Standard(id) => {
                    let motor_id = id.as_raw();

                    println!("MSG: {:02X?} {:02X?}", motor_id, message.data());

                    //  if motor_id == 0x0b {
                    //      min = min.min(u16::from_le_bytes([message.data()[1], message.data()[2]]));
                    //     max = max.max(u16::from_le_bytes([message.data()[1], message.data()[2]]));
                    // }
                }
                _ => (),
            }
        }
    }

    let mut buf = [0u8; 8];

    // 2700, 1800
    loop {
        buf[0] = 0x03; // motion control

        buf[1..3].copy_from_slice(&u16::to_le_bytes(1500));
        buf[3..5].copy_from_slice(&u16::to_le_bytes(20)); // spped, no more than 255

        tx.transmit_queued(
            tx::MessageBuilder {
                id: ecan::Id::Standard(StandardId::new(0x0b).unwrap()), // id=16
                frame_type: tx::FrameType::Classic(tx::ClassicFrameType::Data(&buf)),
                store_tx_event: None,
            }
            .build()
            .unwrap(),
        )
        .unwrap();

        Timer::after_millis(2500).await;

        buf[1..3].copy_from_slice(&u16::to_le_bytes(2400));
        buf[3..5].copy_from_slice(&u16::to_le_bytes(20)); // spped, no more than 255

        tx.transmit_queued(
            tx::MessageBuilder {
                id: ecan::Id::Standard(StandardId::new(0x0b).unwrap()), // id=16
                frame_type: tx::FrameType::Classic(tx::ClassicFrameType::Data(&buf)),
                store_tx_event: None,
            }
            .build()
            .unwrap(),
        )
        .unwrap();

        Timer::after_millis(2500).await;
        while let Some(message) = rx_fifo_0.next() {
            println!("id:                 {:0X?}", message.id());
            println!("data:               {:0X?}", message.data());
        }
    }

    loop {
        led.toggle();

        Timer::after_millis(500).await;
        while let Some(message) = rx_fifo_0.next() {
            println!("id:                 {:0X?}", message.id());
            println!("data:               {:0X?}", message.data());
        }
    }
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    use core::fmt::Write as _;

    let mut err = heapless::String::<1024>::new();

    write!(err, "panic: {}", info).ok();

    defmt::error!("{}", err.as_str());

    loop {}
}
