#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]
#![feature(impl_trait_in_assoc_type)]
#![feature(abi_riscv_interrupt)]

use core::ptr::addr_of_mut;

use embassy_executor::Spawner;
use embassy_sync::waitqueue::AtomicWaker;
use embassy_time::{Duration, Timer};
use hal::gpio::{AnyPin, Flex, Pin};
use hpm_hal::mcan::{Dependencies, RxPin, TxPin};
use hpm_hal::time::Hertz;
use hpm_hal::{bind_interrupts, peripherals, Peripheral};
use {defmt_rtt as _, hpm_hal as hal};

bind_interrupts!(struct Irqs {
    UART2 => hal::uart::InterruptHandler<peripherals::UART2>;
});

const BANNER: &str = include_str!("../../../assets/BANNER");

#[embassy_executor::task(pool_size = 2)]
async fn blink(pin: AnyPin) {
    let mut led = Flex::new(pin);
    led.set_as_output(Default::default());
    led.set_high();

    loop {
        led.toggle();

        Timer::after_millis(500).await;
    }
}

use mcan::bus::CanConfigurable;
use mcan::config::Mode;
use mcan::core::fugit::HertzU32;
use mcan::embedded_can as ecan;
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

type CanPeripheral = peripherals::MCAN1;

#[link_section = ".can"]
static mut CAN_MEMORY: SharedMemory<Capacities> = SharedMemory::new();

static mut LINE_INTERRUPTS: Option<OwnedInterruptSet<CanPeripheral>> = None;

static CAN_RX_WAKER: AtomicWaker = AtomicWaker::new();

pub struct MyCan {
    pub can: mcan::bus::Can<'static, CanPeripheral, hpm_hal::mcan::Dependencies<'static, CanPeripheral>, Capacities>,
}

impl MyCan {
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

        Self { can }
    }

    fn on_interrupt() {
        let line_interrupts = unsafe { LINE_INTERRUPTS.as_mut().unwrap() };
        // let flags = line_interrupts.interrupt_flags();
        for interrupt in line_interrupts.iter_flagged() {
            // println!("interrupt: {:?}", interrupt);
            match interrupt {
                Interrupt::RxFifo0NewMessage | Interrupt::RxFifo0Full => {
                    // CAN_RX_WAKER.wake();
                }
                Interrupt::RxFifo1NewMessage | Interrupt::RxFifo1Full => {
                    CAN_RX_WAKER.wake();
                }
                _ => {}
            }
        }
    }

    fn clear_fifo(&mut self) {
        while let Some(_) = self.can.rx_fifo_1.next() {}
    }

    fn send_raw_mesage(&mut self, eid: u32, data: &[u8]) {
        self.can
            .tx
            .transmit_queued(
                tx::MessageBuilder {
                    id: ecan::Id::Extended(ecan::ExtendedId::new(eid).unwrap()), // 29bit id
                    frame_type: tx::FrameType::Classic(tx::ClassicFrameType::Data(data)),
                    store_tx_event: None,
                }
                .build()
                .unwrap(),
            )
            .unwrap(); // WouldBlock
    }
}

#[allow(non_snake_case)]
#[no_mangle]
unsafe extern "riscv-interrupt-m" fn MCAN1() {
    use hal::interrupt::InterruptExt;

    MyCan::on_interrupt();

    hal::interrupt::MCAN1.complete();
}

#[embassy_executor::main(entry = "hpm_hal::entry")]
async fn main(spawner: Spawner) -> ! {
    let mut config = hal::Config::default();
    {
        use hal::sysctl::*;
        config.sysctl.pll0 = Some(Pll {
            div: (0, 1, 3),
            freq_in: Hertz::mhz(960),
        });
        config.sysctl.cpu0 = ClockConfig::new(ClockMux::PLL0CLK0, 2);
        config.sysctl.ahb_div = AHBDiv::DIV3;
    }

    let p = hal::init(config);

    spawner.spawn(blink(p.PA23.degrade())).unwrap();
    spawner.spawn(blink(p.PA10.degrade())).unwrap();

    // let button = Input::new(p.PA03, Pull::Down); // hpm5300evklite, BOOT1_KEYt
    let mut uart_config = hal::uart::Config::default();
    uart_config.baudrate = 921600;
    let mut uart = hal::uart::Uart::new(p.UART2, p.PB09, p.PB08, Irqs, p.HDMA_CH1, p.HDMA_CH0, uart_config).unwrap();

    let mut can_bus = MyCan::new(p.MCAN1, p.PB04, p.PB05);
    defmt::info!("Loop");

    let mut buf = [0u8; 256];

    let mut atat_mode = false;
    loop {
        let Ok(n) = embassy_time::with_timeout(Duration::from_millis(50), uart.read_until_idle(&mut buf)).await else {
            if can_bus.can.rx_fifo_1.is_empty() {
                continue;
            }

            while let Some(msg) = can_bus.can.rx_fifo_1.next() {
                let ecan::Id::Extended(eid) = msg.id() else { continue };
                let raw_id = eid.as_raw();
                let data = msg.data();

                let encoded_id = (raw_id << 3) | 0b0100; // set extended frame bit

                let mut buf = [0u8; 2 + 2 + 4 + 8 + 1];
                buf[0] = b'A';
                buf[1] = b'T';
                buf[2..6].copy_from_slice(&encoded_id.to_be_bytes());
                buf[6] = data.len() as u8;
                buf[7..7 + data.len()].copy_from_slice(data);
                buf[7 + data.len()] = b'\r';
                buf[7 + data.len() + 1] = b'\n';

                //  defmt::info!("reply! 0x{:08x} {:02x}", raw_id, data);
                // defmt::info!("raw => {:02x}", &buf[..7 + data.len() + 2]);

                uart.write(&buf[..7 + data.len() + 2]).await.unwrap();
            }

            continue;
        };

        let n = n.unwrap();

        let payload = &buf[..n];

        //defmt::info!("read => {:02x}", buf[..n].as_ref());

        if n > 2 && payload[0] == b'A' && payload[1] == b'T' {
            if atat_mode {
                let frame_type = buf[2];
                // 0xAA, 0baaa_eeeee 0x
                let frame_id = u32::from_be_bytes(buf[2..6].try_into().unwrap());
                let frame_len = buf[6];
                let frame_data = &buf[7..7 + frame_len as usize];

                //   defmt::info!("frame_type: {:?}", frame_type);
                // defmt::info!("frame_id: 0x{:08x}", frame_id);
                //defmt::info!("frame_len: {:?}", frame_len);
                //defmt::info!("frame_data: {:?}", frame_data);

                // bit3: reserved ???
                // bit2: extended frame
                // bit1: remote frame
                // bit0: reserved 0
                let can_id = frame_id >> 3;

                defmt::info!("TX can_id: 0x{:08x} {:02x}", can_id, frame_data);

                if can_id & 0xff == 0x7f {
                    defmt::info!("send!");
                    can_bus.send_raw_mesage(can_id, frame_data);
                }
            } else {
                if buf.starts_with(b"AT+AT\r\n") {
                    atat_mode = true;
                }

                let s = "OK\r\n";
                uart.write(s.as_bytes()).await.unwrap();

                let s = core::str::from_utf8(&buf[..n]).unwrap();
                defmt::info!("write => {:?}", s);
            }
        }
    }
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    let mut err = heapless::String::<1024>::new();

    use core::fmt::Write as _;

    write!(err, "panic: {}", info).ok();

    defmt::info!("{}", err.as_str());

    loop {}
}
