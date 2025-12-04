#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]
#![feature(abi_riscv_interrupt)]
#![feature(impl_trait_in_assoc_type)]

use defmt::info;
use embassy_executor::Spawner;
use embassy_usb::class::cdc_acm::{CdcAcmClass, Receiver, Sender, State};
use embassy_usb::driver::EndpointError;
use embassy_usb::Builder;
use futures_util::future::join;
use hal::usb::{EndpointState, Instance, UsbDriver};
use hpm_hal::peripherals;
use static_cell::StaticCell;
use {defmt_rtt as _, hpm_hal as hal};

hal::bind_interrupts!(struct Irqs {
    USB0 => hal::usb::InterruptHandler<peripherals::USB0>;
});

/// USB endpoint state - must be in non-cacheable memory for DMA access
#[link_section = ".noncacheable"]
static EP_STATE: EndpointState = EndpointState::new();

#[embassy_executor::main(entry = "hpm_hal::entry")]
async fn main(_spawner: Spawner) -> ! {
    let p = hal::init(Default::default());

    let usb_driver = hal::usb::UsbDriver::new(p.USB0, Irqs, p.PA24, p.PA25, Default::default(), &EP_STATE);

    // Create embassy-usb Config
    let mut config = embassy_usb::Config::new(0xc0de, 0xcafe);
    config.manufacturer = Some("hpm-hal");
    config.product = Some("USB-serial example");
    config.serial_number = Some("12345678");

    // Required for windows compatibility.
    // https://developer.nordicsemi.com/nRF_Connect_SDK/doc/1.9.1/kconfig/CONFIG_CDC_ACM_IAD.html#help
    config.device_class = 0xEF;
    config.device_sub_class = 0x02;
    config.device_protocol = 0x01;
    config.composite_with_iads = true;

    // Create embassy-usb DeviceBuilder using the driver and config.
    // It needs some buffers for building the descriptors.
    // Use StaticCell to ensure buffer addresses remain stable across await points.
    static CONFIG_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static BOS_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();
    static CDC_STATE: StaticCell<State> = StaticCell::new();

    let mut builder = Builder::new(
        usb_driver,
        config,
        CONFIG_DESC.init([0; 256]),
        BOS_DESC.init([0; 256]),
        &mut [], // no msos descriptors
        CONTROL_BUF.init([0; 64]),
    );

    // Create classes on the builder.
    let class = CdcAcmClass::new(&mut builder, CDC_STATE.init(State::new()), 64);

    // Build the builder.
    let mut usb = builder.build();

    // Run the USB device.
    let usb_fut = usb.run();

    // Do stuff with the class!
    let echo_fut = async {
        // class.wait_connection().await;
        let (mut sender, mut reader) = class.split();
        sender.wait_connection().await;
        reader.wait_connection().await;
        info!("Connected");
        let _ = echo(&mut reader, &mut sender).await;
        info!("Disconnected");
    };

    // Run everything concurrently.
    join(usb_fut, echo_fut).await;

    loop {
        embassy_time::Timer::after_millis(500).await;
    }
}

struct Disconnected {}

impl From<EndpointError> for Disconnected {
    fn from(val: EndpointError) -> Self {
        match val {
            EndpointError::BufferOverflow => panic!("Buffer overflow"),
            EndpointError::Disabled => Disconnected {},
        }
    }
}

async fn echo<'d, T: Instance + 'd>(
    reader: &mut Receiver<'d, UsbDriver<'d, T>>,
    sender: &mut Sender<'d, UsbDriver<'d, T>>,
) -> Result<(), Disconnected> {
    let mut buf = [0; 64];
    loop {
        let n = reader.read_packet(&mut buf).await?;
        let data = &buf[..n];
        info!("echo data: {:x}, len: {}", data, n);
        sender.write_packet(data).await?;
        // Clear bufffer
        buf = [0; 64];
    }
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    defmt::info!("panic: {:?}", defmt::Debug2Format(&info));
    loop {}
}
