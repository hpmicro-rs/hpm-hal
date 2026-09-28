#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]
#![feature(impl_trait_in_assoc_type)]
#![feature(abi_riscv_interrupt)]

use defmt::info;
use embassy_time::Timer;
use embedded_graphics::draw_target::DrawTarget;
use embedded_graphics::geometry::{Dimensions, OriginDimensions, Point, Size};
use embedded_graphics::image::{Image, ImageRawLE};
use embedded_graphics::mono_font::ascii::FONT_10X20;
use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::pixelcolor::raw::ToBytes;
use embedded_graphics::pixelcolor::{Rgb565, RgbColor};
use embedded_graphics::primitives::Rectangle;
use embedded_graphics::text::Text;
use embedded_graphics::transform::Transform;
use embedded_graphics::Drawable;
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::OutputPin;
use embedded_hal::spi::{Polarity, SpiDevice};
use embedded_hal_bus::spi::AtomicDevice;
use embedded_hal_bus::util::AtomicCell;
use hpm_hal::gpio::{Level, Output, Speed};
use hpm_hal::mode::Blocking;
use hpm_hal::spi::{Config, Spi, MODE_0, MODE_1, MODE_2, MODE_3};
use hpm_hal::time::Hertz;
use riscv::delay::McycleDelay;
use {defmt_rtt as _, hpm_hal as hal, panic_halt as _};

#[embassy_executor::main(entry = "hpm_hal::entry")]
async fn main(spawner: embassy_executor::Spawner) -> ! {
    let p = hal::init(Default::default());

    defmt::info!("Init ok");

    let mut cs = Output::new(p.PB01, Level::High, Speed::Fast);

    let spi_config = Config {
        frequency: Hertz(1_000_000),
        mode: MODE_0,
        ..Default::default()
    };
    let mut spi: hal::spi::Spi<'_, Blocking> = Spi::new_blocking(p.SPI1, p.PA27, p.PA29, p.PA28, spi_config);
    // let spi = AtomicCell::new(spi);
    // let mut mt6816 = AtomicDevice::new_no_delay(&spi, cs).unwrap();

    let mut buf = [0u8; 4];
    //  buf[0] = REG | 0x80;

    //mt6816.transfer_in_place(&mut buf).unwrap();

    defmt::info!("buf: {:?}", buf);

    let mut led = Output::new(p.PA23, Level::High, Speed::Fast);

    let mut out = [0u8; 2];

    let mut tx_ops = hal::spi::TransferConfig::default();
    tx_ops.transfer_mode = hal::spi::TransMode::WRITE_DUMMY_READ;
    tx_ops.dummy_cnt = 1;

    const REG: u8 = 0x05;

    loop {
        led.toggle();

        Timer::after_millis(100).await;

        defmt::info!("begin tx");

        cs.set_low();
        spi.blocking_transfer(&mut out, &[REG | 0x80], &tx_ops).unwrap();
        cs.set_high();

        // defmt::info!("Hello, World!");
        defmt::info!("buf: {:?}", out);

        out.fill(0x00);
    }
}
