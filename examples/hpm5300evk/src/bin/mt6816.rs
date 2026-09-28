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
use embedded_hal::spi::SpiDevice;
use embedded_hal_bus::spi::AtomicDevice;
use embedded_hal_bus::util::AtomicCell;
use hpm_hal::gpio::{Input, Level, Output, Pull, Speed};
use hpm_hal::mode::Blocking;
use hpm_hal::spi::{Config, Spi};
use hpm_hal::time::Hertz;
use riscv::delay::McycleDelay;
use {defmt_rtt as _, hpm_hal as hal, panic_halt as _};

#[embassy_executor::main(entry = "hpm_hal::entry")]
async fn main(spawner: embassy_executor::Spawner) -> ! {
    let p = hal::init(Default::default());

    defmt::info!("Init ok");

    let mut a = Input::new(p.PA28, Pull::None);
    let mut b = Input::new(p.PA20, Pull::None);

    let cs = Output::new(p.PA26, Level::High, Speed::Fast);

    let spi_config = Config {
        frequency: Hertz(1_000_000),
        ..Default::default()
    };
    let spi: hal::spi::Spi<'_, Blocking> = Spi::new_blocking(p.SPI1, p.PA27, p.PA29, p.PA28, spi_config);
    let spi = AtomicCell::new(spi);
    let mut mt6816 = AtomicDevice::new_no_delay(&spi, cs).unwrap();

    const REG: u8 = 0x03;

    let mut buf = [0u8; 4];
    //  buf[0] = REG | 0x80;

    //mt6816.transfer_in_place(&mut buf).unwrap();

    defmt::info!("buf: {:?}", buf);

    let mut led = Output::new(p.PA23, Level::High, Speed::Fast);

    loop {
        led.toggle();

        Timer::after_millis(100).await;

        mt6816.write(&[REG | 0x80]).unwrap();
        mt6816.read(&mut buf).unwrap();

        // defmt::info!("Hello, World!");
        defmt::info!("buf: {:?}", buf);
    }
}
