#![no_main]
#![no_std]

use cmds::R_REGISTER;
use defmt::{info, println};
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
use hpm_hal::gpio::{Input, Level, Output, Speed};
use hpm_hal::mode::Blocking;
use hpm_hal::spi::{Config, Spi};
use hpm_hal::time::Hertz;
use riscv::delay::McycleDelay;
use {defmt_rtt as _, hpm_hal as hal, panic_halt as _};

pub mod cmds {
    pub const R_REGISTER: u8 = 0b0000_0000;
    pub const W_REGISTER: u8 = 0b0010_0000;
    pub const R_RX_PAYLOAD: u8 = 0b0110_0001;
    pub const W_TX_PAYLOAD: u8 = 0b1010_0000;
    pub const FLUSH_TX: u8 = 0b1110_0001;
    pub const FLUSH_RX: u8 = 0b1110_0010;
    pub const REUSE_TX_PL: u8 = 0b1110_0011;
    pub const R_RX_PL_WID: u8 = 0b0110_0000;
    pub const W_ACK_PAYLOAD: u8 = 0b1010_1000;
    pub const W_TX_PAYLOAD_NO_ACK: u8 = 0b1011_0000;
    pub const NOP: u8 = 0b1111_1111;
}

#[hal::entry]
fn main() -> ! {
    let p = hal::init(Default::default());

    let mut delay = McycleDelay::new(hal::sysctl::clocks().cpu0.0);
    defmt::info!("Board init!");

    let mut ce = Output::new(p.PB11, Level::High, Speed::Fast);
    let mut irq = Input::new(p.PB10, hpm_hal::gpio::Pull::None); // active low
    ce.set_low();
    delay.delay_ms(200);
    ce.set_high();

    let mut cs = Output::new(p.PA26, Level::High, Speed::Fast);

    let spi_config = Config {
        frequency: Hertz(80_000_000),
        ..Default::default()
    };
    let spi: hal::spi::Spi<'_, Blocking> = Spi::new_blocking(p.SPI1, p.PA27, p.PA29, p.PA28, spi_config);
    // let spi = AtomicCell::new(spi);
    // let mut nrf24l01 = AtomicDevice::new_no_delay(&spi, cs).unwrap();

    let mut nrf24l01 = spi;

    const R_REGISTER: u8 = 0b0000_0000;
    const W_REGISTER: u8 = 0b0010_0000;

    let mut led = Output::new(p.PA23, Level::Low, Speed::Fast);

    let mut config = hal::spi::TransferConfig::default();
    config.transfer_mode = hal::spi::TransMode::WRITE_READ_TOGETHER;

    let mut buf = [R_REGISTER | 0x00, 0x00, 0x00, 0x00, 0x00];
    cs.set_low();
    nrf24l01.blocking_transfer_inplace(&mut buf, &config).unwrap();
    cs.set_high();

    info!("step 3 {:02x}", buf);

    let mut buf = [R_REGISTER | 0x00, 0x00, 0x00, 0x00, 0x00];

    cs.set_low();
    nrf24l01.blocking_write(&buf).unwrap();
    cs.set_high();

    let mut out = [0x00_u8; 5];
    cs.set_low();
    nrf24l01.blocking_read(&mut out).unwrap();
    cs.set_high();

    info!("step 1 {:02x}", out);

    buf.fill(0x00);

    buf[0] = R_REGISTER | 0x00;

    let mut out = [0x00; 5];

    //    nrf24l01.

    cs.set_low();
    nrf24l01.blocking_transfer(&mut out, &buf, &config).unwrap();
    cs.set_high();

    info!("step 2 {:02x}", out);

    //    delay.delay_ms(1);

    loop {
        led.toggle();
        delay.delay_ms(500);
    }
}
