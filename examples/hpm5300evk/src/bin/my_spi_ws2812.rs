#![no_main]
#![no_std]
#![feature(impl_trait_in_assoc_type)]

use defmt::{info, println};
use embassy_time::Timer;
use embedded_graphics::draw_target::DrawTarget;
use embedded_graphics::geometry::{Dimensions, OriginDimensions, Point, Size};
use embedded_graphics::image::{Image, ImageRawLE};
use embedded_graphics::mono_font::ascii::FONT_10X20;
use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::pixelcolor::raw::ToBytes;
use embedded_graphics::pixelcolor::{Rgb565, RgbColor};
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::Rectangle;
use embedded_graphics::text::Text;
use embedded_graphics::transform::Transform;
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::OutputPin;
use embedded_hal::spi::SpiDevice;
use embedded_hal_bus::spi::AtomicDevice;
use embedded_hal_bus::util::AtomicCell;
use hal::pac;
use hpm_hal::gpio::{Level, Output, Speed};
use hpm_hal::mode::{Async, Blocking};
use hpm_hal::pac::uart::regs::Thr;
use hpm_hal::spi::{Config, Spi};
use hpm_hal::time::Hertz;
use riscv::delay::McycleDelay;
use smart_leds::{SmartLedsWrite, White, RGB8, RGBW};
use ws2812_spi::Ws2812;
use {defmt_rtt as _, hpm_hal as hal, panic_halt as _};

const LEDS: usize = 64 * 6;

// 3 bit for 1 bit ws2812 transfer
pub struct Leds(pub [u8; 3 * 3 * LEDS + 20]);

impl Leds {
    pub fn new() -> Self {
        let mut leds = [0u8; 3 * 3 * LEDS + 20];
        for i in 0..3 * 3 * LEDS {
            leds[i] = 0;
        }
        Self(leds)
    }

    pub fn set_pixel(&mut self, n: usize, r: u8, g: u8, b: u8) {
        let offset = n * 3 * 3;

        let one = 0b110;
        let zero = 0b100;

        let raw = (g as u32) << 16 | (r as u32) << 8 | b as u32;
    }
}

#[embassy_executor::main(entry = "hpm_hal::entry")]
async fn main(spawner: embassy_executor::Spawner) -> ! {
    let p = hal::init(Default::default());

    let mut delay = McycleDelay::new(hal::sysctl::clocks().cpu0.0);
    defmt::info!("Board init!");

    let spi_clk_in = hal::sysctl::clocks().get_clock_freq(hal::pac::clocks::SPI1);
    defmt::info!("spi clk in: {}", spi_clk_in.0);

    let mut rst = Output::new(p.PB05, Level::High, Speed::Fast);

    rst.set_low();
    delay.delay_ms(200);
    rst.set_high();

    let spi_config = Config {
        frequency: Hertz(3_200_000),
        bit_order: hpm_hal::spi::BitOrder::LsbFirst,
        mode: hal::spi::Mode {
            polarity: embedded_hal::spi::Polarity::IdleHigh,
            phase: embedded_hal::spi::Phase::CaptureOnFirstTransition,
        },
        ..Default::default()
    };
    let spi: hal::spi::Spi<'_, Async> = Spi::new_txonly_nosck(p.SPI1, p.PA29, p.HDMA_CH1, spi_config);

    println!("spi freq {}", spi.frequency().0);

    let mut ws = Ws2812::new(spi);
    const NUM_LEDS: usize = 60;
    let mut data: [RGB8; NUM_LEDS] = [RGB8::new(0x00, 0x00, 0x00); NUM_LEDS];

    // Blink the LED's in a blue-green-red-white pattern.

    let mut i = 0;
    loop {
        // data[0].b = i;
        for led in data.iter_mut() {
            led.r = 0x00;
            led.g = 0x00;
            led.b = 0x00;
        }
        ws.write(data.iter().cloned()).unwrap();
        delay.delay_ms(500);
        for led in data.iter_mut() {
            led.r = 0x00;
            led.g = 0x50;
            led.b = 0x00;
        }
        ws.write(data.iter().cloned()).unwrap();
        delay.delay_ms(500);
    }

    // reset

    // let color = [0b110_110_11_u8, 0b0_110_110_1, 0b10_110_110]; // 0xFF
    //let color = [0b100_100_10_u8, 0b0_100_100_1, 0b00_100_100]; // 0x00

    #[rustfmt::skip]
    let color = [
        0b1000_1110, 0b1000_1110, 0b1110_1000, 0b1110_1110,
        0b1110_1110, 0b1000_1110, 0b1110_1000, 0b1110_1110,
        0b1000_1000, 0b1000_1110, 0b1110_1000, 0b1110_1110,
    ]; // 0x00

    let mut buf1 = [0u8; 4 * 3 * 64 * 6 + 140];
    let mut buf2 = [0u8; 4 * 3 * 64 * 6 + 140];
    //    spi.blocking_write(&buf).unwrap();

    Timer::after_millis(100).await;

    let mut led = Output::new(p.PA23, Level::High, Speed::Fast);

    for i in 0..4 * 3 * 64 * 6 {
        buf1[i] = color[i % 12];
    }
    for i in 0..4 * 3 * 64 * 6 {
        buf2[i] = color[(i + 4) % 12];
    }

    loop {
        spi.write(&buf1[..]).await.unwrap();
        delay.delay_ms(500);
        spi.write(&buf2[..]).await.unwrap();
        delay.delay_ms(500);
    }

    info!("Looping");
    loop {
        spi.write(&buf1[..]).await.unwrap();
        delay.delay_ms(1000);
        led.toggle();
    }
}
