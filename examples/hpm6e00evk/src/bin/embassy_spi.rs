#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]
#![feature(impl_trait_in_assoc_type)]
#![feature(abi_riscv_interrupt)]

use embassy_executor::Spawner;
use embassy_time::Timer;
use embedded_graphics::mono_font::ascii::FONT_10X20;
use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::prelude::*;
use embedded_graphics::text::{Text, TextStyle, TextStyleBuilder};
use embedded_io::Write as _;
use hal::gpio::{AnyPin, Level, Output, Pin as _};
use hal::mode::Blocking;
use hal::pac;
use hpm_hal::time::Hertz;
use hpm_hal::{bind_interrupts, peripherals};
use pixelcolor::Rgb111;
use {defmt_rtt as _, hpm_hal as hal};

bind_interrupts!(struct Irqs {
    UART0 => hal::uart::InterruptHandler<peripherals::UART0>;
});

const BANNER: &str = include_str!("../../../assets/BANNER");

#[embassy_executor::task(pool_size = 3)]
async fn blink(pin: AnyPin, interval_ms: u32) {
    // all leds are active low
    let mut led = Output::new(pin, Level::Low, Default::default());

    loop {
        led.toggle();

        Timer::after_millis(interval_ms as u64).await;
    }
}

static mut UART: Option<hal::uart::Uart<'static, Blocking>> = None;
macro_rules! println {
    ($($arg:tt)*) => {
        unsafe {
            if let Some(uart) = UART.as_mut() {
                let _ = writeln!(uart, $($arg)*);
            }
        }
    };
}

pub mod cmds {
    // NOTE: The commands are actually 6 bit, but the MSB of next 10 bit is always 0,
    // So it's a trick to use 8 bit to represent the command.
    pub const NO_UPDATE: u8 = 0x00;
    pub const BLINKING_BLACK: u8 = 0x10;
    pub const BLINKING_INVERSION: u8 = 0x14;
    pub const BLINKING_WHITE: u8 = 0x18;
    pub const ALL_CLEAR: u8 = 0x20;
    pub const VCOM: u8 = 0x40;
    // 0x90 = 0b100100_00
    // Use 4 bit data input
    pub const UPDATE_4BIT: u8 = 0x90;
    // Also apply to Sharp's update mode: 0b100_xxxxx
    pub const UPDATE_1BIT: u8 = 0x88;
}

const WIDTH: usize = 400;
const HEIGHT: usize = 240;

pub mod pixelcolor {
    use embedded_graphics::pixelcolor::raw::RawU4;
    use embedded_graphics::pixelcolor::{PixelColor, Rgb888, RgbColor};
    use embedded_graphics::prelude::RawData;

    #[derive(Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash, Default)]
    pub struct Rgb111(pub RawU4);

    impl Rgb111 {
        pub fn new(d: u8) -> Self {
            Self(RawU4::new(d))
        }
    }

    impl PixelColor for Rgb111 {
        type Raw = RawU4;
    }

    /// 0bRGBx
    impl RgbColor for Rgb111 {
        fn r(&self) -> u8 {
            self.0.into_inner() >> 3
        }

        fn g(&self) -> u8 {
            (self.0.into_inner() >> 2) & 0b01
        }

        fn b(&self) -> u8 {
            (self.0.into_inner() >> 1) & 0b01
        }

        const MAX_R: u8 = 1;

        const MAX_G: u8 = 1;

        const MAX_B: u8 = 1;

        const BLACK: Self = Self(RawU4::new(0b0000));

        const RED: Self = Self(RawU4::new(0b1000));

        const GREEN: Self = Self(RawU4::new(0b0100));

        const BLUE: Self = Self(RawU4::new(0b0010));

        const YELLOW: Self = Self(RawU4::new(0b1100));

        const MAGENTA: Self = Self(RawU4::new(0b1010));

        const CYAN: Self = Self(RawU4::new(0b0110));

        const WHITE: Self = Self(RawU4::new(0b1110));
    }

    impl From<Rgb111> for RawU4 {
        fn from(color: Rgb111) -> Self {
            color.0
        }
    }

    impl From<Rgb888> for Rgb111 {
        fn from(color: Rgb888) -> Self {
            let r = color.r() >> 7;
            let g = color.g() >> 7;
            let b = color.b() >> 7;
            Self(RawU4::new(r << 3 | g << 2 | b << 1))
        }
    }
}

pub struct MemoryLcdFramebuffer {
    // N cmd byte, 2 trailing bytes, 1 byte per 2 pixels, height prefix
    data: [u8; WIDTH * HEIGHT / 2 + HEIGHT + HEIGHT + 2],
}

impl MemoryLcdFramebuffer {
    pub fn new() -> Self {
        let mut this = Self {
            data: [0; WIDTH * HEIGHT / 2 + HEIGHT + HEIGHT + 2],
        };

        for i in 0..HEIGHT {
            this.data[i * (WIDTH / 2 + 2)] = cmds::UPDATE_4BIT;
            this.data[1 + i * (WIDTH / 2 + 2)] = i as u8 + 1;
        }

        this
    }
}

impl OriginDimensions for MemoryLcdFramebuffer {
    fn size(&self) -> Size {
        Size::new(WIDTH as u32, HEIGHT as u32)
    }
}

impl DrawTarget for MemoryLcdFramebuffer {
    type Color = Rgb111;

    type Error = core::convert::Infallible;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>>,
    {
        for Pixel(coord, color) in pixels {
            let x = coord.x as usize;
            let y = coord.y as usize;

            if x >= WIDTH || y >= HEIGHT {
                continue;
            }

            //            let byte = &mut self.data[1 + y * WIDTH / 2 + x / 2 + y];
            let byte = &mut self.data[2 + y * (WIDTH / 2 + 2) + x / 2];

            if x % 2 == 0 {
                *byte = (*byte & 0x0f) | (color.into_storage() << 4) & 0xf0;
            } else {
                *byte = (*byte & 0xf0) | color.into_storage() & 0x0f;
            }
        }
        Ok(())
    }

    fn clear(&mut self, color: Self::Color) -> Result<(), Self::Error> {
        let val = color.into_storage() | (color.into_storage() << 4);
        for i in 0..HEIGHT {
            self.data[2 + i * (WIDTH / 2 + 2)..2 + i * (WIDTH / 2 + 2) + (WIDTH / 2)].fill(val);
        }
        Ok(())
    }
}

#[embassy_executor::main(entry = "hpm_hal::entry")]
async fn main(spawner: Spawner) -> ! {
    let p = hal::init(Default::default());

    defmt::info!("Board init!");

    //let key_a = p.PB24;
    //let key_b = p.PB25;

    let led_r = p.PE14; // PWM1_P_6
    let led_g = p.PE15; // PWM1_P_7
    let led_b = p.PE04; // PWM0_P_4

    spawner.spawn(blink(led_r.degrade(), 1000)).unwrap();
    spawner.spawn(blink(led_g.degrade(), 2000)).unwrap();
    spawner.spawn(blink(led_b.degrade(), 3000)).unwrap();
    defmt::info!("Tasks init!");

    let uart = hal::uart::Uart::new_blocking(p.UART0, p.PA01, p.PA00, Default::default()).unwrap();
    unsafe {
        UART = Some(uart);
    }

    println!("{}", BANNER);

    println!("Clocks: {:?}", hal::sysctl::clocks());
    println!(
        "XPI0: {}Hz (noinit if running from ram)",
        hal::sysctl::clocks().get_clock_freq(pac::clocks::XPI0).0
    );
    println!("MCT0: {}Hz", hal::sysctl::clocks().get_clock_freq(pac::clocks::MCT0).0);

    println!("Hello, world!");

    let mut cs = Output::new(p.PF25, Level::Low, Default::default()); // special CS, active high
    let mut disp = Output::new(p.PB04, Level::Low, Default::default());

    disp.set_high();

    cs.set_high();

    let spi_config = hal::spi::Config {
        frequency: Hertz(5_000_000), // MAX 8MHz
        //frequency: Hertz(1_000_000),
        ..Default::default()
    };
    let mut spi = hal::spi::Spi::new_txonly(p.SPI7, p.PF26, p.PF29, p.HDMA_CH10, spi_config);

    println!("SPI freq {}Hz", spi.frequency().0);

    let mut fb = MemoryLcdFramebuffer::new();
    fb.clear(Rgb111::WHITE).unwrap();
    let mut i = 0;
    let start = riscv::register::mcycle::read64();
    let mut frames = 0_u32;

    let style = MonoTextStyle::new(&FONT_10X20, Rgb111::BLUE);

    loop {
        i += 2;
        if i > 15 {
            i = 2;
        }

        fb.clear(Rgb111::WHITE).unwrap();

        cs.set_high();

        Text::new("Hello, Rust!", Point::new(20, 20), style)
            .draw(&mut fb)
            .unwrap();

        spi.write(&fb.data).await.unwrap();

        cs.set_low();

        frames += 1;

        let elapsed = riscv::register::mcycle::read64() - start;
        let fps = frames as f32 / (elapsed as f32 / hal::sysctl::clocks().cpu0.0 as f32);

        defmt::info!("fps: {}", fps);
    }
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    let mut err = heapless::String::<1024>::new();

    use core::fmt::Write as _;

    write!(err, "panic: {}", _info).ok();

    defmt::info!("{}", err.as_str());
    loop {}
}
