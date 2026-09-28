#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]
#![feature(impl_trait_in_assoc_type)]

use assign_resources::assign_resources;
use defmt::println;
use embassy_executor::Spawner;
use embassy_time::{Delay, Timer};
use embedded_graphics::framebuffer::Framebuffer;
use embedded_graphics::image::ImageRaw;
use embedded_graphics::mono_font::ascii::FONT_10X20;
use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::pixelcolor::raw::LittleEndian;
use embedded_graphics::pixelcolor::Gray4;
use embedded_graphics::prelude::{Point, *};
use embedded_graphics::primitives::{Line, PrimitiveStyleBuilder};
use embedded_graphics::text::Text;
use embedded_hal::digital::OutputPin;
use embedded_io::Write as _;
use hal::gpio::{AnyPin, Flex, Pin};
use hal::pac::MCHTMR;
use hal::{pac, peripherals};
use hpm_hal::gpio::{Level, Output};
use hpm_hal::mode::Blocking;
use hpm_hal::spi::Spi;
use hpm_hal::time::Hertz;
use {defmt_rtt as _, hpm_hal as hal};

const BOARD_NAME: &str = "HPM5300EVK";
const BANNER: &str = include_str!("../../../assets/BANNER");

assign_resources! {
    leds: Leds {
        // PWM1_P_7
        red: PA23,
    }
    // FT2232 UART, default uart
    uart: Uart0 {
        tx: PA00,
        rx: PA01,
        uart0: UART0,
    }
    // SSD1327
    ssd1327: SSD1327Resources {
        sclk: PA27,
        mosi: PA29,
        cs: PB02,
        dc: PA25,
        rst: PA26,
        spi1: SPI1,
        dma_ch0: HDMA_CH0,
    }
}

macro_rules! println {
    ($($arg:tt)*) => {
        let _ = writeln!(unsafe {(&mut *(&raw mut UART)).as_mut().unwrap()}, $($arg)*);
    };
}

static mut UART: Option<hal::uart::Uart<'static, Blocking>> = None;

pub mod cmds {
    // Fundamental Commands
    pub const SET_COLUMN_ADDRESS: u8 = 0x15;
    pub const SET_ROW_ADDRESS: u8 = 0x75;
    pub const SET_CONTRAST_CURRENT: u8 = 0x81;
    pub const SET_REMAP: u8 = 0xA0;
    pub const SET_DISPLAY_START_LINE: u8 = 0xA1;
    pub const SET_DISPLAY_OFFSET: u8 = 0xA2;
    pub const SET_DISPLAY_MODE: u8 = 0xA4;
    pub const SET_MULTIPLEX_RATIO: u8 = 0xA8;
    pub const FUNCTION_SELECTION_A: u8 = 0xAB;
    pub const SET_DISPLAY_ON: u8 = 0xAF;
    pub const SET_DISPLAY_OFF: u8 = 0xAE;
    pub const SET_PHASE_LENGTH: u8 = 0xB1;
    pub const SET_FRONT_CLOCK_DIVIDER: u8 = 0xB3;
    pub const SET_GPIO: u8 = 0xB5;
    pub const SET_SECOND_PRECHARGE_PERIOD: u8 = 0xB6;
    pub const SET_GRAY_SCALE_TABLE: u8 = 0xB8;
    pub const SELECT_DEFAULT_LINEAR_GRAY_SCALE_TABLE: u8 = 0xB9;
    pub const SET_PRECHARGE_VOLTAGE: u8 = 0xBC;
    pub const SET_VCOMH_VOLTAGE: u8 = 0xBE;
    pub const FUNCTION_SELECTION_B: u8 = 0xD5;
    pub const SET_COMMAND_LOCK: u8 = 0xFD;

    // Graphic Acceleration Commands
    pub const HORIZONTAL_SCROLL_SETUP: u8 = 0x26;
    pub const HORIZONTAL_SCROLL_SETUP_ALT: u8 = 0x27; // Alternative command for Horizontal Scroll Setup
    pub const DEACTIVATE_SCROLL: u8 = 0x2E;
    pub const ACTIVATE_SCROLL: u8 = 0x2F;
}

pub struct SSD1327<SPI: embedded_hal_async::spi::SpiBus, DC: OutputPin, CS: OutputPin> {
    spi: SPI,
    dc: DC,
    cs: CS,
}

impl<SPI: embedded_hal_async::spi::SpiBus, DC: OutputPin, CS: OutputPin> SSD1327<SPI, DC, CS> {
    pub fn new(spi: SPI, cs: CS, dc: DC) -> Self {
        Self { spi, cs, dc }
    }

    pub async fn init(&mut self) {
        self.write_command(&[cmds::SET_DISPLAY_OFF]).await;
        self.write_command(&[cmds::SET_COLUMN_ADDRESS, 0x00, 0x3F]).await;
        self.write_command(&[cmds::SET_ROW_ADDRESS, 0x00, 0x7F]).await;
        self.write_command(&[cmds::SET_CONTRAST_CURRENT, 0x80]).await;

        // address remap
        self.write_command(&[cmds::SET_REMAP, 0x51]).await;

        self.write_command(&[cmds::SET_DISPLAY_START_LINE, 0x00]).await;
        self.write_command(&[cmds::SET_DISPLAY_OFFSET, 0x00]).await;

        self.write_command(&[cmds::SET_MULTIPLEX_RATIO, 0x7F]).await;
        self.write_command(&[cmds::SET_PHASE_LENGTH, 0x11]).await; // gray scale tune

        // gamma setting
        // 0xb8: SET_GRAY_SCALE_TABLE, [u8; 15]
        // 0xb9: SET_DEFAULT_LINEAR_GRAY_SCALE_TABLE
        //self.send_cmd_data(0xb8, &[1,2,30,40,5,6,7,8,9,10,11,12,13,14,0b11111])?;
        self.write_command(&[cmds::SELECT_DEFAULT_LINEAR_GRAY_SCALE_TABLE])
            .await;

        self.write_command(&[cmds::SET_FRONT_CLOCK_DIVIDER, 0x00]).await;
        self.write_command(&[cmds::FUNCTION_SELECTION_A, 0x01]).await;
        self.write_command(&[cmds::SET_SECOND_PRECHARGE_PERIOD, 0x08]).await;
        self.write_command(&[cmds::SET_VCOMH_VOLTAGE, 0x0f]).await;
        self.write_command(&[cmds::SET_PRECHARGE_VOLTAGE, 0x08]).await;
        self.write_command(&[cmds::FUNCTION_SELECTION_B, 0x62]).await;
        self.write_command(&[cmds::SET_COMMAND_LOCK, 0x12]).await;
        self.write_command(&[cmds::SET_DISPLAY_MODE]).await; // display mode normal
        self.write_command(&[cmds::SET_DISPLAY_ON]).await;

        self.write_command(&[cmds::DEACTIVATE_SCROLL]).await;
    }

    pub async fn send_framebuffer(&mut self, fb: &[u8]) {
        self.write_command(&[cmds::SET_COLUMN_ADDRESS, 0x00, 0x3F]).await;
        self.write_command(&[cmds::SET_ROW_ADDRESS, 0x00, 0x7F]).await;
        self.write_data(fb).await;
    }

    async fn write_command(&mut self, cmd: &[u8]) {
        self.dc.set_low().unwrap();
        self.cs.set_low().unwrap();
        self.spi.write(cmd).await.unwrap();
        self.cs.set_high().unwrap();
    }

    async fn write_data(&mut self, data: &[u8]) {
        self.dc.set_high().unwrap();
        self.cs.set_low().unwrap();
        self.spi.write(data).await.unwrap();
        self.cs.set_high().unwrap();

        self.dc.set_low().unwrap();
    }
}

#[embassy_executor::task]
async fn oled_task(r: SSD1327Resources) {
    let mut config = hal::spi::Config::default();
    config.frequency = Hertz::mhz(20);
    let spi = Spi::new_txonly(r.spi1, r.sclk, r.mosi, r.dma_ch0, config);
    let mut rst = Output::new(r.rst, Level::High, Default::default());
    let mut dc = Output::new(r.dc, Level::Low, Default::default());
    let mut cs = Output::new(r.cs, Level::High, Default::default());

    rst.set_low();
    Timer::after_millis(100).await;
    rst.set_high();
    Timer::after_millis(100).await;

    //let spi_bus = AtomicCell::new(spi);
    //let spi_dev = AtomicDevice::new(&spi_bus, cs, Delay).unwrap();

    let mut ssd1327 = SSD1327::new(spi, cs, dc);
    ssd1327.init().await;

    let mut fb = Framebuffer::<
        Gray4,
        _,
        LittleEndian,
        128,
        128,
        { embedded_graphics::framebuffer::buffer_size::<Gray4>(128, 128) },
    >::new();

    /*
    let (w, h, img) = text_image::text_image!(
        text = "Rust yyds",
        font = "默陌静斋笔迹.ttf",
        font_size = 28.0,
        gamma = 1.8,
        Gray4,
    );
    let image: ImageRaw<Gray4> = ImageRaw::new(img, w);
    image.draw(&mut fb.translated(Point::new(2, 65))).unwrap();

    loop {
        ssd1327.send_framebuffer(fb.data()).await;

        Timer::after_millis(1000).await;
    }
    */

    let (w, _h, img_raw) = text_image::gray_image!("pattern1_128x128.png", gamma = 2.8, Gray4);
    let image1: ImageRaw<Gray4> = ImageRaw::new(img_raw, w);

    let (w, _h, img_raw) = text_image::gray_image!("pattern2_128x128.png", Gray4);
    let image2: ImageRaw<Gray4> = ImageRaw::new(img_raw, w);

    let (w, _h, img_raw) = text_image::gray_image!("pattern3_128x128.png", Gray4);
    let image3: ImageRaw<Gray4> = ImageRaw::new(img_raw, w);

    let (w, _h, img_raw) = text_image::gray_image!("pattern4_128x128.png", Gray4);
    let image4: ImageRaw<Gray4> = ImageRaw::new(img_raw, w);

    let (w, _h, img_raw) = text_image::gray_image!("pattern5_128x128.png", Gray4);
    let image5: ImageRaw<Gray4> = ImageRaw::new(img_raw, w);

    /*
    let mut gif: tinygif::Gif<Gray4> = tinygif::Gif::from_slice(include_bytes!("../../pika_anim_128x128.gif")).unwrap();
    loop {
        if true {
            for frame in gif.frames() {
                // fb.clear(Gray4::BLACK).unwrap();

                frame.draw(&mut fb).unwrap();

                ssd1327.send_framebuffer(fb.data()).await;
                Timer::after_millis(frame.delay_centis as u64 * 10).await;
            }
        }
    }
    */

    loop {
        for (img1, img2) in [
            (&image1, &image2),
            (&image2, &image3),
            (&image3, &image4),
            (&image4, &image5),
            (&image5, &image1),
        ]
        .iter()
        {
            for i in (0..128).step_by(4) {
                fb.clear(Gray4::BLACK).unwrap();

                img1.draw(&mut fb.translated(Point::new(0, -i))).unwrap();
                img2.draw(&mut fb.translated(Point::new(0, 128 - i))).unwrap();

                ssd1327.send_framebuffer(fb.data()).await;

                // Timer::after_millis(1).await;

                if i == 0 {
                    Timer::after_millis(1000).await;
                }
            }
        }
    }

    loop {
        image1.draw(&mut fb).unwrap();
        ssd1327.send_framebuffer(fb.data()).await;
        Timer::after_millis(1000).await;

        image2.draw(&mut fb).unwrap();
        ssd1327.send_framebuffer(fb.data()).await;
        Timer::after_millis(1000).await;

        image3.draw(&mut fb).unwrap();
        ssd1327.send_framebuffer(fb.data()).await;
        Timer::after_millis(1000).await;

        image4.draw(&mut fb).unwrap();
        ssd1327.send_framebuffer(fb.data()).await;
        Timer::after_millis(1000).await;
    }

    loop {
        //       fb.clear(Gray4::BLACK).unwrap();

        for i in 0..8 {
            let offset = (i as i32) * 18;
            let style = MonoTextStyle::new(&FONT_10X20, Gray4::new(i));
            Text::new("Hello HPM!", Point::new(19, offset), style)
                .draw(&mut fb)
                .unwrap();
        }

        //   ssd1327.send_framebuffer(fb.data()).await;

        Timer::after_millis(500).await;
    }
}

#[embassy_executor::main(entry = "hpm_hal::entry")]
async fn main(spawner: Spawner) -> ! {
    let p = hal::init(Default::default());
    let r = split_resources!(p);

    // let button = Input::new(p.PA03, Pull::Down); // hpm5300evklite, BOOT1_KEY
    let uart = hal::uart::Uart::new_blocking(r.uart.uart0, r.uart.rx, r.uart.tx, Default::default()).unwrap();
    unsafe {
        UART = Some(uart);
    }

    println!("{}", BANNER);
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
    println!("==============================");
    let mut led = Output::new(r.leds.red, hal::gpio::Level::Low, hal::gpio::Speed::default());

    spawner.spawn(oled_task(r.ssd1327)).unwrap();

    loop {
        Timer::after_millis(1000).await;

        defmt::info!("tick {}", MCHTMR.mtime().read());

        led.toggle();
    }
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    println!("\n\n\nPANIC:\n{}", info);

    loop {}
}
