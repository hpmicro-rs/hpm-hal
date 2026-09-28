#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]
#![feature(impl_trait_in_assoc_type)]
#![feature(abi_riscv_interrupt)]

use assign_resources::assign_resources;
use defmt::println;
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_time::{Delay, Duration, Instant, Timer};
use embedded_graphics::image::ImageRaw;
use embedded_graphics::mono_font::ascii::{FONT_10X20, FONT_5X7, FONT_7X13};
use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::pixelcolor::raw::LittleEndian;
use embedded_graphics::pixelcolor::Gray4;
use embedded_graphics::prelude::*;
use embedded_graphics::text::Text;
use embedded_io::Write as _; // `writeln!` provider
use hal::gpio::{Level, Output, Speed};
use hal::i2c::I2c;
use hal::mode::Blocking;
use hal::peripherals;
use hpm_hal::bind_interrupts;
use hpm_hal::gpio::Pin;
use hpm_hal::time::Hertz;
use rand_core::RngCore;
use {defmt_rtt as _, hpm_hal as hal};

bind_interrupts!(struct Irqs {
    I2C1 => hal::i2c::InterruptHandler<peripherals::I2C1>;
});

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
    i2c: I2c1 {
        sda: PB06,
        scl: PB07,
        i2c1: I2C1,
    }
    // SSD1327
    ssd1327: SSD1327Resources {
        sclk: PA27,
        mosi: PA29,
        cs: PA25,
        dc: PA28, // miso
        rst: PA26,
        spi1: SPI1,
        dma_ch0: HDMA_CH0,
    }
}
// - MARK: tasks

#[embassy_executor::task]
async fn oled_task(r: SSD1327Resources) {
    let mut rst = Output::new(r.rst, Level::High, Default::default());
    let dc = Output::new(r.dc, Level::Low, Default::default());
    let cs = Output::new(r.cs, Level::High, Default::default());

    rst.set_low();
    Timer::after_millis(100).await;
    rst.set_high();
    Timer::after_millis(100).await;

    let mut config = hal::spi::Config::default();
    config.frequency = Hertz::mhz(10);
    let mut spi = hal::spi::Spi::new_txonly(r.spi1, r.sclk, r.mosi, r.dma_ch0, config);

    defmt::info!("go !");

    spi.write(&[0xaa_u8; 10]).await; // 1, 2 fail

    let spi_bus = embassy_sync::mutex::Mutex::<NoopRawMutex, _>::new(spi);
    let spi_dev = embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice::new(&spi_bus, cs);

    let mut ssd1327 = edrv_ssd1327::SSD1327::new(spi_dev, dc);
    ssd1327.init().await;

    let mut fb =
        embedded_graphics::framebuffer::Framebuffer::<Gray4, _, LittleEndian, 128, 128, { 128 * 128 / 2 }>::new();

    println!("begin rng");
    let mut rng = unsafe { hal::rng::Rng::new(peripherals::RNG::steal()).unwrap() };

    println!("begin loop");

    let gif: tinygif::Gif<Gray4> = tinygif::Gif::from_slice(include_bytes!("../../bad-apple-128x128.gif")).unwrap();

    fb.clear(Gray4::BLACK).unwrap();
    loop {
        for frame in gif.frames() {
            // fb.clear(Gray4::BLACK).unwrap();
            let start = Instant::now();
            frame.draw(&mut fb).unwrap();

            ssd1327.write_framebuffer(fb.data()).await;

            let remain_delay = ((frame.delay_centis as u64) * 10).saturating_sub(start.elapsed().as_millis());
            Timer::after_millis(remain_delay).await;
        }
    }

    let mut fbx = [0u8; 128 * 128 / 2];

    loop {
        rng.fill_bytes(&mut fbx);

        //        defmt::info!("frame data {:?}", &fbx[0..100]);

        //  defmt::info!("frame update");

        ssd1327.write_framebuffer(&fbx).await;

        Timer::after(Duration::from_millis(10)).await;
    }

    let (w, _h, img1) = text_image::text_image!(
        text = "9DoF 数据监测",
        font = "./LXGWWenKaiScreen.ttf",
        font_size = 20.0,
        gamma = 1.0,
        Gray4,
    );
    let (w, _h, img2) = text_image::text_image!(
        text = "9DoF 数据监测",
        font = "./LXGWWenKaiScreen.ttf",
        font_size = 20.0,
        gamma = 1.0,
        inverse,
        Gray4,
    );
    let label_img1: ImageRaw<Gray4> = ImageRaw::new(img1, w);
    let label_img2: ImageRaw<Gray4> = ImageRaw::new(img2, w);

    ssd1327.clear(0x00).await;

    loop {
        fb.clear(Gray4::WHITE);

        //  label_img1.draw(&mut fb.translated(Point::new(4, 10))).unwrap();

        ssd1327.write_framebuffer(fb.data()).await;

        Timer::after(Duration::from_millis(100)).await;
        fb.clear(Gray4::BLACK).unwrap();

        //  label_img2.draw(&mut fb.translated(Point::new(4, 10))).unwrap();

        ssd1327.write_framebuffer(fb.data()).await;

        Timer::after(Duration::from_millis(100)).await;
    }

    loop {
        ssd1327.clear(0x00).await;

        Timer::after(Duration::from_millis(100)).await;

        ssd1327.clear(0xff).await;

        Timer::after(Duration::from_millis(100)).await;
    }

    let (w, _h, img) = text_image::text_image!(
        text = "9DoF 数据监测",
        font = "./LXGWWenKaiScreen.ttf",
        font_size = 20.0,
        gamma = 1.0,
        Gray4,
    );
    let label_img: ImageRaw<Gray4> = ImageRaw::new(img, w);

    let style = MonoTextStyle::new(&FONT_7X13, Gray4::new(3));

    let mut buf = heapless::String::<128>::new();

    let mut i = 0;
    loop {
        Timer::after(Duration::from_millis(100)).await;

        buf.clear();
        fb.clear(Gray4::BLACK).unwrap();

        i += 1;

        label_img.draw(&mut fb.translated(Point::new(4, 10))).unwrap();

        core::fmt::write(&mut buf, format_args!("Hello HPM! {}", i)).unwrap();

        Text::new(buf.as_str(), Point::new(5, 50), style).draw(&mut fb).unwrap();

        ssd1327.write_framebuffer(fb.data()).await.unwrap();
    }
}

// - MARK: main

#[embassy_executor::main(entry = "hpm_hal::entry")]
async fn main(spawner: embassy_executor::Spawner) -> ! {
    let config = hal::Config::default();
    let p = hal::init(config);

    let r = split_resources!(p);

    let mut led = Output::new(r.leds.red, Level::Low, Default::default());

    spawner.spawn(oled_task(r.ssd1327)).unwrap();

    let mut i2c_config = hal::i2c::Config::default();
    {
        use hal::i2c::*;
        i2c_config.mode = I2cMode::Fast;
        i2c_config.timeout = Duration::from_secs(1);
    }
    let i2c = I2c::new(r.i2c.i2c1, r.i2c.scl, r.i2c.sda, Irqs, p.HDMA_CH10, i2c_config);

    let i2c_bus = embassy_sync::mutex::Mutex::<NoopRawMutex, _>::new(i2c);

    let i2c_dev1 = I2cDevice::new(&i2c_bus);
    let i2c_dev2 = I2cDevice::new(&i2c_bus);

    let mut gyro = edrv_l3g4200d::L3G4200D::new_primary(i2c_dev1);
    gyro.init(Default::default()).await.unwrap();

    let mut accel = edrv_adxl345::ADXL345::new_primary(i2c_dev2);
    accel.init(Default::default()).await.unwrap();

    let time_step = 0.01;

    let mut pitch = 0.0;
    let mut roll = 0.0;
    let mut yaw = 0.0;

    loop {
        led.toggle();

        let (x, y, z) = gyro.read_normalized().await.unwrap();

        pitch += x as f32 * time_step;
        roll += y as f32 * time_step;
        yaw += z as f32 * time_step;

        defmt::info!("GYRO: {:?}", (x, y, z));
        defmt::debug!("Pitch: {}, Roll: {}, Yaw: {}", pitch, roll, yaw);

        Timer::after(Duration::from_millis(100)).await;
    }
}

#[panic_handler]
unsafe fn panic(info: &core::panic::PanicInfo) -> ! {
    defmt::println!("panic!\n {}", defmt::Debug2Format(info));

    loop {}
}
