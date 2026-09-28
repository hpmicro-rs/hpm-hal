#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]
#![feature(impl_trait_in_assoc_type)]
#![feature(abi_riscv_interrupt)]

use assign_resources::assign_resources;
use defmt::{info, println};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Delay, Timer};
use embedded_graphics::framebuffer::Framebuffer;
use embedded_graphics::image::ImageRaw;
use embedded_graphics::mono_font::ascii::FONT_10X20;
use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::pixelcolor::raw::{BigEndian, LittleEndian};
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_graphics::text::Text;
use embedded_hal::delay::DelayNs;
use embedded_hal_bus::spi::AtomicDevice;
use embedded_hal_bus::util::AtomicCell;
use hal::{pac, peripherals};
use hpm_hal::bind_interrupts;
use hpm_hal::gpio::{Input, Level, NoPin, Output};
use hpm_hal::interrupt::InterruptExt as _;
use hpm_hal::pac::qei::vals;
use hpm_hal::pac::sei::regs::In;
use hpm_hal::time::Hertz;
use {defmt_rtt as _, hpm_hal as hal};

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
    // 0.96" 160x80 IPS TFT LCD
    lcd: ST7789Resources {
        sclk: PA27,
        mosi: PA29,
        cs: PA25,
        dc: PA28, // miso
        rst: PA26,
        spi1: SPI1,
        dma_ch0: HDMA_CH0,
    }
}

/*
#[embassy_executor::task]
async fn display_task(r: ST7789Resources) {
    use edrv_st7735::blocking::ST7735;
    use edrv_st7735::Display160x80Type2;

    let mut rst = Output::new(r.rst, Level::High, Default::default());
    let cs = Output::new(r.cs, Level::High, Default::default());
    let dc = Output::new(r.dc, Level::High, Default::default());

    rst.set_low();
    Timer::after_millis(120).await;
    rst.set_high();
    Timer::after_millis(120).await;

    let mut config = hal::spi::Config::default();
    config.frequency = Hertz::mhz(2);
    let spi = hal::spi::Spi::new_blocking_txonly(r.spi1, r.sclk, r.mosi, config);
    let spi_bus = AtomicCell::new(spi);

    let spi_dev = AtomicDevice::new(&spi_bus, cs, Delay).unwrap();
    let mut display: ST7735<Display160x80Type2, _, _> = ST7735::new(spi_dev, dc);

    println!("dev ok");

    let mut fb = Framebuffer::<
        Rgb565,
        _,
        BigEndian,
        160,
        80,
        { embedded_graphics::framebuffer::buffer_size::<Rgb565>(160, 80) },
    >::new();

    println!("fb ok");

    display.init(&mut Delay);

    println!("init ok");

    loop {
        fb.clear(Rgb565::GREEN).unwrap();
        display.write_framebuffer(fb.data());
        Timer::after_millis(1000).await;

        fb.clear(Rgb565::RED).unwrap();
        display.write_framebuffer(fb.data());

        Timer::after_millis(1000).await;

        fb.clear(Rgb565::MAGENTA).unwrap();
        display.write_framebuffer(fb.data());
        Timer::after_millis(1000).await;
        println!("tick");
    }
}
*/

#[embassy_executor::task]
async fn display_task(r: ST7789Resources) {
    use edrv_st7735::{Display160x80Type2, ST7735};

    let mut rst = Output::new(r.rst, Level::High, Default::default());
    let cs = Output::new(r.cs, Level::High, Default::default());
    let dc = Output::new(r.dc, Level::High, Default::default());

    rst.set_low();
    Timer::after_millis(120).await;
    rst.set_high();
    Timer::after_millis(120).await;

    defmt::info!("rst");

    let mut config = hal::spi::Config::default();
    config.frequency = Hertz::khz(28_000);
    let spi = hal::spi::Spi::new_txonly(r.spi1, r.sclk, r.mosi, r.dma_ch0, config);

    let spi_bus = Mutex::<NoopRawMutex, _>::new(spi);
    let spi_dev = embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice::new(&spi_bus, cs);
    let mut display: ST7735<Display160x80Type2, _, _> = ST7735::new(spi_dev, dc);

    let mut fb = Framebuffer::<
        Rgb565,
        _,
        BigEndian,
        160,
        80,
        { embedded_graphics::framebuffer::buffer_size::<Rgb565>(160, 80) },
    >::new();

    display.init(&mut Delay).await;

    defmt::info!("init ok");
    let style = MonoTextStyle::new(&FONT_10X20, Rgb565::BLACK);

    loop {
        fb.clear(Rgb565::CSS_WHEAT).unwrap();

        Text::new("Hello HPM!", Point::new(19, 50), style)
            .draw(&mut fb)
            .unwrap();

        display.write_framebuffer(fb.data()).await;
        Timer::after_millis(1000).await;

        fb.clear(Rgb565::CSS_TURQUOISE).unwrap();
        display.write_framebuffer(fb.data()).await;

        Timer::after_millis(1000).await;
    }
}

#[embassy_executor::main(entry = "hpm_hal::entry")]
async fn main(spawner: embassy_executor::Spawner) -> ! {
    let p = hal::init(Default::default());
    let r = split_resources!(p);

    defmt::info!("Clock summary:");
    defmt::info!("  CPU0:\t{}Hz", hal::sysctl::clocks().cpu0.0);
    defmt::info!("  AHB:\t{}Hz", hal::sysctl::clocks().ahb.0);

    // WS PB10
    // LR GND
    // other SPI2

    // let mut ws = Output::new(p.PB10, Level::High, Default::default());

    spawner.must_spawn(display_task(r.lcd));

    let mut cfg = hal::spi::Config::default();
    cfg.frequency = Hertz::hz(2_000_000);
    let mut spi = hal::spi::Spi::new_blocking(p.SPI2, p.PB11, p.PB13, p.PB12, cfg);

    // 24bit, 24kpbs,
    const SIZE: usize = 256;

    let mut out_buf = [0u32; SIZE];
    for i in 0..SIZE {
        if i % 2 == 0 {
            out_buf[i] = 0x00000000;
        } else {
            out_buf[i] = 0xFFFFFFFF;
        }
    }

    let mut txconf = hal::spi::TransferConfig::default();
    txconf.transfer_mode = hal::spi::TransMode::WRITE_READ_TOGETHER;

    loop {
        let mut b = [0u32; SIZE];
        spi.blocking_transfer(&mut b, &out_buf, &txconf).unwrap();

        let avg = b.iter().step_by(2).fold(0, |acc, x| acc + x) / 512;

        println!("I2S => {:08x}", avg);

        Timer::after_millis(1000).await;
    }

    let mut buf = [0u32; 128];

    println!("done");

    loop {
        spi.blocking_read(&mut buf).unwrap();

        println!("=> {:?}", buf);
        Timer::after_millis(1000).await;
    }

    // SEI0
    // CK: PB05
    // DE: PB04
    // RX: PB15
    // TX: __

    /*

    let mut ck = Output::new(p.PB05, Level::High, Default::default());
    let mut ws = Output::new(p.PB04, Level::High, Default::default());
    let sd = Input::new(p.PB15, hpm_hal::gpio::Pull::None);

    let mut buf = [0u32; 128];

    let mut i = 0;
    loop {
        let mut data = 0u32;

        ws.set_low();

        for _ in 0..32 {
            ck.set_low();
            Delay.delay_us(1);
            ck.set_high();
            Delay.delay_us(1);
        }

        ws.set_high();

        for _ in 0..32 {
            ck.set_low();
            Delay.delay_us(1);
            ck.set_high();
            Delay.delay_us(1);

            if sd.is_high() {
                data = (data << 1) | 1;
            } else {
                data = (data << 1) | 0;
            }
        }

        buf[i] = data;
        i += 1;
        if i >= 128 {
            i = 0;
            defmt::info!("=> {:?}", buf);
        }
    }

    let mut led = Output::new(p.PA23, Level::High, Default::default());

    loop {
        led.toggle();

        Timer::after_millis(500).await;
    }
    */
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    defmt::error!("{}", defmt::Debug2Format(info));

    loop {}
}
