#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]
#![feature(impl_trait_in_assoc_type)]

use assign_resources::assign_resources;
use defmt::println;
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
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
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::OutputPin;
use embedded_hal_async::spi::SpiDevice;
use embedded_io::Write as _;
use hal::gpio::{AnyPin, Flex, Pin};
use hal::pac::MCHTMR;
use hal::{pac, peripherals};
use hpm_hal::gpio::{Level, Output};
use hpm_hal::mode::{Async, Blocking};
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
    pwm3610: PWM3610Resources {
        sclk: PA27,
        mosi: PA29,
        cs: PA26,
        spi1: SPI1,
        dma_ch0: HDMA_CH0,
        dma_ch1: HDMA_CH1,
    }
}

/* macro_rules! println {
    ($($arg:tt)*) => {
        let _ = writeln!(unsafe {(&mut *(&raw mut UART)).as_mut().unwrap()}, $($arg)*);
    };
}

static mut UART: Option<hal::uart::Uart<'static, Blocking>> = None; */

pub mod regs {
    pub const PROD_ID: u8 = 0x00;
    pub const REV_ID: u8 = 0x01;
    pub const MOTION: u8 = 0x02;
    pub const DELTA_X_L: u8 = 0x03;
    pub const DELTA_Y_L: u8 = 0x04;
    pub const DELTA_XY_H: u8 = 0x05;
    pub const SQUAL: u8 = 0x06;
    pub const SHUTTER_HIGHER: u8 = 0x07;
    pub const SHUTTER_LOWER: u8 = 0x08;
    pub const PIX_MAX: u8 = 0x09;
    pub const PIX_AVG: u8 = 0x0a;
    pub const PIX_MIN: u8 = 0x0b;
    pub const CRC0: u8 = 0x0c;
    pub const CRC1: u8 = 0x0d;
    pub const CRC2: u8 = 0x0e;
    pub const CRC3: u8 = 0x0f;
    pub const SELF_TEST: u8 = 0x10;
    pub const PERFORMANCE: u8 = 0x11;
    pub const BURST_READ: u8 = 0x12;
    pub const RUN_DOWNSHIFT: u8 = 0x1b;
    pub const REST1_RATE: u8 = 0x1c;
    pub const REST1_DOWNSHIFT: u8 = 0x1d;
    pub const REST2_RATE: u8 = 0x1e;
    pub const REST2_DOWNSHIFT: u8 = 0x1f;
    pub const REST3_RATE: u8 = 0x20;
    pub const OBSERVATION1: u8 = 0x2d;
    pub const DTEST2_PAD: u8 = 0x32;
    pub const PIXEL_GRAB: u8 = 0x35;
    pub const FRAME_GRAB: u8 = 0x36;
    pub const POWER_UP_RESET: u8 = 0x3a;
    pub const SHUTDOWN: u8 = 0x3b;
    pub const NOT_REV_ID: u8 = 0x3e;
    pub const NOT_PROD_ID: u8 = 0x3f;
    pub const SPI_CLK_ON_REQ: u8 = 0x41;
    pub const PRBS_TEST_CTL: u8 = 0x47;
    pub const SPI_PAGE0: u8 = 0x7f;
    pub const RES_STEP: u8 = 0x85;
    pub const VCSEL_CTL: u8 = 0x9e;
    pub const LSR_CONTROL: u8 = 0x9f;
    pub const SPI_PAGE1: u8 = 0xff;
}

pub const PMW3610_POWERUP_CMD_RESET: u8 = 0x5A;
pub const PMW3610_POWERUP_CMD_WAKEUP: u8 = 0x96;

/// Counts Per Inch
#[derive(Debug, PartialEq, defmt::Format)]
pub enum Cpi {
    Cpi200 = 1, // 0x1
    Cpi400 = 2, // 0x2
    Cpi600 = 3, // 0x3
    Cpi800 = 4,
    Cpi1000 = 5,
    Cpi1200 = 6,
    Cpi1400 = 7,
    Cpi1600 = 8,
    Cpi1800 = 9,
    Cpi2000 = 10,
    Cpi2200 = 11,
    Cpi2400 = 12,
    Cpi2600 = 13,
    Cpi2800 = 14,
    Cpi3000 = 15,
    Cpi3200 = 16, // 0x10
}

impl Cpi {
    fn from_raw(raw: u8) -> Self {
        match raw {
            0x1 => Cpi::Cpi200,
            0x2 => Cpi::Cpi400,
            0x3 => Cpi::Cpi600,
            0x4 => Cpi::Cpi800,
            0x5 => Cpi::Cpi1000,
            0x6 => Cpi::Cpi1200,
            0x7 => Cpi::Cpi1400,
            0x8 => Cpi::Cpi1600,
            0x9 => Cpi::Cpi1800,
            0xA => Cpi::Cpi2000,
            0xB => Cpi::Cpi2200,
            0xC => Cpi::Cpi2400,
            0xD => Cpi::Cpi2600,
            0xE => Cpi::Cpi2800,
            0xF => Cpi::Cpi3000,
            0x10 => Cpi::Cpi3200,
            _ => panic!("Invalid CPI value: {}", raw),
        }
    }
}

/*
/* Burst register offsets */
#define BURST_MOTION		0
#define BURST_DELTA_X_L		1
#define BURST_DELTA_Y_L		2
#define BURST_DELTA_XY_H	3
#define BURST_SQUAL		4
#define BURST_SHUTTER_HI	5
#define BURST_SHUTTER_LO	6
 */
#[derive(Debug, defmt::Format)]
pub struct BurstReadData {
    pub motion: u8,
    pub delta_x: i16,
    pub delta_y: i16,
    pub squal: u8,
    pub shutter: u16,
}

impl BurstReadData {}

pub struct PWM3610 {
    spi: Spi<'static, Async>,
    cs: Output<'static>,
    smart_mode: bool,
}

impl PWM3610 {
    pub fn new(spi: Spi<'static, Async>, cs: Output<'static>) -> Self {
        Self {
            spi,
            cs,
            smart_mode: true,
        }
    }

    pub async fn init(&mut self) -> Result<(), hal::spi::Error> {
        // step 1: power up
        self.write_reg(regs::POWER_UP_RESET, PMW3610_POWERUP_CMD_RESET).await?;
        Delay.delay_ms(10);

        // step 2: clear ob1
        self.write_reg(regs::OBSERVATION1, 0x00).await?;
        Delay.delay_ms(200);

        // step 3: check ob1
        let value = self.read_reg(regs::OBSERVATION1).await?;
        if value & 0x0F != 0x0F {
            defmt::error!("OBSERVATION1: 0x{:02x}", value);
        }

        const PMW3610_PRODUCT_ID: u8 = 0x3e;

        let prod_id = self.read_reg(regs::PROD_ID).await?;
        if prod_id != PMW3610_PRODUCT_ID {
            defmt::error!("PROD_ID: 0x{:02x}", prod_id);
        }
        Delay.delay_ms(50);

        // step 4: configure
        // clear motion registers first (required in datasheet)
        for i in 2..=5 {
            let _value = self.read_reg(i).await?;
        }

        // set CPI
        self.set_cpi(Cpi::Cpi800).await?;

        // TODO: set_downshift_time
        // TODO: set_sample_time

        // reset SPI port
        //    self.cs.set_low();
        //  self.cs.set_high();

        Ok(())
    }

    pub async fn set_cpi(&mut self, cpi: Cpi) -> Result<(), hal::spi::Error> {
        self.write_reg(regs::RES_STEP, cpi as u8).await?;
        Ok(())
    }

    pub async fn burst_read(&mut self) -> Result<BurstReadData, hal::spi::Error> {
        /*
        /* Burst register offsets */
        #define BURST_MOTION		0
        #define BURST_DELTA_X_L		1
        #define BURST_DELTA_Y_L		2
        #define BURST_DELTA_XY_H	3
        #define BURST_SQUAL		4
        #define BURST_SHUTTER_HI	5
        #define BURST_SHUTTER_LO	6
         */

        let mut out = [0u8; 7];

        let in_buf = [regs::BURST_READ];
        self.write_read(&in_buf, &mut out).await?;

        // smart mode enable
        self.write_reg(0x32, 0x00).await?;

        let delta_x = (((out[3] as u16) << 4) & 0xf00) | (out[1] as u16);
        let delta_y = (((out[3] as u16) << 8) & 0xf00) | (out[2] as u16);

        let data = BurstReadData {
            motion: out[0],
            delta_x: sign_extend(delta_x, 12),
            delta_y: sign_extend(delta_y, 12),
            squal: out[4],
            shutter: u16::from_le_bytes([out[5], out[6]]),
        };
        Ok(data)
    }

    async fn read_reg(&mut self, reg: u8) -> Result<u8, hal::spi::Error> {
        let in_buf = [reg];
        let mut out_buf = [0u8; 1];
        self.write_read(&in_buf, &mut out_buf).await?;
        Ok(out_buf[0])
    }

    async fn write_reg(&mut self, reg: u8, val: u8) -> Result<(), hal::spi::Error> {
        let in_buf = [0b1000_0000 | reg, val];
        self.write_read(&in_buf, &mut []).await?;
        Ok(())
    }

    async fn write_read(&mut self, write: &[u8], read: &mut [u8]) -> Result<(), hal::spi::Error> {
        self.cs.set_low();
        self.spi.write(write).await?;
        Delay.delay_us(4);
        self.spi.read(read).await?;
        self.cs.set_high();
        Ok(())
    }
}

fn sign_extend(val: u16, bits: u8) -> i16 {
    let sign_bit = 1 << (bits - 1);
    if val & sign_bit != 0 {
        val as i16 - (1 << bits)
    } else {
        val as i16
    }
}

#[embassy_executor::task]
async fn oled_task(r: PWM3610Resources) {
    let mut config = hal::spi::Config::default();
    config.frequency = Hertz::mhz(1);
    let mut spi = Spi::new_half_duplex(r.spi1, r.sclk, r.mosi, r.dma_ch0, r.dma_ch1, config);
    let mut cs = Output::new(r.cs, Level::High, Default::default());

    let mut sensor = PWM3610::new(spi, cs);

    //    let spi_bus = embassy_sync::mutex::Mutex::<NoopRawMutex, _>::new(spi);
    //   let mut spi_dev = embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice::new(&spi_bus, cs);

    let prod_id = sensor.read_reg(regs::PROD_ID).await.unwrap();
    println!("PROD_ID: 0x{:02x}", prod_id);

    let rev_id = sensor.read_reg(regs::REV_ID).await.unwrap();
    println!("REV_ID: 0x{:02x}", rev_id);

    let cpi = sensor.read_reg(regs::RES_STEP).await.unwrap() & 0x1f;
    // defmt::info!("CPI: {:?}", Cpi::from_raw(cpi));
    defmt::info!("CPI: 0x{:02x} {}", cpi, cpi);

    sensor.init().await;

    let prod_id = sensor.read_reg(regs::PROD_ID).await.unwrap();
    println!("PROD_ID: 0x{:02x}", prod_id);

    for i in 2..=5 {

        // defmt::info!("reg[{:02x}] = {:02x}", i, out);
    }

    loop {
        Timer::after_millis(10).await;
        let c = sensor.burst_read().await.unwrap();

        defmt::info!("burst_read: {:?}", c);
    }
}

#[embassy_executor::main(entry = "hpm_hal::entry")]
async fn main(spawner: Spawner) -> ! {
    let p = hal::init(Default::default());
    let r = split_resources!(p);

    // let button = Input::new(p.PA03, Pull::Down); // hpm5300evklite, BOOT1_KEY
    //let uart = hal::uart::Uart::new_blocking(r.uart.uart0, r.uart.rx, r.uart.tx, Default::default()).unwrap();
    // unsafe { UART = Some(uart) };

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

    spawner.spawn(oled_task(r.pwm3610)).unwrap();

    loop {
        Timer::after_millis(1000).await;

        defmt::info!("tick {}", MCHTMR.mtime().read());

        led.toggle();
    }
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    println!("\n\n\nPANIC:\n{}", defmt::Debug2Format(info));

    loop {}
}
