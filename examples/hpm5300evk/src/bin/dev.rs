#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]
#![feature(impl_trait_in_assoc_type)]

use core::arch::asm;

use assign_resources::assign_resources;
use defmt::println;
use embassy_time::{Delay, Duration, Timer};
use embedded_hal::delay::DelayNs;
use embedded_io::Write as _; // `writeln!` provider
use hal::gpio::{Level, Output, Speed};
use hal::i2c::I2c;
use hal::mode::Blocking;
use hal::peripherals;
use hpm_hal::gpio::Pin;
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
    i2c: I2c2 {
        sda: PB06,
        scl: PB07,
        periph: I2C1,
    }
}

static mut UART: Option<hal::uart::Uart<'static, Blocking>> = None;

macro_rules! println {
    ($($arg:tt)*) => {
        {
            if let Some(uart) = unsafe { UART.as_mut() } {
                writeln!(uart, $($arg)*).unwrap();
            }
        }
    }
}

// - MARK: DS3231M Driver
pub const ADDRESS: u8 = 0x68;

mod regs {
    pub const SECONDS: u8 = 0x00;
    pub const MINUTES: u8 = 0x01;
    pub const HOURS: u8 = 0x02;
    pub const DAY: u8 = 0x03;
    pub const DATE: u8 = 0x04;
    pub const MONTH: u8 = 0x05;
    pub const YEAR: u8 = 0x06;

    pub const ALARM1_SECONDS: u8 = 0x07;
    pub const ALARM1_MINUTES: u8 = 0x08;
    pub const ALARM1_HOURS: u8 = 0x09;
    pub const ALARM1_DAY_DATE: u8 = 0x0A;

    pub const ALARM2_MINUTES: u8 = 0x0B;
    pub const ALARM2_HOURS: u8 = 0x0C;
    pub const ALARM2_DAY_DATE: u8 = 0x0D;

    pub const CONTROL: u8 = 0x0E;
    pub const STATUS: u8 = 0x0F;
    pub const AGING_OFFSET: u8 = 0x10;
    pub const TEMP_MSB: u8 = 0x11;
    pub const TEMP_LSB: u8 = 0x12;
}

/// Structure containing date and time information
#[derive(Clone, Debug)]
pub struct DateTime {
    /// 0..4095
    pub year: u16,
    /// 1..12, 1 is January
    pub month: u8,
    /// 1..28,29,30,31 depending on month
    pub day: u8,
    ///
    pub day_of_week: DayOfWeek,
    /// 0..23
    pub hour: u8,
    /// 0..59
    pub minute: u8,
    /// 0..59
    pub second: u8,
}

/// A day of the week
#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq, Eq, Ord, PartialOrd, Hash)]
#[allow(missing_docs)]
pub enum DayOfWeek {
    Sunday = 7,
    Monday = 1,
    Tuesday = 2,
    Wednesday = 3,
    Thursday = 4,
    Friday = 5,
    Saturday = 6,
}

impl DayOfWeek {
    /// Convert a u8 to a DayOfWeek
    pub fn from_u8(d: u8) -> Option<Self> {
        match d {
            1 => Some(DayOfWeek::Monday),
            2 => Some(DayOfWeek::Tuesday),
            3 => Some(DayOfWeek::Wednesday),
            4 => Some(DayOfWeek::Thursday),
            5 => Some(DayOfWeek::Friday),
            6 => Some(DayOfWeek::Saturday),
            7 => Some(DayOfWeek::Sunday),
            _ => None,
        }
    }
}

pub struct DS3231M<'a> {
    i2c: I2c<'a, Blocking>,
}

impl<'d> DS3231M<'d> {
    pub fn new(i2c: I2c<'d, Blocking>) -> Self {
        Self { i2c }
    }

    pub fn now(&mut self) -> Option<DateTime> {
        let mut buf = [0u8; 7];
        self.i2c.blocking_write_read(ADDRESS, &[regs::SECONDS], &mut buf).ok()?;

        let seconds = bcd2bin(buf[0]);
        let minutes = bcd2bin(buf[1]);
        let hours = bcd2bin(buf[2]);
        let day = bcd2bin(buf[3]);
        let date = bcd2bin(buf[4]);
        let month = bcd2bin(buf[5]);
        let year = bcd2bin(buf[6]) as u16 + 2000;

        Some(DateTime {
            year: year as u16,
            month,
            day: date,
            day_of_week: DayOfWeek::from_u8(day).unwrap(),
            hour: hours,
            minute: minutes,
            second: seconds,
        })
    }

    pub fn set_datetime(&mut self, dt: &DateTime) {
        let mut buf = [0u8; 8];
        buf[0] = regs::SECONDS; // addr
        buf[1] = bin2bcd(dt.second);
        buf[2] = bin2bcd(dt.minute);
        buf[3] = bin2bcd(dt.hour);
        buf[4] = bin2bcd(dt.day_of_week as u8);
        buf[5] = bin2bcd(dt.day);
        buf[6] = bin2bcd(dt.month);
        buf[7] = bin2bcd((dt.year - 2000) as u8);

        self.i2c.blocking_write(ADDRESS, &buf).ok();
    }
}

fn bcd2bin(bcd: u8) -> u8 {
    (bcd & 0x0F) + ((bcd >> 4) * 10)
}

fn bin2bcd(bin: u8) -> u8 {
    ((bin / 10) << 4) + (bin % 10)
}

// - MARK: Main

#[embassy_executor::main(entry = "hpm_hal::entry")]
async fn main(spawner: embassy_executor::Spawner) -> ! {
    let config = hal::Config::default();
    let p = hal::init(config);

    let r = split_resources!(p);

    let x: u32 = 0xf1f2f3f4;
    let y: u32 = 0xf5f6f7f8;

    let c = x.overflowing_add(y).0;

    // .insn r opcode, func3, func7, rd, rs1, rs2
    let mut tmp: u32 = 0;

    unsafe {
        asm!(
            r#"
                .insn r 0x7f, 0, 0b0100100, {out}, {in1}, {in2}
            "#,
            in1 = in(reg) x,
            in2 = in(reg) y,
            out = out(reg) tmp,
        )
    };

    defmt::println!("x + y = 0x{:08x}", tmp);
    defmt::println!("x + y = 0x{:08x}", c);

    // use IOC for power domain PY pins
    r.uart.tx.set_as_ioc_gpio();
    r.uart.rx.set_as_ioc_gpio();

    let mut led = Output::new(r.leds.red, Level::Low, Default::default());

    let uart = hal::uart::Uart::new_blocking(r.uart.uart0, r.uart.rx, r.uart.tx, Default::default()).unwrap();
    unsafe { UART = Some(uart) };

    println!("{}", BANNER);
    println!("Board: {}", BOARD_NAME);
    println!("Board init!");

    extern "C" {
        static __noncacheable_data_start__: [u32; 0];
        static __noncacheable_data_end__: [u32; 0];
    }

    println!("__noncacheable_data_start__: {:p}", unsafe {
        __noncacheable_data_start__.as_ptr()
    });

    println!("__noncacheable_data_end__: {:p}", unsafe {
        __noncacheable_data_end__.as_ptr()
    });

    let mut i2c_config = hal::i2c::Config::default();
    {
        use hal::i2c::*;
        i2c_config.mode = I2cMode::Fast;
        i2c_config.timeout = Duration::from_secs(1);
    }
    let i2c = I2c::new_blocking(r.i2c.periph, r.i2c.scl, r.i2c.sda, i2c_config);

    //let mut sensor = edrv_vl53l0x::VL53L0X::new(i2c);

    //sensor.init().unwrap();

    loop {
        led.toggle();

        Timer::after(Duration::from_millis(1000)).await;
    }

    let mut sht20 = edrv_sht20::blocking::SHT20::new_primary(i2c);

    defmt::info!("measuring...");

    loop {
        let temp = sht20.read_temperature(Delay).unwrap();
        let hum = sht20.read_humidity(Delay).unwrap();

        defmt::info!("Temp: {} DegC, Humidity: {} %RH", temp, hum);

        Timer::after(Duration::from_millis(1000)).await;
    }

    defmt::info!("init ok");

    let mut s = edrv_bme280::blocking::BME280::new_primary(i2c);

    s.init().unwrap();

    loop {
        led.toggle();

        let m = s.read_measurement().unwrap();

        defmt::info!(
            "tick= > Temp: {} DegC, Pressure: {} hPa, Humm: {} %RH",
            m.temperature_celsius(),
            m.pressure_hpa(),
            m.humidity_percent()
        );

        Timer::after(Duration::from_millis(1000)).await;
    }
}

#[panic_handler]
unsafe fn panic(info: &core::panic::PanicInfo) -> ! {
    println!("panic!\n {}", info);

    loop {}
}
