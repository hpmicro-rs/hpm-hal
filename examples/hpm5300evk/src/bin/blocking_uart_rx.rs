#![no_main]
#![no_std]

use embedded_hal::delay::DelayNs;
use embedded_io::Write as _; // `writeln!` provider
use hpm_hal::gpio::{Level, Output, Speed};
use hpm_hal::uart::UartRx;
use riscv::delay::McycleDelay;
use {defmt_rtt as _, hpm_hal as hal};

const BANNER: &str = include_str!("../../../assets/BANNER");

#[hal::entry]
fn main() -> ! {
    let p = hal::init(Default::default());

    let mut delay = McycleDelay::new(hal::sysctl::clocks().cpu0.0);

    defmt::info!("Board init!");
    defmt::info!("\n{}", BANNER);
    defmt::info!(
        "Clock summary:\nCPU0:\t{}Hz\nAHB:\t{}Hz",
        hal::sysctl::clocks().cpu0.0,
        hal::sysctl::clocks().ahb.0
    );

    let mut led = Output::new(p.PA23, Level::Low, Speed::default());

    let mut uart_config = hal::uart::Config::default();
    uart_config.baudrate = 9600;

    //let mut rx = UartRx::new_blocking(p.UART0, p.PA01, uart_config).unwrap();

    let mut rx = UartRx::new_blocking(p.UART2, p.PB09, uart_config).unwrap();

    loop {
        defmt::info!("Waiting for data...");

        let mut buf = [0u8; 32];

        rx.blocking_read(&mut buf).unwrap();

        let measurement = Measurement::new(&buf);

        if let Some(measurement) = measurement {
            defmt::info!("PM1.0 {} ug/m^3", measurement.pm1_0_atm());
            defmt::info!("PM2.5 {} ug/m^3", measurement.pm2_5_atm());
            defmt::info!("PM10  {} ug/m^3", measurement.pm10_atm());
        } else {
            defmt::info!("Invalid measurement data");
        }
    }
}

#[derive(Debug)]
pub struct Measurement<'a>(&'a [u8; 32]);

impl Measurement<'_> {
    pub fn new<'a>(data: &'a [u8; 32]) -> Option<Measurement<'a>> {
        if data[0] == 0x42 && data[1] == 0x4d {
            let this = Measurement(data);
            if this.checksum() {
                return Some(this);
            }
        }
        None
    }

    #[inline]
    fn checksum(&self) -> bool {
        let checksum: u16 = self.0.iter().take(30).map(|&n| n as u16).sum();
        if checksum == u16::from_be_bytes([self.0[30], self.0[31]]) {
            true
        } else {
            false
        }
    }

    pub fn pm1_0(&self) -> u16 {
        u16::from_be_bytes([self.0[4], self.0[5]])
    }

    pub fn pm2_5(&self) -> u16 {
        u16::from_be_bytes([self.0[6], self.0[7]])
    }

    pub fn pm10(&self) -> u16 {
        u16::from_be_bytes([self.0[8], self.0[9]])
    }

    pub fn pm1_0_atm(&self) -> u16 {
        u16::from_be_bytes([self.0[10], self.0[11]])
    }

    pub fn pm2_5_atm(&self) -> u16 {
        u16::from_be_bytes([self.0[12], self.0[13]])
    }

    pub fn pm10_atm(&self) -> u16 {
        u16::from_be_bytes([self.0[14], self.0[15]])
    }

    pub fn gt0_3um(&self) -> u16 {
        u16::from_be_bytes([self.0[16], self.0[17]])
    }

    pub fn gt0_5um(&self) -> u16 {
        u16::from_be_bytes([self.0[18], self.0[19]])
    }

    pub fn gt1_0um(&self) -> u16 {
        u16::from_be_bytes([self.0[20], self.0[21]])
    }

    pub fn gt2_5um(&self) -> u16 {
        u16::from_be_bytes([self.0[22], self.0[23]])
    }

    pub fn gt5_0um(&self) -> u16 {
        u16::from_be_bytes([self.0[24], self.0[25]])
    }

    pub fn gt10um(&self) -> u16 {
        u16::from_be_bytes([self.0[26], self.0[27]])
    }
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    defmt::info!("panic: {:?}", defmt::Debug2Format(&info));
    loop {}
}
