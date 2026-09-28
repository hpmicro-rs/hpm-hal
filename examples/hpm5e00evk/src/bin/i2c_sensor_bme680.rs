#![no_main]
#![no_std]

use bosch_bme680::{Bme680, BmeError, Configuration, DeviceAddress};
use defmt_rtt as _;
use embassy_time::{Delay, Duration};
use embedded_hal::delay::DelayNs;
use hpm_hal as hal;
use hpm_hal::i2c::{I2c, I2cMode};
use panic_halt as _;

const ADDR: u8 = 0x77;
const REG_CHIP_ID: u8 = 0xd0;

#[unsafe(no_mangle)]
static mut I2C_SENSOR_BME680_STAGE: u32 = 0;

#[inline(never)]
fn mark_stage(stage: u32) {
    unsafe {
        core::ptr::write_volatile(core::ptr::addr_of_mut!(I2C_SENSOR_BME680_STAGE), stage);
    }
}

#[hal::entry]
fn main() -> ! {
    mark_stage(1);
    let p = hal::init(Default::default());
    mark_stage(2);

    defmt::info!("i2c bme680 start");
    defmt::info!("bus: I2C0, scl: PC08, sda: PC09, addr: 0x77");
    defmt::info!("cpu0: {}Hz", hal::sysctl::clocks().cpu0.0);

    let mut i2c_config = hal::i2c::Config::default();
    i2c_config.mode = I2cMode::Standard;
    i2c_config.timeout = Duration::from_millis(100);

    let mut i2c = I2c::new_blocking(p.I2C0, p.PC08, p.PC09, i2c_config);
    mark_stage(3);

    let mut chip_id = [0u8; 1];
    match i2c.blocking_write_read(ADDR, &[REG_CHIP_ID], &mut chip_id) {
        Ok(()) => defmt::info!("chip id: 0x{:02x}", chip_id[0]),
        Err(_) => {
            defmt::error!("chip id read failed");
            loop {
                riscv::asm::nop();
            }
        }
    }
    mark_stage(4);

    let config = Configuration::default();
    let mut sensor = match Bme680::new(i2c, DeviceAddress::Secondary, Delay, &config, 20) {
        Ok(sensor) => {
            defmt::info!("bme680 init ok");
            sensor
        }
        Err(error) => {
            log_bme_error("bme680 init failed", error);
            loop {
                riscv::asm::nop();
            }
        }
    };
    mark_stage(5);

    let mut sample = 0u32;
    loop {
        sample = sample.wrapping_add(1);
        mark_stage(0x100 | (sample & 0xff));

        match sensor.measure() {
            Ok(data) => {
                defmt::info!("sample {}", sample);
                defmt::info!("temperature: {} C", data.temperature);
                defmt::info!("humidity: {} %", data.humidity);
                defmt::info!("pressure: {} Pa", data.pressure);
                match data.gas_resistance {
                    Some(gas) => defmt::info!("gas resistance: {} ohm", gas),
                    None => defmt::warn!("gas resistance unavailable"),
                }
            }
            Err(error) => {
                log_bme_error("bme680 measure failed", error);
            }
        }

        Delay.delay_ms(1000);
    }
}

fn log_bme_error(message: &str, error: BmeError<I2c<'_, hal::mode::Blocking>>) {
    match error {
        BmeError::WriteError(_) => defmt::error!("{}: write error", message),
        BmeError::WriteReadError(_) => defmt::error!("{}: write-read error", message),
        BmeError::UnexpectedChipId(id) => defmt::error!("{}: unexpected chip id 0x{:02x}", message, id),
        BmeError::MeasuringTimeOut => defmt::error!("{}: measurement timeout", message),
    }
}
