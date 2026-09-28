#![no_main]
#![no_std]
#![feature(impl_trait_in_assoc_type)]

use core::cell::RefCell;

use bosch_bme680::{Bme680, BmeError, Configuration, DeviceAddress};
use defmt_rtt as _;
use embassy_embedded_hal::shared_bus::blocking::i2c::I2cDevice;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::{Delay, Duration, Timer};
use embedded_hal::i2c::I2c as _;
use hpm_hal as hal;
use hpm_hal::i2c::{I2c, I2cMode};
use panic_halt as _;

const BH1750_ADDR: u8 = 0x23;
const APDS9930_ADDR: u8 = 0x39;

const BH1750_POWER_ON: u8 = 0x01;
const BH1750_RESET: u8 = 0x07;
const BH1750_CONTINUOUS_HIGH_RESOLUTION: u8 = 0x10;

const APDS9930_CMD_BYTE: u8 = 0x80;
const APDS9930_CMD_AUTO_INCREMENT: u8 = 0xa0;
const APDS9930_ENABLE: u8 = 0x00;
const APDS9930_ATIME: u8 = 0x01;
const APDS9930_PTIME: u8 = 0x02;
const APDS9930_WTIME: u8 = 0x03;
const APDS9930_PPULSE: u8 = 0x0e;
const APDS9930_CONTROL: u8 = 0x0f;
const APDS9930_ID: u8 = 0x12;
const APDS9930_STATUS: u8 = 0x13;
const APDS9930_CH0DATAL: u8 = 0x14;

#[embassy_executor::main(entry = "hpm_hal::entry")]
async fn main(_spawner: embassy_executor::Spawner) -> ! {
    let p = hal::init(Default::default());

    defmt::info!("shared I2C sensor bus start");
    defmt::info!("I2C0 PC08/PC09: BH1750=0x23 APDS-9930=0x39 BME680=0x77");
    defmt::info!("cpu0: {}Hz", hal::sysctl::clocks().cpu0.0);

    let mut i2c_config = hal::i2c::Config::default();
    i2c_config.mode = I2cMode::Standard;
    i2c_config.timeout = Duration::from_millis(100);

    let i2c = I2c::new_blocking(p.I2C0, p.PC08, p.PC09, i2c_config);
    let bus = Mutex::<CriticalSectionRawMutex, _>::new(RefCell::new(i2c));

    // Each driver owns an independent device proxy. The proxies serialize every
    // transaction through the shared Embassy mutex.
    let mut bh1750 = I2cDevice::new(&bus);
    let mut apds9930 = I2cDevice::new(&bus);
    let bme680_i2c = I2cDevice::new(&bus);

    let bh1750_ready = bh1750.write(BH1750_ADDR, &[BH1750_POWER_ON]).is_ok()
        && bh1750.write(BH1750_ADDR, &[BH1750_RESET]).is_ok()
        && bh1750.write(BH1750_ADDR, &[BH1750_CONTINUOUS_HIGH_RESOLUTION]).is_ok();
    if bh1750_ready {
        defmt::info!("BH1750 init ok");
    } else {
        defmt::error!("BH1750 init failed");
    }

    let apds9930_ready = init_apds9930(&mut apds9930);
    if apds9930_ready {
        let mut id = [0u8; 1];
        match apds9930.write_read(APDS9930_ADDR, &[APDS9930_CMD_BYTE | APDS9930_ID], &mut id) {
            Ok(()) => defmt::info!("APDS-9930 init ok, id=0x{:02x}", id[0]),
            Err(_) => defmt::error!("APDS-9930 ID read failed"),
        }
    } else {
        defmt::error!("APDS-9930 init failed");
    }

    let bme_config = Configuration::default();
    let mut bme680 = match Bme680::new(bme680_i2c, DeviceAddress::Secondary, Delay, &bme_config, 20) {
        Ok(sensor) => {
            defmt::info!("BME680 init ok");
            Some(sensor)
        }
        Err(error) => {
            log_bme_error("BME680 init failed", error);
            None
        }
    };

    // BH1750 needs at least 120 ms for the first high-resolution conversion;
    // APDS-9930 uses a 101 ms ALS integration period.
    Timer::after_millis(180).await;

    let mut cycle = 0u32;
    loop {
        cycle = cycle.wrapping_add(1);
        defmt::info!("sensor cycle {}", cycle);

        if bh1750_ready {
            read_bh1750(&mut bh1750);
        }

        if apds9930_ready {
            read_apds9930(&mut apds9930);
        }

        if let Some(sensor) = bme680.as_mut() {
            match sensor.measure() {
                Ok(data) => {
                    defmt::info!(
                        "BME680 temp={}C humidity={}% pressure={}Pa",
                        data.temperature,
                        data.humidity,
                        data.pressure
                    );
                    match data.gas_resistance {
                        Some(gas) => defmt::info!("BME680 gas={}ohm", gas),
                        None => defmt::warn!("BME680 gas unavailable"),
                    }
                }
                Err(error) => log_bme_error("BME680 measure failed", error),
            }
        }

        Timer::after_secs(1).await;
    }
}

fn init_apds9930<I: embedded_hal::i2c::I2c>(i2c: &mut I) -> bool {
    // Datasheet basic setup: 101 ms ALS integration, minimum proximity/wait
    // periods, eight 100 mA proximity pulses, CH1 diode, and 1x gains.
    let setup = [
        (APDS9930_ENABLE, 0x00),
        (APDS9930_ATIME, 0xdb),
        (APDS9930_PTIME, 0xff),
        (APDS9930_WTIME, 0xff),
        (APDS9930_PPULSE, 8),
        (APDS9930_CONTROL, 0x20),
        (APDS9930_ENABLE, 0x0f),
    ];

    for (register, value) in setup {
        if i2c
            .write(APDS9930_ADDR, &[APDS9930_CMD_BYTE | register, value])
            .is_err()
        {
            return false;
        }
    }
    true
}

fn read_bh1750<I: embedded_hal::i2c::I2c>(i2c: &mut I) {
    let mut data = [0u8; 2];
    match i2c.read(BH1750_ADDR, &mut data) {
        Ok(()) => {
            let raw = u16::from_be_bytes(data);
            let lux_x100 = raw as u32 * 250 / 3;
            defmt::info!(
                "BH1750 raw={} illuminance={}.{}lux",
                raw,
                lux_x100 / 100,
                lux_x100 % 100
            );
        }
        Err(_) => defmt::error!("BH1750 read failed"),
    }
}

fn read_apds9930<I: embedded_hal::i2c::I2c>(i2c: &mut I) {
    let mut status = [0u8; 1];
    if i2c
        .write_read(APDS9930_ADDR, &[APDS9930_CMD_BYTE | APDS9930_STATUS], &mut status)
        .is_err()
    {
        defmt::error!("APDS-9930 status read failed");
        return;
    }

    let mut data = [0u8; 6];
    match i2c.write_read(
        APDS9930_ADDR,
        &[APDS9930_CMD_AUTO_INCREMENT | APDS9930_CH0DATAL],
        &mut data,
    ) {
        Ok(()) => {
            let clear = u16::from_le_bytes([data[0], data[1]]);
            let infrared = u16::from_le_bytes([data[2], data[3]]);
            let proximity = u16::from_le_bytes([data[4], data[5]]);
            defmt::info!(
                "APDS-9930 status=0x{:02x} clear={} ir={} proximity={}",
                status[0],
                clear,
                infrared,
                proximity
            );
        }
        Err(_) => defmt::error!("APDS-9930 data read failed"),
    }
}

fn log_bme_error<I>(message: &str, error: BmeError<I>)
where
    I: embedded_hal::i2c::ErrorType,
{
    match error {
        BmeError::WriteError(_) => defmt::error!("{}: write error", message),
        BmeError::WriteReadError(_) => defmt::error!("{}: write-read error", message),
        BmeError::UnexpectedChipId(id) => {
            defmt::error!("{}: unexpected chip id 0x{:02x}", message, id)
        }
        BmeError::MeasuringTimeOut => defmt::error!("{}: measurement timeout", message),
    }
}
