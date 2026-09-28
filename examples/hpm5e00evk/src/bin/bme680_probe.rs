#![no_main]
#![no_std]

use defmt_rtt as _;
use embassy_time::{Delay, Duration};
use embedded_hal::delay::DelayNs;
use hpm_hal as hal;
use hpm_hal::i2c::{I2c, I2cMode};
use hpm_hal::mode::Blocking;
use panic_halt as _;

const ADDR: u8 = 0x77;

const REG_CHIP_ID: u8 = 0xd0;
const REG_RESET: u8 = 0xe0;
const REG_CTRL_GAS_1: u8 = 0x71;
const REG_CTRL_HUM: u8 = 0x72;
const REG_CTRL_MEAS: u8 = 0x74;
const REG_GAS_WAIT_0: u8 = 0x64;
const REG_RES_HEAT_0: u8 = 0x5a;
const REG_FIELD0: u8 = 0x1d;

type I2c0<'d> = I2c<'d, Blocking>;

#[unsafe(no_mangle)]
static mut BME680_STAGE: u32 = 0;

#[inline(never)]
fn mark_stage(stage: u32) {
    unsafe {
        core::ptr::write_volatile(core::ptr::addr_of_mut!(BME680_STAGE), stage);
    }
}

fn write_reg(i2c: &mut I2c0<'_>, reg: u8, value: u8) -> Result<(), hal::i2c::Error> {
    i2c.blocking_write(ADDR, &[reg, value])
}

fn read_reg(i2c: &mut I2c0<'_>, reg: u8) -> Result<u8, hal::i2c::Error> {
    let mut value = [0u8; 1];
    i2c.blocking_write_read(ADDR, &[reg], &mut value)?;
    Ok(value[0])
}

fn read_regs(i2c: &mut I2c0<'_>, reg: u8, buf: &mut [u8]) -> Result<(), hal::i2c::Error> {
    i2c.blocking_write_read(ADDR, &[reg], buf)
}

#[hal::entry]
fn main() -> ! {
    mark_stage(1);
    let p = hal::init(Default::default());
    mark_stage(2);

    defmt::info!("bme680 raw probe start");
    defmt::info!("bus: I2C0, scl: PC08, sda: PC09, addr: 0x77");
    defmt::info!("cpu0: {}Hz", hal::sysctl::clocks().cpu0.0);

    let mut i2c_config = hal::i2c::Config::default();
    i2c_config.mode = I2cMode::Standard;
    i2c_config.timeout = Duration::from_millis(100);

    let mut i2c = I2c::new_blocking(p.I2C0, p.PC08, p.PC09, i2c_config);
    mark_stage(3);

    mark_stage(4);
    match read_reg(&mut i2c, REG_CHIP_ID) {
        Ok(id) => defmt::info!("chip id: 0x{:02x}", id),
        Err(_) => defmt::error!("chip id read failed"),
    }

    mark_stage(5);
    let _ = write_reg(&mut i2c, REG_RESET, 0xb6);
    Delay.delay_ms(10);

    mark_stage(6);
    match read_reg(&mut i2c, REG_CHIP_ID) {
        Ok(id) => defmt::info!("chip id after reset: 0x{:02x}", id),
        Err(_) => defmt::error!("chip id read after reset failed"),
    }

    // Minimal forced-mode setup:
    // humidity x1, temperature x1, pressure x1, gas heater profile 0.
    let _ = write_reg(&mut i2c, REG_CTRL_HUM, 0x01);
    let _ = write_reg(&mut i2c, REG_RES_HEAT_0, 0x60);
    let _ = write_reg(&mut i2c, REG_GAS_WAIT_0, 0x59);
    let _ = write_reg(&mut i2c, REG_CTRL_GAS_1, 0x10);

    loop {
        mark_stage(7);
        let _ = write_reg(&mut i2c, REG_CTRL_HUM, 0x01);
        let _ = write_reg(&mut i2c, REG_CTRL_MEAS, 0x25);
        Delay.delay_ms(250);

        mark_stage(8);
        let mut frame = [0u8; 15];
        match read_regs(&mut i2c, REG_FIELD0, &mut frame) {
            Ok(()) => {
                defmt::info!(
                    "field: {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x}",
                    frame[0],
                    frame[1],
                    frame[2],
                    frame[3],
                    frame[4],
                    frame[5],
                    frame[6],
                    frame[7],
                    frame[8],
                    frame[9],
                    frame[10],
                    frame[11],
                    frame[12],
                    frame[13],
                    frame[14]
                );

                let status = frame[0];
                let pressure_raw = ((frame[2] as u32) << 12) | ((frame[3] as u32) << 4) | ((frame[4] as u32) >> 4);
                let temperature_raw = ((frame[5] as u32) << 12) | ((frame[6] as u32) << 4) | ((frame[7] as u32) >> 4);
                let humidity_raw = ((frame[8] as u16) << 8) | frame[9] as u16;
                let gas_raw = ((frame[13] as u16) << 2) | ((frame[14] as u16) >> 6);
                let gas_range = frame[14] & 0x0f;

                defmt::info!("status: 0x{:02x}", status);
                defmt::info!("temperature raw: {}", temperature_raw);
                defmt::info!("pressure raw: {}", pressure_raw);
                defmt::info!("humidity raw: {}", humidity_raw);
                defmt::info!("gas raw: {}, range: {}", gas_raw, gas_range);
            }
            Err(_) => {
                defmt::error!("field read failed");
            }
        }

        Delay.delay_ms(1000);
    }
}
