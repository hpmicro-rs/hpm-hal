#![no_main]
#![no_std]

use defmt_rtt as _;
use embassy_time::{Delay, Duration};
use embedded_hal::delay::DelayNs;
use hpm_hal as hal;
use hpm_hal::i2c::{I2c, I2cMode};
use panic_halt as _;

const ADDRESSES: [u8; 2] = [0x23, 0x5c];

const POWER_ON: u8 = 0x01;
const RESET: u8 = 0x07;
const CONTINUOUS_HIGH_RESOLUTION: u8 = 0x10;

#[unsafe(no_mangle)]
static mut I2C_SENSOR_BH1750_STAGE: u32 = 0;

#[hal::entry]
fn main() -> ! {
    mark_stage(1);
    let p = hal::init(Default::default());
    mark_stage(2);

    defmt::info!("i2c bh1750 start");
    defmt::info!("bus: I2C0, scl: PC08, sda: PC09");
    defmt::info!("cpu0: {}Hz", hal::sysctl::clocks().cpu0.0);

    let mut config = hal::i2c::Config::default();
    config.mode = I2cMode::Standard;
    config.timeout = Duration::from_millis(100);

    let mut i2c = I2c::new_blocking(p.I2C0, p.PC08, p.PC09, config);
    mark_stage(3);

    let address = loop {
        let mut found = None;
        for address in ADDRESSES {
            if i2c.blocking_write(address, &[POWER_ON]).is_ok() {
                found = Some(address);
                break;
            }
        }

        if let Some(address) = found {
            break address;
        }

        defmt::error!("bh1750 not found at 0x23 or 0x5c");
        mark_stage(0xe001);
        Delay.delay_ms(1000);
    };

    defmt::info!("found bh1750 at 0x{:02x}", address);
    mark_stage(4);

    if i2c.blocking_write(address, &[RESET]).is_err() {
        defmt::error!("bh1750 reset failed");
        mark_stage(0xe002);
    }

    if i2c.blocking_write(address, &[CONTINUOUS_HIGH_RESOLUTION]).is_err() {
        defmt::error!("bh1750 mode setup failed");
        mark_stage(0xe003);
        loop {
            riscv::asm::nop();
        }
    }

    defmt::info!("continuous high-resolution mode enabled");
    mark_stage(5);
    Delay.delay_ms(180);

    let mut sample = 0u32;
    loop {
        let mut data = [0u8; 2];
        match i2c.blocking_read(address, &mut data) {
            Ok(()) => {
                sample = sample.wrapping_add(1);
                let raw = u16::from_be_bytes(data);
                // Datasheet conversion at the default MTreg=69: lux = raw / 1.2.
                let lux_x100 = raw as u32 * 250 / 3;

                mark_stage(0x100 | (sample & 0xff));
                defmt::info!("sample {}", sample);
                defmt::info!("raw: {}", raw);
                defmt::info!("illuminance: {}.{} lux", lux_x100 / 100, lux_x100 % 100);
            }
            Err(_) => {
                defmt::error!("bh1750 read failed");
                mark_stage(0xe004);
            }
        }

        Delay.delay_ms(1000);
    }
}

#[inline(never)]
fn mark_stage(stage: u32) {
    unsafe {
        core::ptr::write_volatile(core::ptr::addr_of_mut!(I2C_SENSOR_BH1750_STAGE), stage);
    }
}
