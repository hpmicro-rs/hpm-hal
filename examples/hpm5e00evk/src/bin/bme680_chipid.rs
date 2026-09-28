#![no_main]
#![no_std]

use defmt_rtt as _;
use embassy_time::Duration;
use hpm_hal as hal;
use hpm_hal::i2c::{I2c, I2cMode};
use panic_halt as _;

#[hal::entry]
fn main() -> ! {
    let p = hal::init(Default::default());

    defmt::info!("bme680 chipid start");
    defmt::info!("bus: I2C0, scl: PC08, sda: PC09, addr: 0x77");
    defmt::info!("cpu0: {}Hz", hal::sysctl::clocks().cpu0.0);

    let mut i2c_config = hal::i2c::Config::default();
    i2c_config.mode = I2cMode::Standard;
    i2c_config.timeout = Duration::from_millis(100);

    let mut i2c = I2c::new_blocking(p.I2C0, p.PC08, p.PC09, i2c_config);
    let mut id = [0u8; 1];

    match i2c.blocking_write_read(0x77, &[0xd0], &mut id) {
        Ok(()) => defmt::info!("chip id: 0x{:02x}", id[0]),
        Err(_) => defmt::error!("chip id read failed"),
    }

    loop {
        riscv::asm::wfi();
    }
}
