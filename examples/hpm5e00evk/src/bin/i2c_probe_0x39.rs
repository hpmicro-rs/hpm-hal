#![no_main]
#![no_std]

use defmt_rtt as _;
use embassy_time::Duration;
use hpm_hal as hal;
use hpm_hal::i2c::{I2c, I2cMode};
use panic_halt as _;

const ADDRESS: u8 = 0x39;
const REGISTERS: [u8; 10] = [0x80, 0x81, 0x83, 0x8a, 0x8d, 0x8f, 0x90, 0x91, 0x92, 0xa2];

#[hal::entry]
fn main() -> ! {
    let p = hal::init(Default::default());

    defmt::info!("i2c 0x39 identification probe start");
    defmt::info!("bus: I2C0, scl: PC08, sda: PC09, addr: 0x39");

    let mut config = hal::i2c::Config::default();
    config.mode = I2cMode::Standard;
    config.timeout = Duration::from_millis(100);

    let mut i2c = I2c::new_blocking(p.I2C0, p.PC08, p.PC09, config);

    for register in REGISTERS {
        let mut value = [0u8; 1];
        match i2c.blocking_write_read(ADDRESS, &[register], &mut value) {
            Ok(()) => defmt::info!("reg 0x{:02x} = 0x{:02x}", register, value[0]),
            Err(_) => defmt::error!("reg 0x{:02x} read failed", register),
        }
    }

    defmt::info!("i2c 0x39 identification probe done");

    loop {
        riscv::asm::wfi();
    }
}
