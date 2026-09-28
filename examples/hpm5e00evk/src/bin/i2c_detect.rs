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

    spin(2_000_000);

    defmt::info!("i2c detect start");
    defmt::info!("bus: I2C0, scl: PC08, sda: PC09");
    defmt::info!("cpu0: {}Hz", hal::sysctl::clocks().cpu0.0);

    let mut i2c_config = hal::i2c::Config::default();
    i2c_config.mode = I2cMode::Standard;
    i2c_config.timeout = Duration::from_millis(20);

    let mut i2c = I2c::new_blocking(p.I2C0, p.PC08, p.PC09, i2c_config);
    let mut found = 0u32;

    for addr in 0x03u8..=0x77 {
        if (addr & 0x0f) == 0 {
            defmt::info!("scan 0x{:02x}..0x{:02x}", addr, addr + 0x0f);
        }

        if i2c.blocking_write(addr, &[]).is_ok() {
            found += 1;
            defmt::info!("found 0x{:02x}", addr);
        }
    }

    defmt::info!("i2c detect done, found {} device(s)", found);

    loop {
        riscv::asm::nop();
    }
}

#[inline(never)]
fn spin(cycles: u32) {
    for _ in 0..cycles {
        riscv::asm::nop();
    }
}
