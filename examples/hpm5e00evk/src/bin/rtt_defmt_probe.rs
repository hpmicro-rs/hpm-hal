#![no_main]
#![no_std]

use defmt_rtt as _;
use hpm_hal as hal;
use panic_halt as _;

#[hal::entry]
fn main() -> ! {
    let _p = hal::init(Default::default());

    defmt::info!("rtt defmt probe start");
    defmt::info!("CPU0 clock: {}Hz", hal::sysctl::clocks().cpu0.0);

    for index in 0..16u32 {
        defmt::info!("rtt defmt probe sample {}", index);
        spin(1_000_000);
    }

    defmt::info!("rtt defmt probe done");

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
