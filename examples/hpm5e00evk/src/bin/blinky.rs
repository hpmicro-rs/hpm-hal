#![no_main]
#![no_std]

use defmt_rtt as _;
use embedded_hal::delay::DelayNs;
use hpm_hal as hal;
use hpm_hal::gpio::{Level, Output};
use panic_halt as _;
use riscv::delay::McycleDelay;

#[hal::entry]
fn main() -> ! {
    let p = hal::init(Default::default());
    let mut delay = McycleDelay::new(hal::sysctl::clocks().cpu0.0);

    let mut led0 = Output::new(p.PD06, Level::Low, Default::default());
    let mut led1 = Output::new(p.PD07, Level::High, Default::default());

    defmt::info!("HPM5E00EVK PD06/PD07 blinky start");
    defmt::info!("CPU0 clock: {}Hz", hal::sysctl::clocks().cpu0.0);

    loop {
        led0.toggle();
        led1.toggle();
        defmt::info!("tick");
        delay.delay_ms(500);
    }
}
