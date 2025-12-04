//! PWM Simple Example
//!
//! Demonstrates basic PWM output using the SimplePwm driver.
//!
//! Hardware:
//! - HPM5300EVK board
//! - LED on PA23 (PWM1_P_7)
//!
//! The LED brightness will fade in and out.

#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]
#![feature(impl_trait_in_assoc_type)]

use hal::gpio::Output;
use hal::pac;
use hal::pac::{iomux, pins};
use hal::pwm::{Channel, SimplePwm, SimplePwmConfig};
use hal::time::Hertz;
use {defmt_rtt as _, hpm_hal as hal, panic_halt as _};

#[hal::entry]
fn main() -> ! {
    let mut config = hal::Config::default();
    config.sysctl.ahb_div = hal::sysctl::AHBDiv::DIV2;
    let p = hal::init(config);

    defmt::info!("PWM Simple Example");
    defmt::info!("AHB clock: {}Hz", hal::sysctl::clocks().ahb.0);

    // First, set pin to GPIO output high to turn off LED (active low)
    let _led = Output::new(p.PA23, hal::gpio::Level::High, Default::default());

    // Configure pin for PWM function
    pac::IOC
        .pad(pins::PA23)
        .func_ctl()
        .modify(|w| w.set_alt_select(iomux::IOC_PA23_FUNC_CTL_PWM1_P_7));

    // Create PWM with 1kHz frequency
    let pwm_config = SimplePwmConfig {
        frequency: Hertz(1_000),
        ..Default::default()
    };

    let mut pwm = SimplePwm::new(p.PWM1, pwm_config);

    // Enable channel 7 (connected to PA23)
    pwm.enable(Channel::Ch7);

    defmt::info!("PWM started, max_duty = {}", pwm.max_duty());

    // Fade LED in and out
    let max_duty = pwm.max_duty();
    let step = max_duty / 100;

    loop {
        // Fade in (LED gets brighter, duty decreases for active-low LED)
        for i in (0..=100).rev() {
            pwm.set_duty(Channel::Ch7, i * step);
            blocking_delay_ms(10);
        }

        // Fade out (LED gets dimmer, duty increases for active-low LED)
        for i in 0..=100 {
            pwm.set_duty(Channel::Ch7, i * step);
            blocking_delay_ms(10);
        }
    }
}

fn blocking_delay_ms(ms: u32) {
    let start = embassy_time::Instant::now();
    while start.elapsed().as_millis() < ms as u64 {}
}

