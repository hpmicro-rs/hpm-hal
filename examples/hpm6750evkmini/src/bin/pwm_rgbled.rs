//! RGB LED using PWM
//!
//! RGB LED pins (active low):
//! - R: PB19 -> PWM1 CH0
//! - G: PB18 -> PWM1 CH1
//! - B: PB20 -> PWM0 CH7

#![no_main]
#![no_std]

use defmt::info;
use embassy_time::Delay;
use embedded_hal::delay::DelayNs;
use hal::pwm::{Polarity, SimplePwm, SimplePwmConfig};
use hal::time::Hertz;
use hpm_hal as hal;
use {defmt_rtt as _};

const BOARD_NAME: &str = "HPM6750EVKMINI";

#[hal::entry]
fn main() -> ! {
    let p = hal::init(Default::default());
    let mut delay = Delay;

    info!("{} RGB LED PWM example", BOARD_NAME);
    info!("Clock: CPU0={}Hz", hal::sysctl::clocks().cpu0.0);

    let config = SimplePwmConfig {
        frequency: Hertz(1_000), // 1kHz
        polarity: Polarity::ActiveLow,
        ..Default::default()
    };

    // PWM1: Red (CH0) and Green (CH1)
    let mut pwm1 = SimplePwm::new(p.PWM1, config.clone());
    pwm1.enable_ch0(p.PB19); // Red
    pwm1.enable_ch1(p.PB18); // Green

    // PWM0: Blue (CH7)
    let mut pwm0 = SimplePwm::new(p.PWM0, config);
    pwm0.enable_ch7(p.PB20); // Blue

    let max_duty = pwm1.max_duty();
    info!("PWM initialized, max_duty={}, starting color cycle...", max_duty);

    loop {
        // HSL color wheel
        for h in 0..360 {
            let [r, g, b] = hsl_to_rgb(h as f32 / 360.0, 0.9, 0.5);

            // Convert 0-255 to duty cycle
            let duty_r = r as u32 * max_duty / 255;
            let duty_g = g as u32 * max_duty / 255;
            let duty_b = b as u32 * max_duty / 255;

            pwm1.set_duty_ch0(duty_r);
            pwm1.set_duty_ch1(duty_g);
            pwm0.set_duty_ch7(duty_b);

            delay.delay_ms(10);
        }
    }
}

// Simple HSL to RGB conversion
fn hsl_to_rgb(h: f32, s: f32, l: f32) -> [u8; 3] {
    if s == 0.0 {
        let v = (l * 255.0) as u8;
        return [v, v, v];
    }

    let temp1 = if l < 0.5 { l * (1.0 + s) } else { l + s - l * s };
    let temp2 = 2.0 * l - temp1;

    let r = hue_to_rgb(temp2, temp1, h + 1.0 / 3.0);
    let g = hue_to_rgb(temp2, temp1, h);
    let b = hue_to_rgb(temp2, temp1, h - 1.0 / 3.0);

    [(r * 255.0) as u8, (g * 255.0) as u8, (b * 255.0) as u8]
}

fn hue_to_rgb(p: f32, q: f32, mut t: f32) -> f32 {
    if t < 0.0 {
        t += 1.0;
    }
    if t > 1.0 {
        t -= 1.0;
    }

    if t < 1.0 / 6.0 {
        p + (q - p) * 6.0 * t
    } else if t < 0.5 {
        q
    } else if t < 2.0 / 3.0 {
        p + (q - p) * (2.0 / 3.0 - t) * 6.0
    } else {
        p
    }
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    defmt::error!("panic: {}", defmt::Display2Format(info));
    loop {}
}
