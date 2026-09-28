#![no_main]
#![no_std]

use core::sync::atomic::{AtomicU32, Ordering};

use defmt_rtt as _;
use hpm_hal as hal;
use hpm_hal::adc::AdcChannel;
use panic_halt as _;

const SAMPLE_COUNT: u32 = 256;
const GROUND_MAX_COUNTS: u16 = 4096;

#[unsafe(no_mangle)]
pub static HPM5E_ADC_VALIDATION_STATUS: AtomicU32 = AtomicU32::new(0);

#[hal::entry]
fn main() -> ! {
    let p = hal::init(Default::default());
    let mut channel = p.PF27.degrade_adc();
    let mut config = hal::adc::Config::default();
    config.clock_divider = hal::adc::ClockDivider::DIV10;
    let mut adc = hal::adc::Adc::new(p.ADC0, config);

    defmt::info!("HPM5E ADC0 ground validation start: PF27/ADC0_IN0 -> GND");

    let mut minimum = u16::MAX;
    let mut maximum = u16::MIN;
    let mut sum = 0u32;
    for _ in 0..SAMPLE_COUNT {
        let sample = adc.blocking_read(&mut channel, Default::default());
        minimum = minimum.min(sample);
        maximum = maximum.max(sample);
        sum += sample as u32;
    }

    let average = sum / SAMPLE_COUNT;
    let average_mv = average * 3300 / 65535;
    defmt::info!(
        "ADC0_IN0 samples={} min={} max={} avg={} avg_mv={}",
        SAMPLE_COUNT,
        minimum,
        maximum,
        average,
        average_mv
    );

    if maximum <= GROUND_MAX_COUNTS {
        HPM5E_ADC_VALIDATION_STATUS.store(average | 0x8000_0000, Ordering::Release);
        defmt::info!("HPM5E ADC0 ground validation PASS");
    } else {
        HPM5E_ADC_VALIDATION_STATUS.store(0xe300_0000 | maximum as u32, Ordering::Release);
        defmt::error!(
            "HPM5E ADC0 ground validation FAIL: max={} limit={}",
            maximum,
            GROUND_MAX_COUNTS
        );
    }

    loop {
        core::hint::spin_loop();
    }
}
