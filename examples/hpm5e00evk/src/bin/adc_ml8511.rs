#![no_main]
#![no_std]

use core::sync::atomic::{AtomicU32, Ordering};

use defmt_rtt as _;
use embedded_hal::delay::DelayNs;
use hpm_hal as hal;
use hpm_hal::adc::AdcChannel;
use hpm_hal::timer::{Channel, Config as TimerConfig, Timer};
use panic_halt as _;

const VALIDATION_SAMPLE_COUNT: u32 = 512;
const PERIODIC_SAMPLE_COUNT: u32 = 64;
const ADC_REFERENCE_MV: u32 = 3300;
const SIGNAL_MIN_COUNTS: u16 = 4096;
const SIGNAL_MAX_COUNTS: u16 = 64_000;
const UV_KNOWN_POINT_UV: i64 = 2_200_000;
const UV_KNOWN_POINT_UW_CM2: i64 = 10_000;
const UV_SENSITIVITY_UV_PER_MW_CM2: i64 = 129_000;

#[unsafe(no_mangle)]
pub static HPM5E_ML8511_VALIDATION_STATUS: AtomicU32 = AtomicU32::new(0);

#[hal::entry]
fn main() -> ! {
    let p = hal::init(Default::default());
    let mut channel = p.PF27.degrade_adc();
    let mut config = hal::adc::Config::default();
    config.clock_divider = hal::adc::ClockDivider::DIV10;
    let mut adc = hal::adc::Adc::new(p.ADC0, config);
    let mut timer = Timer::new(p.GPTMR1, Channel::Ch0, TimerConfig::default());

    defmt::info!("HPM5E ML8511 ADC validation start: PF27/ADC0_IN0");

    let stats = sample_adc(&mut adc, &mut channel, VALIDATION_SAMPLE_COUNT);
    log_reading(VALIDATION_SAMPLE_COUNT, stats);

    if stats.minimum >= SIGNAL_MIN_COUNTS && stats.maximum <= SIGNAL_MAX_COUNTS {
        HPM5E_ML8511_VALIDATION_STATUS.store(0x8000_0000 | stats.average, Ordering::Release);
        defmt::info!("HPM5E ML8511 ADC signal validation PASS");
    } else {
        HPM5E_ML8511_VALIDATION_STATUS.store(0xe600_0000 | stats.average, Ordering::Release);
        defmt::error!(
            "HPM5E ML8511 ADC signal validation FAIL: expected raw range={}..{}",
            SIGNAL_MIN_COUNTS,
            SIGNAL_MAX_COUNTS
        );
    }

    loop {
        timer.delay_ms(500);
        let stats = sample_adc(&mut adc, &mut channel, PERIODIC_SAMPLE_COUNT);
        log_reading(PERIODIC_SAMPLE_COUNT, stats);
    }
}

#[derive(Clone, Copy)]
struct SampleStats {
    minimum: u16,
    maximum: u16,
    average: u32,
}

fn sample_adc<T: hal::adc::Instance>(
    adc: &mut hal::adc::Adc<'_, T>,
    channel: &mut impl AdcChannel<T>,
    count: u32,
) -> SampleStats {
    let mut minimum = u16::MAX;
    let mut maximum = u16::MIN;
    let mut sum = 0u32;
    for _ in 0..count {
        let sample = adc.blocking_read(channel, Default::default());
        minimum = minimum.min(sample);
        maximum = maximum.max(sample);
        sum += sample as u32;
    }
    SampleStats {
        minimum,
        maximum,
        average: sum / count,
    }
}

fn log_reading(sample_count: u32, stats: SampleStats) {
    let minimum_mv = stats.minimum as u32 * ADC_REFERENCE_MV / 65535;
    let maximum_mv = stats.maximum as u32 * ADC_REFERENCE_MV / 65535;
    let average_uv = stats.average as i64 * ADC_REFERENCE_MV as i64 * 1000 / 65535;
    let average_mv = average_uv / 1000;
    let uv_uw_cm2 =
        (UV_KNOWN_POINT_UW_CM2 + (average_uv - UV_KNOWN_POINT_UV) * 1000 / UV_SENSITIVITY_UV_PER_MW_CM2).max(0);

    defmt::info!(
        "ML8511 samples={} raw_min={} raw_max={} raw_avg={} mv_min={} mv_max={} mv_avg={} uv_estimate_uw_cm2={}",
        sample_count,
        stats.minimum,
        stats.maximum,
        stats.average,
        minimum_mv,
        maximum_mv,
        average_mv,
        uv_uw_cm2
    );
}
