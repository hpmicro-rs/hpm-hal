#![no_main]
#![no_std]
#![feature(abi_riscv_interrupt)]
#![feature(impl_trait_in_assoc_type)]

use core::sync::atomic::{AtomicU32, Ordering};

use defmt_rtt as _;
use embassy_time::Timer;
use hpm_hal as hal;
use hpm_hal::adc::{AdcChannel, SequenceChannel, SequenceSample};
use hpm_hal::{bind_interrupts, peripherals};
use panic_halt as _;

bind_interrupts!(struct Irqs {
    ADC0 => hal::adc::InterruptHandler<peripherals::ADC0>;
});

#[unsafe(no_mangle)]
pub static HPM5E_ADC_ASYNC_STATUS: AtomicU32 = AtomicU32::new(0);

#[embassy_executor::main(entry = "hpm_hal::entry")]
async fn main(_spawner: embassy_executor::Spawner) -> ! {
    let p = hal::init(Default::default());

    let mut config = hal::adc::Config::default();
    config.clock_divider = hal::adc::ClockDivider::DIV10;
    let mut adc = hal::adc::Adc::new_async(p.ADC0, config, Irqs);
    let mut pin = p.PF27.degrade_adc();

    defmt::info!("HPM5E ADC0 async validation start: PF27/IN0 + PF28/IN5");

    match adc.read(&mut pin, Default::default()).await {
        Ok(value) => defmt::info!("async one-shot value={}", value),
        Err(_error) => {
            HPM5E_ADC_ASYNC_STATUS.store(0xe700_0001, Ordering::Release);
            defmt::error!("async one-shot failed");
        }
    }

    let sequence = [
        SequenceChannel::new(pin, Default::default()),
        SequenceChannel::new(p.PF28.degrade_adc(), Default::default()),
    ];
    let mut samples = [SequenceSample::default(); 2];
    let mut completed = 0u32;

    loop {
        match adc.read_sequence(&sequence, &mut samples).await {
            Ok(()) => {
                completed = completed.wrapping_add(1);
                let first = samples[0];
                let second = samples[1];
                HPM5E_ADC_ASYNC_STATUS.store(0x8000_0000 | completed, Ordering::Release);
                defmt::info!(
                    "first(seq={}, ch={}, value={}) second(seq={}, ch={}, value={}) completed={}",
                    first.sequence,
                    first.channel,
                    first.value,
                    second.sequence,
                    second.channel,
                    second.value,
                    completed
                );
            }
            Err(_error) => {
                HPM5E_ADC_ASYNC_STATUS.store(0xe700_0002, Ordering::Release);
                defmt::error!("sequence read failed");
            }
        }

        Timer::after_millis(500).await;
    }
}
