#![no_main]
#![no_std]
#![feature(impl_trait_in_assoc_type)]

use defmt_rtt as _;
use embassy_time::{Duration, Instant, Timer};
use hpm_hal as hal;
use hpm_hal::gpio::{Flex, Pull};
use panic_halt as _;

struct SignalStats {
    samples: u32,
    high: u32,
    low: u32,
    edges: u32,
}

#[embassy_executor::main(entry = "hpm_hal::entry")]
async fn main(_spawner: embassy_executor::Spawner) -> ! {
    let p = hal::init(Default::default());
    let mut pin = Flex::new(p.PC14);
    pin.set_as_input(Pull::None);

    defmt::info!("PMS7003 PC14 signal diagnostic");

    loop {
        let floating = sample(&pin, Duration::from_secs(2));
        report("no-pull", &floating);

        pin.set_pull(Pull::Up);
        Timer::after_millis(10).await;
        let pulled_up = sample(&pin, Duration::from_secs(2));
        report("pull-up", &pulled_up);

        pin.set_pull(Pull::None);
        Timer::after_secs(1).await;
    }
}

fn sample(pin: &Flex<'_>, duration: Duration) -> SignalStats {
    let deadline = Instant::now() + duration;
    let mut previous = pin.is_high();
    let mut stats = SignalStats {
        samples: 0,
        high: 0,
        low: 0,
        edges: 0,
    };

    while Instant::now() < deadline {
        let high = pin.is_high();
        stats.samples = stats.samples.wrapping_add(1);
        if high {
            stats.high = stats.high.wrapping_add(1);
        } else {
            stats.low = stats.low.wrapping_add(1);
        }
        if high != previous {
            stats.edges = stats.edges.wrapping_add(1);
            previous = high;
        }
    }

    stats
}

fn report(mode: &str, stats: &SignalStats) {
    defmt::info!(
        "PC14 {} samples={} high={} low={} edges={}",
        mode,
        stats.samples,
        stats.high,
        stats.low,
        stats.edges
    );
}
