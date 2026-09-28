#![no_main]
#![no_std]

use core::sync::atomic::{AtomicU32, Ordering};

use defmt_rtt as _;
use hpm_hal as hal;
use hpm_hal::gpio::{Level, Output};
use hpm_hal::timer::{Channel, Config, Timer};
use panic_halt as _;

const ITERATIONS: u32 = 32;
const PERIOD_MS: u32 = 10;
const TOLERANCE_PERCENT: u64 = 10;
const MCHTMR_HZ: u64 = 24_000_000;

#[unsafe(no_mangle)]
pub static HPM5E_TIMER_GPIO_VALIDATION_STATUS: AtomicU32 = AtomicU32::new(0);

#[hal::entry]
fn main() -> ! {
    let p = hal::init(Default::default());
    let cpu_hz = hal::sysctl::clocks().cpu0.0 as u64;
    let mut timer = Timer::new(p.GPTMR1, Channel::Ch0, Config::default());
    let timer_hz = timer.input_frequency().0;
    let ticks = timer_hz / (1000 / PERIOD_MS);
    let expected_mtime_ticks = MCHTMR_HZ * PERIOD_MS as u64 / 1000;
    let minimum_mtime_ticks = expected_mtime_ticks * (100 - TOLERANCE_PERCENT) / 100;
    let maximum_mtime_ticks = expected_mtime_ticks * (100 + TOLERANCE_PERCENT) / 100;
    let mut led = Output::new(p.PD06, Level::Low, Default::default());

    defmt::info!(
        "HPM5E GPTMR1/GPIO validation start: timer_hz={} cpu_hz={} ticks={}",
        timer_hz,
        cpu_hz,
        ticks
    );

    for iteration in 0..ITERATIONS {
        let before_level = led.is_set_high();
        let mcycle_start = riscv::register::mcycle::read64();
        let mtime_start = read_mtime();
        timer.delay_ticks(ticks);
        let mtime_elapsed = read_mtime() - mtime_start;
        let mcycle_elapsed = riscv::register::mcycle::read64() - mcycle_start;
        led.toggle();

        if iteration == 0 {
            defmt::info!(
                "cross-check first interval: mtime_ticks={} mcycle_ticks={}",
                mtime_elapsed,
                mcycle_elapsed
            );
        }

        if led.is_set_high() == before_level {
            HPM5E_TIMER_GPIO_VALIDATION_STATUS.store(0xe400_0000 | iteration, Ordering::Release);
            defmt::error!("GPIO output latch failed at iteration={}", iteration);
            wait_forever();
        }
        if mtime_elapsed < minimum_mtime_ticks || mtime_elapsed > maximum_mtime_ticks {
            HPM5E_TIMER_GPIO_VALIDATION_STATUS.store(0xe500_0000 | iteration, Ordering::Release);
            defmt::error!(
                "GPTMR timing failed iteration={} mtime_elapsed={} expected={} range={}..{} mcycle_elapsed={}",
                iteration,
                mtime_elapsed,
                expected_mtime_ticks,
                minimum_mtime_ticks,
                maximum_mtime_ticks,
                mcycle_elapsed
            );
            wait_forever();
        }
    }

    HPM5E_TIMER_GPIO_VALIDATION_STATUS.store(ITERATIONS, Ordering::Release);
    defmt::info!("HPM5E GPTMR1/GPIO validation PASS, iterations={}", ITERATIONS);
    wait_forever()
}

fn read_mtime() -> u64 {
    let registers = hal::pac::MCHTMR.as_ptr() as *const u32;
    loop {
        let high_before = unsafe { core::ptr::read_volatile(registers.add(1)) };
        let low = unsafe { core::ptr::read_volatile(registers) };
        let high_after = unsafe { core::ptr::read_volatile(registers.add(1)) };
        if high_before == high_after {
            return (high_after as u64) << 32 | low as u64;
        }
    }
}

fn wait_forever() -> ! {
    loop {
        core::hint::spin_loop();
    }
}
