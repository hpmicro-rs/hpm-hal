//! Embassy time driver using GPTMR (General Purpose Timer)
//!
//! This driver provides an alternative to MCHTMR for embassy-time.
//!
//! ## Design
//!
//! GPTMR has 32-bit counters, so we use a "period" mechanism similar to STM32:
//! - period (32-bit) + counter (32-bit) = 63-bit effective timestamp
//! - period increments at overflow (CNT=0) and halfway (CNT=0x8000_0000)
//!
//! ## Channel Assignment
//!
//! Only Channel 0 is used (HPM GPTMR channels have independent counters):
//! - CMP0: halfway interrupt (period tracking)
//! - CMP1: alarm interrupt
//! - RLD: overflow interrupt (period tracking)
//!
//! ## Usage
//!
//! Enable feature `time-driver-gptmr0` in Cargo.toml instead of default MCHTMR driver.
//!
//! ## Important Notes
//!
//! - `CR.CMPEN` must be enabled for CMP interrupts to trigger
//! - Clock conversion: `ticks_per_tick = timer_freq / TICK_HZ`

use core::cell::{Cell, RefCell};
use core::sync::atomic::{AtomicU32, Ordering, compiler_fence};

use critical_section::CriticalSection;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time_driver::Driver;
use embassy_time_queue_utils::Queue;

use crate::interrupt::InterruptExt;
use crate::pac;
use crate::pac::tmr::Tmr;

#[cfg(time_driver_gptmr0)]
const GPTMR: Tmr = pac::GPTMR0;
#[cfg(time_driver_gptmr1)]
const GPTMR: Tmr = pac::GPTMR1;

/// We only use Channel 0 since each HPM GPTMR channel has independent counter
const CH: usize = 0;

/// Halfway point for period increment
const HALF: u32 = 0x8000_0000;

// Period mechanism (adapted from STM32):
// - `period` is a 32-bit counter of 2^31 hardware tick intervals
// - `period` increments at overflow (CNT = 0) and halfway (CNT = HALF)
// - When period is even, counter is in 0..HALF-1
// - When period is odd, counter is in HALF..MAX
//
// Returns raw hardware ticks, needs to be divided by ticks_per_tick for embassy ticks
fn calc_now_hw(period: u32, counter: u32) -> u64 {
    ((period as u64) << 31) + ((counter ^ ((period & 1) << 31)) as u64)
}

struct AlarmState {
    timestamp: Cell<u64>,
}

unsafe impl Send for AlarmState {}

impl AlarmState {
    const fn new() -> Self {
        Self {
            timestamp: Cell::new(u64::MAX),
        }
    }
}

pub(crate) struct GptmrDriver {
    /// Number of 2^31 periods elapsed since boot
    period: AtomicU32,
    /// Hardware ticks per embassy tick (timer_freq / TICK_HZ)
    ticks_per_tick: AtomicU32,
    /// Alarm state
    alarm: Mutex<CriticalSectionRawMutex, AlarmState>,
    /// Timer queue
    queue: Mutex<CriticalSectionRawMutex, RefCell<Queue>>,
}

embassy_time_driver::time_driver_impl!(static DRIVER: GptmrDriver = GptmrDriver {
    period: AtomicU32::new(0),
    ticks_per_tick: AtomicU32::new(1), // Will be set in init
    alarm: Mutex::const_new(CriticalSectionRawMutex::new(), AlarmState::new()),
    queue: Mutex::const_new(CriticalSectionRawMutex::new(), RefCell::new(Queue::new())),
});

impl GptmrDriver {
    fn init(&'static self, cs: CriticalSection) {
        use embassy_time_driver::TICK_HZ;

        // Enable GPTMR clock (resource name is TMRx, not GPTMRx)
        #[cfg(time_driver_gptmr0)]
        crate::sysctl::clock_add_to_group(pac::resources::TMR0, 0);
        #[cfg(time_driver_gptmr1)]
        crate::sysctl::clock_add_to_group(pac::resources::TMR1, 0);

        let r = GPTMR;

        // Get timer frequency (clock name is TMRx, not GPTMRx)
        #[cfg(time_driver_gptmr0)]
        let timer_freq = crate::sysctl::clocks().get_clock_freq(pac::clocks::TMR0);
        #[cfg(time_driver_gptmr1)]
        let timer_freq = crate::sysctl::clocks().get_clock_freq(pac::clocks::TMR1);

        // Calculate ticks_per_tick: how many hardware ticks per embassy tick
        let ticks_per_tick = timer_freq.0 as u64 / TICK_HZ;
        self.ticks_per_tick.store(ticks_per_tick as u32, Ordering::Relaxed);

        // Stop and reset channel 0
        r.channel(CH).cr().write(|w| {
            w.set_cen(false);
            w.set_cmpen(false);
        });

        // Configure channel 0:
        // - Reload at max value (free-running)
        // - CMP0 at halfway point for mid-period interrupt (period tracking)
        // - CMP1 for alarm
        r.channel(CH).rld().write_value(u32::MAX - 1);
        r.channel(CH).cmp(0).write_value(HALF - 1); // halfway
        r.channel(CH).cmp(1).write_value(u32::MAX - 1); // alarm (disabled initially)
        r.channel(CH).cr().write(|w| {
            w.set_dbgpause(true);
            w.set_cmpen(true); // CRITICAL: Enable compare function for CMP interrupts!
        });

        // Reset counter
        r.channel(CH).cr().modify(|w| w.set_cntrst(true));
        r.channel(CH).cr().modify(|w| w.set_cntrst(false));

        // Clear all status flags
        r.sr().write_value(pac::tmr::regs::Sr(0xFFFF_FFFF));

        // Enable interrupts:
        // - Ch0 reload (overflow) for period tracking
        // - Ch0 CMP0 (halfway) for period tracking
        // - Ch0 CMP1 (alarm) - initially disabled
        r.irqen().write(|w| {
            w.set_chrlden(CH, true);  // Overflow
            w.set_chcmp0en(CH, true); // Halfway
            w.set_chcmp1en(CH, false); // Alarm (disabled until set)
        });

        // Start channel 0
        r.channel(CH).cr().modify(|w| w.set_cen(true));

        // Enable GPTMR interrupt in PLIC
        #[cfg(time_driver_gptmr0)]
        unsafe {
            crate::interrupt::GPTMR0.enable();
        }
        #[cfg(time_driver_gptmr1)]
        unsafe {
            crate::interrupt::GPTMR1.enable();
        }

        let _ = cs;
    }

    pub(crate) fn on_interrupt(&self) {
        let r = GPTMR;

        critical_section::with(|cs| {
            let sr = r.sr().read();

            // Clear handled flags (write 1 to clear)
            r.sr().write_value(sr);

            // Overflow (reload) - increment period
            if sr.chrldf(CH) {
                self.next_period();
            }

            // Halfway - increment period
            if sr.chcmp0f(CH) {
                self.next_period();
            }

            // Alarm (CMP1)
            if sr.chcmp1f(CH) {
                self.trigger_alarm(cs);
            }
        });
    }

    fn next_period(&self) {
        let r = GPTMR;
        let ticks_per_tick = self.ticks_per_tick.load(Ordering::Relaxed) as u64;

        // Increment period (only called from interrupt, no race)
        let period = self.period.load(Ordering::Relaxed) + 1;
        self.period.store(period, Ordering::Relaxed);

        // Current embassy time at start of new period
        let t = ((period as u64) << 31) / ticks_per_tick;

        critical_section::with(|cs| {
            let alarm = self.alarm.borrow(cs);
            let at = alarm.timestamp.get();

            // If alarm is coming up soon (within 3/4 period), enable alarm interrupt
            let threshold = 0xC000_0000u64 / ticks_per_tick;
            if at < t + threshold {
                r.irqen().modify(|w| w.set_chcmp1en(CH, true));
            }
        });
    }

    fn trigger_alarm(&self, cs: CriticalSection) {
        let mut next = self.queue.borrow(cs).borrow_mut().next_expiration(self.now());
        while !self.set_alarm(cs, next) {
            next = self.queue.borrow(cs).borrow_mut().next_expiration(self.now());
        }
    }

    fn set_alarm(&self, cs: CriticalSection, timestamp: u64) -> bool {
        let r = GPTMR;
        let ticks_per_tick = self.ticks_per_tick.load(Ordering::Relaxed) as u64;

        let alarm = self.alarm.borrow(cs);
        alarm.timestamp.set(timestamp);

        let t = self.now();
        if timestamp <= t {
            // Alarm already passed
            r.irqen().modify(|w| w.set_chcmp1en(CH, false));
            alarm.timestamp.set(u64::MAX);
            return false;
        }

        // Convert embassy timestamp to hardware counter value (low 32 bits)
        let hw_timestamp = timestamp.saturating_mul(ticks_per_tick);
        let cmp_val = hw_timestamp as u32;

        // Set CMP1 for alarm
        r.channel(CH).cmp(1).write_value(cmp_val.saturating_sub(1));

        // Enable alarm if it's coming soon (within ~3/4 of period in embassy ticks)
        let diff = timestamp - t;
        let threshold = 0xC000_0000u64 / ticks_per_tick;
        r.irqen().modify(|w| w.set_chcmp1en(CH, diff < threshold));

        // Re-check to handle race
        let t = self.now();
        if timestamp <= t {
            r.irqen().modify(|w| w.set_chcmp1en(CH, false));
            alarm.timestamp.set(u64::MAX);
            return false;
        }

        true
    }
}

impl GptmrDriver {
    /// Get current time in hardware ticks
    fn now_hw(&self) -> u64 {
        let r = GPTMR;

        let period = self.period.load(Ordering::Relaxed);
        compiler_fence(Ordering::Acquire);
        let counter = r.channel(CH).cnt().read();

        calc_now_hw(period, counter)
    }
}

impl Driver for GptmrDriver {
    fn now(&self) -> u64 {
        let ticks_per_tick = self.ticks_per_tick.load(Ordering::Relaxed) as u64;
        // Convert hardware ticks to embassy ticks
        self.now_hw() / ticks_per_tick
    }

    fn schedule_wake(&self, at: u64, waker: &core::task::Waker) {
        critical_section::with(|cs| {
            let mut queue = self.queue.borrow(cs).borrow_mut();

            if queue.schedule_wake(at, waker) {
                let mut next = queue.next_expiration(self.now());
                while !self.set_alarm(cs, next) {
                    next = queue.next_expiration(self.now());
                }
            }
        });
    }
}

pub(crate) fn init(cs: CriticalSection) {
    DRIVER.init(cs);
}

/// Called from auto-generated interrupt handler in build.rs
pub(crate) fn on_interrupt() {
    DRIVER.on_interrupt();
}
