//! Embassy time driver using machine timer(mchtmr)

use core::cell::{Cell, RefCell};
use core::sync::atomic::{AtomicU32, Ordering};

use critical_section::CriticalSection;
use embassy_sync::blocking_mutex::Mutex as BlockingMutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time_driver::Driver;
use embassy_time_queue_utils::Queue;
use hpm_metapac::sysctl::vals;
use hpm_metapac::{MCHTMR, SYSCTL};

use crate::pac;
use crate::sysctl::ClockConfig;

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

/// HPM Machine Timer Driver using 64-bit MCHTMR peripheral
pub struct MachineTimerDriver {
    /// Hardware counter to Embassy tick conversion factor
    period: AtomicU32,
    /// Current hardware alarm state
    alarm: BlockingMutex<CriticalSectionRawMutex, AlarmState>,
    /// Standard queue for managing concurrent timers
    queue: BlockingMutex<CriticalSectionRawMutex, RefCell<Queue>>,
}

embassy_time_driver::time_driver_impl!(static DRIVER: MachineTimerDriver = MachineTimerDriver {
    period: AtomicU32::new(1), // avoid div by zero, will be set in init
    alarm: BlockingMutex::const_new(CriticalSectionRawMutex::new(), AlarmState::new()),
    queue: BlockingMutex::const_new(CriticalSectionRawMutex::new(), RefCell::new(Queue::new())),
});

impl MachineTimerDriver {
    fn init(&'static self) {
        // FIXME: The name in SDK is MCHTMR0
        #[cfg(hpm67)]
        let regs = SYSCTL.clock(pac::clocks::MCHTMR0).read();
        #[cfg(not(hpm67))]
        let regs = SYSCTL.clock(pac::clocks::MCT0).read();

        let mchtmr_cfg = ClockConfig {
            src: regs.mux(),
            raw_div: regs.div(),
        };

        // Calculate precise timer period for high accuracy tick
        let cnt_per_second = crate::sysctl::clocks().get_freq(&mchtmr_cfg).0 as u64;
        let cnt_per_tick = cnt_per_second / embassy_time_driver::TICK_HZ;

        self.period.store(cnt_per_tick as u32, Ordering::Relaxed);

        // Ensure MCHTMR will not be clock gated on WFI
        // Note: WAIT mode can also be used for low-power operation
        SYSCTL.cpu(0).lp().modify(|w| w.set_mode(vals::LpMode::RUN));

        // Enable wake up from all interrupts (128 bits = 4 * 32)
        SYSCTL.cpu(0).wakeup_enable(0).write(|w| w.set_enable(0xFFFFFFFF));
        SYSCTL.cpu(0).wakeup_enable(1).write(|w| w.set_enable(0xFFFFFFFF));
        SYSCTL.cpu(0).wakeup_enable(2).write(|w| w.set_enable(0xFFFFFFFF));
        SYSCTL.cpu(0).wakeup_enable(3).write(|w| w.set_enable(0xFFFFFFFF));
        #[cfg(hpm67)]
        {
            SYSCTL.cpu(0).wakeup_enable(4).write(|w| w.set_enable(0xFFFFFFFF));
            SYSCTL.cpu(0).wakeup_enable(5).write(|w| w.set_enable(0xFFFFFFFF));
            SYSCTL.cpu(0).wakeup_enable(6).write(|w| w.set_enable(0xFFFFFFFF));
            SYSCTL.cpu(0).wakeup_enable(7).write(|w| w.set_enable(0xFFFFFFFF));
        }

        MCHTMR.mtimecmp().write_value(u64::MAX - 1);

        // Enable global machine mode interrupts
        unsafe {
            riscv::register::mstatus::set_mie();
        }
    }

    #[inline(always)]
    fn on_interrupt(&self) {
        unsafe {
            riscv::register::mie::clear_mtimer();
        }

        critical_section::with(|cs| {
            self.trigger_alarm(cs);
        })
    }

    /// Set hardware alarm for the given timestamp
    fn set_alarm(&self, cs: CriticalSection, timestamp: u64) -> bool {
        let alarm = self.alarm.borrow(cs);
        alarm.timestamp.set(timestamp);

        let now = self.now();
        if timestamp <= now {
            alarm.timestamp.set(u64::MAX);
            return false;
        }

        // Convert embassy timestamp to hardware timestamp
        let period = self.period.load(Ordering::Relaxed) as u64;
        let hardware_timestamp = timestamp
            .saturating_add(1) // Ensure alarm doesn't trigger immediately
            .overflowing_mul(period) // Handle multiplication overflow
            .0;

        // Set MCHTMR comparison register
        MCHTMR.mtimecmp().write_value(hardware_timestamp);

        // Enable machine timer interrupt
        unsafe {
            riscv::register::mie::set_mtimer();
        }

        true
    }

    #[inline(always)]
    fn trigger_alarm(&self, cs: CriticalSection) {
        // Process expired timers from queue
        let mut queue = self.queue.borrow(cs).borrow_mut();
        let mut next = queue.next_expiration(self.now());

        // Retry until alarm is successfully set
        while !self.set_alarm(cs, next) {
            next = queue.next_expiration(self.now());
        }
    }
}

impl embassy_time_driver::Driver for MachineTimerDriver {
    fn now(&self) -> u64 {
        // Read 64-bit MCHTMR counter directly, no overflow handling needed
        MCHTMR.mtime().read() / self.period.load(Ordering::Relaxed) as u64
    }

    fn schedule_wake(&self, at: u64, waker: &core::task::Waker) {
        critical_section::with(|cs| {
            let mut queue = self.queue.borrow(cs).borrow_mut();

            // Use standard queue for managing concurrent timers
            let should_update = queue.schedule_wake(at, waker);

            if should_update {
                // Update hardware alarm with earliest expiration time
                let mut next = queue.next_expiration(self.now());
                while !self.set_alarm(cs, next) {
                    next = queue.next_expiration(self.now());
                }
            }
        })
    }
}

#[riscv_rt::core_interrupt(riscv::interrupt::machine::Interrupt::MachineTimer)]
fn machine_timer() {
    DRIVER.on_interrupt();
}

pub(crate) fn init() {
    DRIVER.init();
}
