//! Embassy time driver using machine timer(mchtmr)

use core::cell::{Cell, RefCell};
use core::task::Waker;

use critical_section::{CriticalSection, Mutex};
use embassy_time_driver::{Driver, TICK_HZ};
use embassy_time_queue_utils::Queue;
use hpm_metapac::sysctl::vals;
use hpm_metapac::{MCHTMR, SYSCTL};
use portable_atomic::{AtomicU64, Ordering};

use crate::sysctl::ClockConfig;
use crate::{interrupt, pac};

// Alarm state structure to manage individual alarms
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

// Machine timer based time driver implementation
pub(crate) struct MachineTimerDriver {
    // Total number of ticks since system start
    ticks: AtomicU64,
    // Number of allocated alarms
    alarm: Mutex<AlarmState>,
    queue: Mutex<RefCell<Queue>>,
}

// Constant initialization for alarm states
#[allow(clippy::declare_interior_mutable_const)]
const ALARM_STATE_NEW: AlarmState = AlarmState::new();

// Macro to create a static driver instance
embassy_time_driver::time_driver_impl!(static DRIVER: MachineTimerDriver = MachineTimerDriver {
    ticks: AtomicU64::new(0),
    alarm: Mutex::new(ALARM_STATE_NEW),
    queue: Mutex::new(RefCell::new(Queue::new()))
});

impl MachineTimerDriver {
    // Initialize the machine timer driver
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

        let cnt_per_second = crate::sysctl::clocks().get_freq(&mchtmr_cfg).0 as u64;

        // make sure mchtmr will not be gated on "wfi"
        // Design consideration: use WAIT is also useful to enter low-power mode
        SYSCTL.cpu(0).lp().modify(|w| w.set_mode(vals::LpMode::RUN));

        // 4 * 32 = 128 bits
        // enable wake up from all interrupts
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
    }

    // Machine timer interrupt handler
    fn on_interrupt(&self) {
        unsafe {
            riscv::register::mie::clear_mtimer();
        }

        critical_section::with(|cs| {
            // Increment global tick counter
            let current_ticks = self.ticks.fetch_add(1, Ordering::Relaxed);
            self.check_and_trigger_alarm(current_ticks, cs);
        })
    }

    // Check if an alarm is due and trigger it if necessary
    #[inline]
    fn check_and_trigger_alarm(&self, current_time: u64, cs: CriticalSection) {
        let alarm = &self.alarm.borrow(cs);
        let alarm_timestamp = alarm.timestamp.get();

        // Check if alarm is scheduled and due
        if alarm_timestamp != u64::MAX && current_time >= alarm_timestamp {
            let mut next = self.queue.borrow(cs).borrow_mut().next_expiration(current_time);
            while !self.set_alarm(cs, next) {
                next = self.queue.borrow(cs).borrow_mut().next_expiration(self.now());
            }
        }
    }

    // Set alarm timestamp
    fn set_alarm(&self, cs: CriticalSection, timestamp: u64) -> bool {
        if self.now() >= timestamp {
            // Alarm time has passed, cannot set
            return false;
        }
        self.alarm.borrow(cs).timestamp.set(timestamp);
        if self.now() >= timestamp {
            self.alarm.borrow(cs).timestamp.set(u64::MAX);
            return false;
        }
        true
    }
}

// Implement the Driver trait for MachineTimerDriver
impl Driver for MachineTimerDriver {
    // Get current system time in ticks
    fn now(&self) -> u64 {
        self.ticks.load(Ordering::Relaxed)
    }

    fn schedule_wake(&self, at: u64, waker: &Waker) {
        critical_section::with(|cs| {
            let mut queue = self.queue.borrow(cs).borrow_mut();

            if queue.schedule_wake(at, waker) {
                let mut next = queue.next_expiration(self.now());
                while !self.set_alarm(cs, next) {
                    next = queue.next_expiration(self.now());
                }
            }
        })
    }
}

// Machine timer interrupt handler (to be implemented in your interrupt vector)
#[interrupt]
fn MachineTimer() {
    DRIVER.on_interrupt();
}

pub(crate) fn init() {
    DRIVER.init();
}
