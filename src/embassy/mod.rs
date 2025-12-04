//! The embassy time driver for HPMicro MCUs.
//!
//! ## Time Driver Options
//!
//! - **MCHTMR (default)**: Uses 64-bit machine timer, most accurate
//! - **GPTMR**: Uses 32-bit general purpose timer with period mechanism
//!
//! Enable `time-driver-gptmr0` or `time-driver-gptmr1` feature to use GPTMR instead.

#[cfg(not(any(time_driver_gptmr0, time_driver_gptmr1)))]
#[path = "time_driver_mchtmr.rs"]
pub mod time_driver_impl;

#[cfg(any(time_driver_gptmr0, time_driver_gptmr1))]
#[path = "time_driver_gptmr.rs"]
pub mod time_driver_impl;

// This should be called after global clocks inited
pub(crate) fn init() {
    #[cfg(not(any(time_driver_gptmr0, time_driver_gptmr1)))]
    time_driver_impl::init();

    #[cfg(any(time_driver_gptmr0, time_driver_gptmr1))]
    critical_section::with(|cs| {
        time_driver_impl::init(cs);
    });
}
