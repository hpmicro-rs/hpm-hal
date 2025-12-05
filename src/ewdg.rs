//! Enhanced Watchdog (EWDG) driver.
//!
//! This module supports EWDG (wdg_v53/v68) found in HPM5300, HPM6200, HPM6800 series.
//! HPM6700 series uses a different simple WDG peripheral (wdg_v67) which is not supported.
//!
//! ## Features
//!
//! - Configurable timeout (using 32K external or bus clock)
//! - Window mode: feed only within valid time window (not yet implemented)
//! - Can be disabled (unlike most MCUs)
//! - Debug and low-power mode behavior
//!
//! ## Example
//!
//! ```rust,ignore
//! use hpm_hal::ewdg::{Watchdog, Config};
//! use embassy_time::Duration;
//!
//! let mut wdg = Watchdog::new(p.EWDG0, Config::default());
//! wdg.start(Duration::from_secs(1));
//!
//! loop {
//!     // Do work...
//!     wdg.feed();
//! }
//! ```

use embassy_hal_internal::Peri;
use embassy_time::Duration;

use crate::pac;

const PARITY_BIT: u32 = 1 << 31;
const REFRESH_KEY: u32 = 0x5A45_524F;
const OSC32K_FREQ: u32 = 32768;

/// Clock source for EWDG counter
#[derive(Clone, Copy, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ClockSource {
    /// Bus clock (stops in low power mode)
    BusClock,
    #[default]
    /// External 32K oscillator (continues in low power mode)
    Osc32k,
}

/// Low power mode behavior
#[derive(Clone, Copy, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum LowPowerMode {
    /// Watchdog halts in low power mode
    Halt,
    /// Watchdog runs at 1/4 normal frequency
    Quarter,
    /// Watchdog runs at 1/2 normal frequency
    Half,
    #[default]
    /// Watchdog runs at normal frequency
    Normal,
}

impl LowPowerMode {
    fn to_bits(self) -> u8 {
        match self {
            LowPowerMode::Halt => 0,
            LowPowerMode::Quarter => 1,
            LowPowerMode::Half => 2,
            LowPowerMode::Normal => 3,
        }
    }
}

/// Watchdog configuration
#[derive(Clone)]
pub struct Config {
    /// Clock source (default: Osc32k)
    pub clock_source: ClockSource,
    /// Keep running in debug mode (default: false)
    pub run_in_debug: bool,
    /// Low power mode behavior (default: Normal)
    pub low_power_mode: LowPowerMode,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            clock_source: ClockSource::Osc32k,
            run_in_debug: false,
            low_power_mode: LowPowerMode::Normal,
        }
    }
}

/// Watchdog status flags
#[derive(Clone, Copy, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Status {
    /// Refresh violation (fed outside window)
    pub refresh_violation: bool,
    /// Refresh unlock failed
    pub refresh_unlock_fail: bool,
    /// Control register violation
    pub ctrl_violation: bool,
    /// Control unlock failed
    pub ctrl_unlock_fail: bool,
    /// Timeout interrupt pending
    pub timeout_interrupt: bool,
    /// Timeout reset pending
    pub timeout_reset: bool,
    /// Parity error
    pub parity_error: bool,
}

impl From<pac::wdg::regs::WdtStatus> for Status {
    fn from(reg: pac::wdg::regs::WdtStatus) -> Self {
        Self {
            refresh_violation: reg.ref_vio(),
            refresh_unlock_fail: reg.ref_unl_fail(),
            ctrl_violation: reg.ctl_vio(),
            ctrl_unlock_fail: reg.ctl_unl_fail(),
            timeout_interrupt: reg.ot_int(),
            timeout_reset: reg.ot_rst(),
            parity_error: reg.parity_error(),
        }
    }
}

/// Calculate parity bit for control registers
fn calc_parity(val: u32) -> u32 {
    if val.count_ones() & 1 != 0 {
        val | PARITY_BIT
    } else {
        val & !PARITY_BIT
    }
}

/// Enhanced Watchdog driver
pub struct Watchdog<'d, T: Instance> {
    _peri: Peri<'d, T>,
    clock_freq: u32,
}

impl<'d, T: Instance> Watchdog<'d, T> {
    /// Create a new watchdog with default configuration
    pub fn new(peri: Peri<'d, T>, config: Config) -> Self {
        T::add_resource_group(0);

        let r = T::regs();

        // Determine clock frequency
        let clock_freq = if config.clock_source == ClockSource::Osc32k {
            OSC32K_FREQ
        } else {
            crate::sysctl::clocks().ahb.0
        };

        // Configure CTRL0
        let mut ctrl0 = pac::wdg::regs::Ctrl0(0);
        ctrl0.set_clk_sel(config.clock_source == ClockSource::Osc32k);
        ctrl0.set_en_dbg(config.run_in_debug);
        ctrl0.set_en_lp(config.low_power_mode.to_bits());

        // Write with parity
        r.ctrl0().write_value(pac::wdg::regs::Ctrl0(calc_parity(ctrl0.0)));

        Self { _peri: peri, clock_freq }
    }

    /// Start the watchdog with specified timeout
    ///
    /// Max timeout with 32K clock: ~2000 seconds (16-bit counter, max divider)
    pub fn start(&mut self, timeout: Duration) {
        let r = T::regs();

        // Calculate timeout ticks
        let timeout_us = timeout.as_micros() as u64;
        let mut ticks = timeout_us * self.clock_freq as u64 / 1_000_000;

        // Find appropriate divider (2^div_value)
        let mut div_value = 0u8;
        while ticks > 0xFFFF && div_value < 7 {
            div_value += 1;
            ticks >>= 1;
        }
        let ticks = ticks.min(0xFFFF) as u16;

        // Update divider in CTRL0
        let ctrl0 = r.ctrl0().read();
        let mut new_ctrl0 = pac::wdg::regs::Ctrl0(ctrl0.0 & !((0x07 << 25) | PARITY_BIT));
        new_ctrl0.set_div_value(div_value);
        r.ctrl0().write_value(pac::wdg::regs::Ctrl0(calc_parity(new_ctrl0.0)));

        // Set timeout value
        r.ot_rst_val().write(|w| w.set_ot_rst_val(ticks));

        // Enable timeout reset in CTRL1
        let mut ctrl1 = pac::wdg::regs::Ctrl1(0);
        ctrl1.set_ot_rst_en(true);
        r.ctrl1().write_value(pac::wdg::regs::Ctrl1(calc_parity(ctrl1.0)));

        // Enable watchdog
        r.wdt_en().write(|w| w.set_wdog_en(true));
    }

    /// Feed (refresh) the watchdog to prevent reset
    #[inline]
    pub fn feed(&mut self) {
        T::regs().wdt_refresh_reg().write_value(pac::wdg::regs::WdtRefreshReg(REFRESH_KEY));
    }

    /// Stop the watchdog
    ///
    /// Note: Unlike most MCUs, HPM EWDG can be stopped.
    pub fn stop(&mut self) {
        T::regs().wdt_en().write(|w| w.set_wdog_en(false));
    }

    /// Check if the watchdog is running
    #[inline]
    pub fn is_running(&self) -> bool {
        T::regs().wdt_en().read().wdog_en()
    }

    /// Get current status flags
    pub fn status(&self) -> Status {
        T::regs().wdt_status().read().into()
    }

    /// Clear status flags (write 1 to clear)
    pub fn clear_status(&mut self, status: Status) {
        let mut val = pac::wdg::regs::WdtStatus(0);
        val.set_ref_vio(status.refresh_violation);
        val.set_ref_unl_fail(status.refresh_unlock_fail);
        val.set_ctl_vio(status.ctrl_violation);
        val.set_ctl_unl_fail(status.ctrl_unlock_fail);
        val.set_parity_error(status.parity_error);
        T::regs().wdt_status().write_value(val);
    }

    /// Get the current counter value (for debugging)
    pub fn counter(&self) -> u32 {
        T::regs().wdt_refresh_reg().read().wdt_refresh_reg()
    }

    /// Trigger an immediate system reset
    pub fn trigger_reset(&mut self) -> ! {
        let r = T::regs();

        // Set very short timeout
        r.ot_rst_val().write(|w| w.set_ot_rst_val(1));

        // Enable timeout reset
        let mut ctrl1 = pac::wdg::regs::Ctrl1(0);
        ctrl1.set_ot_rst_en(true);
        r.ctrl1().write_value(pac::wdg::regs::Ctrl1(calc_parity(ctrl1.0)));

        // Enable watchdog
        r.wdt_en().write(|w| w.set_wdog_en(true));

        loop {
            core::hint::spin_loop();
        }
    }
}

// Instance trait
pub(crate) trait SealedInstance {
    fn regs() -> pac::wdg::Wdg;
    fn add_resource_group(group: usize);
}

/// EWDG instance trait
#[allow(private_bounds)]
pub trait Instance: SealedInstance + crate::PeripheralType + 'static {}

#[cfg(peri_wdg0)]
impl SealedInstance for crate::peripherals::WDG0 {
    fn regs() -> pac::wdg::Wdg {
        pac::WDG0
    }
    fn add_resource_group(group: usize) {
        crate::sysctl::clock_add_to_group(pac::resources::WDG0, group);
    }
}
#[cfg(peri_wdg0)]
impl Instance for crate::peripherals::WDG0 {}

#[cfg(peri_wdg1)]
impl SealedInstance for crate::peripherals::WDG1 {
    fn regs() -> pac::wdg::Wdg {
        pac::WDG1
    }
    fn add_resource_group(group: usize) {
        crate::sysctl::clock_add_to_group(pac::resources::WDG1, group);
    }
}
#[cfg(peri_wdg1)]
impl Instance for crate::peripherals::WDG1 {}

#[cfg(peri_pwdg)]
impl SealedInstance for crate::peripherals::PWDG {
    fn regs() -> pac::wdg::Wdg {
        pac::PWDG
    }
    fn add_resource_group(_group: usize) {
        // PWDG is always on, no need to add to resource group
    }
}
#[cfg(peri_pwdg)]
impl Instance for crate::peripherals::PWDG {}

