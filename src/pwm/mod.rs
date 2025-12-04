//! PWM - Pulse Width Modulation
//!
//! HPM MCU's PWM module provides:
//! - 8 PWM output channels (can be paired for complementary output)
//! - 24 comparators (v53/v67) or 16 (v62), flexibly assigned to channels
//! - Shadow register mechanism for glitch-free updates
//! - Dead-time insertion for complementary pairs
//! - Fault protection (4 internal + 2 external inputs)
//! - Input capture mode
//! - High-resolution PWM (v62 only)
//!
//! # Note
//!
//! This module covers PWM classic (v53/v62/v67).
//! PWMV2 (v6e, HPM6E00/5E00) has a different architecture and is not yet implemented.
//!
//! # Example
//!
//! ```rust,ignore
//! use hpm_hal::pwm::{SimplePwm, SimplePwmConfig, Channel};
//!
//! let config = SimplePwmConfig {
//!     frequency: Hertz(20_000),
//!     ..Default::default()
//! };
//!
//! let mut pwm = SimplePwm::new(p.PWM1, &mut p.PA23, config);
//! pwm.set_duty(Channel::Ch7, pwm.max_duty() / 2); // 50%
//! pwm.enable(Channel::Ch7);
//! ```

use embassy_hal_internal::Peri;

use crate::pac;
use crate::time::Hertz;

/// PWM output channel index (0-7)
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum Channel {
    Ch0 = 0,
    Ch1 = 1,
    Ch2 = 2,
    Ch3 = 3,
    Ch4 = 4,
    Ch5 = 5,
    Ch6 = 6,
    Ch7 = 7,
}

impl Channel {
    #[inline]
    pub const fn index(&self) -> usize {
        *self as usize
    }
}

/// Comparator index
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Comparator(pub u8);

impl Comparator {
    #[inline]
    pub const fn index(&self) -> usize {
        self.0 as usize
    }
}

/// PWM output polarity
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Polarity {
    #[default]
    ActiveHigh,
    ActiveLow,
}

/// Shadow register update timing
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ShadowUpdateMode {
    /// Update on software lock
    OnSoftwareLock,
    /// Update immediately after write
    #[default]
    Immediate,
    /// Update on hardware event (comparator match)
    OnHardwareEvent,
    /// Update on SHSYNCI signal
    OnSyncInput,
}

impl From<ShadowUpdateMode> for pac::pwm::vals::ShadowUpdateTrigger {
    fn from(mode: ShadowUpdateMode) -> Self {
        match mode {
            ShadowUpdateMode::OnSoftwareLock => pac::pwm::vals::ShadowUpdateTrigger::ON_SHLK,
            ShadowUpdateMode::Immediate => pac::pwm::vals::ShadowUpdateTrigger::ON_MODIFY,
            ShadowUpdateMode::OnHardwareEvent => pac::pwm::vals::ShadowUpdateTrigger::ON_HW_EVENT,
            ShadowUpdateMode::OnSyncInput => pac::pwm::vals::ShadowUpdateTrigger::ON_SHSYNCI,
        }
    }
}

/// Fault output mode
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FaultMode {
    /// Force output low
    #[default]
    ForceLow,
    /// Force output high
    ForceHigh,
    /// High impedance (disable output)
    HighZ,
}

/// Fault recovery timing
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FaultRecovery {
    /// Recover immediately when fault clears
    #[default]
    Immediate,
    /// Recover at counter reload
    OnReload,
    /// Recover on hardware event
    OnHardwareEvent,
    /// Recover on software clear
    OnSoftwareClear,
}

// =============================================================================
// SimplePwm
// =============================================================================

/// Simple PWM configuration
#[derive(Clone)]
pub struct SimplePwmConfig {
    /// PWM frequency
    pub frequency: Hertz,
    /// Output polarity
    pub polarity: Polarity,
    /// Shadow update mode
    pub shadow_update: ShadowUpdateMode,
}

impl Default for SimplePwmConfig {
    fn default() -> Self {
        Self {
            frequency: Hertz(1_000), // 1kHz default
            polarity: Polarity::ActiveHigh,
            shadow_update: ShadowUpdateMode::Immediate,
        }
    }
}

/// Simple PWM driver for independent channels
pub struct SimplePwm<'d, T: Instance> {
    _peri: Peri<'d, T>,
    reload: u32,
}

impl<'d, T: Instance> SimplePwm<'d, T> {
    /// Create a new SimplePwm.
    ///
    /// This configures the PWM with the given frequency.
    /// Use `enable(channel)` to start output on a specific channel.
    pub fn new(peri: Peri<'d, T>, config: SimplePwmConfig) -> Self {
        T::add_resource_group(0);

        let r = T::regs();

        // Stop counter first
        r.gcr().modify(|w| w.set_cen(false));

        // Calculate reload value from frequency
        let clk_freq = T::frequency().0;
        let reload = (clk_freq / config.frequency.0).max(1);

        // Set start and reload
        r.sta().write(|w| {
            w.set_sta(0);
            w.set_xsta(0);
        });
        r.rld().write(|w| {
            w.set_rld(reload - 1);
            w.set_xrld(0);
        });

        // Configure shadow update mode
        r.shcr()
            .modify(|w| w.set_cntshdwupt(config.shadow_update.into()));

        // Start counter
        r.gcr().modify(|w| w.set_cen(true));

        Self { _peri: peri, reload }
    }

    /// Get raw register access for advanced operations
    #[inline]
    pub fn regs(&self) -> pac::pwm::Pwm {
        T::regs()
    }

    /// Get max duty cycle value (reload value)
    #[inline]
    pub fn max_duty(&self) -> u32 {
        self.reload
    }

    /// Configure and enable a channel for PWM output.
    ///
    /// The comparator with the same index as the channel is used.
    pub fn enable(&mut self, ch: Channel) {
        let r = T::regs();
        let idx = ch.index();

        // Configure comparator for output compare mode
        r.cmpcfg(idx).modify(|w| {
            w.set_cmpmode(pac::pwm::vals::CmpMode::OUTPUT_COMPARE);
            w.set_cmpshdwupt(pac::pwm::vals::ShadowUpdateTrigger::ON_MODIFY);
        });

        // Configure channel to use this comparator
        r.chcfg(idx).modify(|w| {
            w.set_cmpselbeg(idx as u8);
            w.set_cmpselend(idx as u8);
            w.set_outpol(false); // Active high
        });

        // Enable output
        r.pwmcfg(idx).modify(|w| {
            w.set_oen(true);
            w.set_pair(false);
        });
    }

    /// Disable a channel's output
    pub fn disable(&mut self, ch: Channel) {
        T::regs().pwmcfg(ch.index()).modify(|w| w.set_oen(false));
    }

    /// Set duty cycle for a channel (0 to max_duty)
    pub fn set_duty(&mut self, ch: Channel, duty: u32) {
        let duty = duty.min(self.reload);
        T::regs().cmp(ch.index()).modify(|w| {
            w.set_cmp(duty);
            w.set_xcmp(0);
        });
    }

    /// Set duty cycle as percentage (0-100)
    pub fn set_duty_percent(&mut self, ch: Channel, percent: u8) {
        let percent = percent.min(100);
        let duty = (self.reload as u64 * percent as u64 / 100) as u32;
        self.set_duty(ch, duty);
    }

    /// Get current duty cycle
    pub fn get_duty(&self, ch: Channel) -> u32 {
        T::regs().cmp(ch.index()).read().cmp()
    }

    /// Set output polarity for a channel
    pub fn set_polarity(&mut self, ch: Channel, polarity: Polarity) {
        T::regs()
            .chcfg(ch.index())
            .modify(|w| w.set_outpol(polarity == Polarity::ActiveLow));
    }

    /// Set frequency (affects all channels)
    pub fn set_frequency(&mut self, freq: Hertz) {
        let r = T::regs();
        let clk_freq = T::frequency().0;
        let reload = (clk_freq / freq.0).max(1);

        r.rld().write(|w| {
            w.set_rld(reload - 1);
            w.set_xrld(0);
        });
        self.reload = reload;
    }

    /// Stop the PWM counter
    pub fn stop(&mut self) {
        T::regs().gcr().modify(|w| w.set_cen(false));
    }

    /// Start the PWM counter
    pub fn start(&mut self) {
        T::regs().gcr().modify(|w| w.set_cen(true));
    }
}

/// Split SimplePwm into individual channel handles for embedded-hal compatibility
pub struct SimplePwmChannel<'d, T: Instance> {
    pwm: &'d mut SimplePwm<'d, T>,
    channel: Channel,
}

impl<'d, T: Instance> SimplePwmChannel<'d, T> {
    /// Create a channel handle from a SimplePwm reference
    pub fn new(pwm: &'d mut SimplePwm<'d, T>, channel: Channel) -> Self {
        Self { pwm, channel }
    }
}

impl<T: Instance> embedded_hal::pwm::ErrorType for SimplePwmChannel<'_, T> {
    type Error = core::convert::Infallible;
}

impl<T: Instance> embedded_hal::pwm::SetDutyCycle for SimplePwmChannel<'_, T> {
    fn max_duty_cycle(&self) -> u16 {
        // Scale to u16 range
        (self.pwm.max_duty().min(u16::MAX as u32)) as u16
    }

    fn set_duty_cycle(&mut self, duty: u16) -> Result<(), Self::Error> {
        // Scale from u16 to actual range
        let max = self.pwm.max_duty();
        let scaled = if max > u16::MAX as u32 {
            (duty as u64 * max as u64 / u16::MAX as u64) as u32
        } else {
            duty as u32
        };
        self.pwm.set_duty(self.channel, scaled);
        Ok(())
    }
}

// =============================================================================
// ComplementaryPwm (placeholder)
// =============================================================================

/// Complementary PWM configuration
#[derive(Clone)]
pub struct ComplementaryPwmConfig {
    /// PWM frequency
    pub frequency: Hertz,
    /// Dead-time in nanoseconds
    pub dead_time_ns: u32,
    /// Polarity for positive output
    pub polarity_p: Polarity,
    /// Polarity for negative output
    pub polarity_n: Polarity,
    /// Fault mode
    pub fault_mode: FaultMode,
    /// Fault recovery
    pub fault_recovery: FaultRecovery,
}

impl Default for ComplementaryPwmConfig {
    fn default() -> Self {
        Self {
            frequency: Hertz(20_000),
            dead_time_ns: 100,
            polarity_p: Polarity::ActiveHigh,
            polarity_n: Polarity::ActiveHigh,
            fault_mode: FaultMode::ForceLow,
            fault_recovery: FaultRecovery::Immediate,
        }
    }
}

/// Complementary PWM driver for paired channels with dead-time
///
/// Pairs: (Ch0,Ch1), (Ch2,Ch3), (Ch4,Ch5), (Ch6,Ch7)
pub struct ComplementaryPwm<'d, T: Instance> {
    _peri: Peri<'d, T>,
    reload: u32,
}

impl<'d, T: Instance> ComplementaryPwm<'d, T> {
    /// Create complementary PWM for a specific pair.
    ///
    /// pair: 0 = (Ch0,Ch1), 1 = (Ch2,Ch3), 2 = (Ch4,Ch5), 3 = (Ch6,Ch7)
    pub fn new(peri: Peri<'d, T>, _pair: u8, config: ComplementaryPwmConfig) -> Self {
        T::add_resource_group(0);

        let r = T::regs();

        // Stop counter
        r.gcr().modify(|w| w.set_cen(false));

        // Calculate reload value
        let clk_freq = T::frequency().0;
        let reload = (clk_freq / config.frequency.0).max(1);

        // Set start and reload
        r.sta().write(|w| {
            w.set_sta(0);
            w.set_xsta(0);
        });
        r.rld().write(|w| {
            w.set_rld(reload - 1);
            w.set_xrld(0);
        });

        // Configure shadow update
        r.shcr()
            .modify(|w| w.set_cntshdwupt(pac::pwm::vals::ShadowUpdateTrigger::ON_MODIFY));

        // Start counter
        r.gcr().modify(|w| w.set_cen(true));

        Self { _peri: peri, reload }
    }

    /// Get max duty value
    #[inline]
    pub fn max_duty(&self) -> u32 {
        self.reload
    }

    /// Enable a complementary pair.
    ///
    /// pair: 0 = (Ch0,Ch1), 1 = (Ch2,Ch3), etc.
    pub fn enable(&mut self, pair: u8) {
        let r = T::regs();
        let ch_p = (pair * 2) as usize;
        let ch_n = ch_p + 1;

        // Configure comparators
        for idx in [ch_p, ch_n] {
            r.cmpcfg(idx).modify(|w| {
                w.set_cmpmode(pac::pwm::vals::CmpMode::OUTPUT_COMPARE);
                w.set_cmpshdwupt(pac::pwm::vals::ShadowUpdateTrigger::ON_MODIFY);
            });

            r.chcfg(idx).modify(|w| {
                w.set_cmpselbeg(idx as u8);
                w.set_cmpselend(idx as u8);
            });
        }

        // Configure pair mode with dead-time
        r.pwmcfg(ch_p).modify(|w| {
            w.set_oen(true);
            w.set_pair(true);
            w.set_deadarea(10); // TODO: calculate from config
        });

        r.pwmcfg(ch_n).modify(|w| {
            w.set_oen(true);
            w.set_pair(true);
        });
    }

    /// Disable a complementary pair
    pub fn disable(&mut self, pair: u8) {
        let r = T::regs();
        let ch_p = (pair * 2) as usize;
        let ch_n = ch_p + 1;

        r.pwmcfg(ch_p).modify(|w| w.set_oen(false));
        r.pwmcfg(ch_n).modify(|w| w.set_oen(false));
    }

    /// Set duty cycle for a pair (affects both outputs)
    pub fn set_duty(&mut self, pair: u8, duty: u32) {
        let r = T::regs();
        let duty = duty.min(self.reload);
        let ch_p = (pair * 2) as usize;

        r.cmp(ch_p).modify(|w| {
            w.set_cmp(duty);
            w.set_xcmp(0);
        });
    }

    /// Check if fault is active
    pub fn is_fault_active(&self) -> bool {
        T::regs().sr().read().faultf()
    }

    /// Clear fault status
    pub fn clear_fault(&mut self) {
        T::regs().gcr().modify(|w| w.set_faultclr(true));
        T::regs().gcr().modify(|w| w.set_faultclr(false));
    }
}

// =============================================================================
// Instance trait
// =============================================================================

pub(crate) trait SealedInstance {
    fn regs() -> pac::pwm::Pwm;

    /// Add PWM to resource group.
    /// PWM belongs to MOT subsystem, clock is controlled via MOT resource.
    fn add_resource_group(group: usize);

    /// Get PWM clock frequency (from AHB for MOT subsystem)
    fn frequency() -> Hertz;
}

/// PWM peripheral instance
#[allow(private_bounds)]
pub trait Instance: SealedInstance + crate::PeripheralType + 'static {}

foreach_peripheral!(
    (pwm, $inst:ident) => {
        impl SealedInstance for crate::peripherals::$inst {
            fn regs() -> pac::pwm::Pwm {
                pac::$inst
            }

            fn add_resource_group(group: usize) {
                // PWM belongs to MOT subsystem, use MOT0 resource
                crate::sysctl::clock_add_to_group(pac::resources::MOT0, group);
            }

            fn frequency() -> Hertz {
                // PWM uses AHB clock (MOT subsystem)
                crate::sysctl::clocks().ahb
            }
        }

        impl Instance for crate::peripherals::$inst {}
    };
);

