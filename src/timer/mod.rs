//! GPTMR - General Purpose Timer
//!
//! HPM MCU's GPTMR provides:
//! - 4 independent channels per timer
//! - 32-bit counter with reload
//! - 2 compare registers per channel
//! - Input capture (rising/falling/both/PWM measurement)
//! - Compare output
//! - DMA support
//! - Channel synchronization
//!
//! # Example
//!
//! ```rust,ignore
//! use hpm_hal::timer::{LowLevelTimer, Timer, Channel};
//!
//! // Low-level access
//! let timer = LowLevelTimer::new(p.GPTMR0);
//! timer.set_reload(Channel::Ch0, 1000);
//! timer.start(Channel::Ch0);
//!
//! // High-level Timer API
//! let mut timer = Timer::new(p.GPTMR0, Channel::Ch0, Default::default());
//! timer.delay_ms(100);
//! ```

use embassy_hal_internal::Peri;
use embassy_sync::waitqueue::AtomicWaker;

use crate::pac;
use crate::time::Hertz;

/// Timer channel index
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum Channel {
    Ch0 = 0,
    Ch1 = 1,
    Ch2 = 2,
    Ch3 = 3,
}

impl Channel {
    #[inline]
    pub const fn index(&self) -> usize {
        *self as usize
    }
}

/// Capture mode
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum CaptureMode {
    #[default]
    NoCapture = 0,
    Rising = 1,
    Falling = 2,
    Both = 3,
    MeasurePwm = 4,
}

impl From<CaptureMode> for pac::tmr::vals::Capmode {
    fn from(mode: CaptureMode) -> Self {
        match mode {
            CaptureMode::NoCapture => pac::tmr::vals::Capmode::NOCAP,
            CaptureMode::Rising => pac::tmr::vals::Capmode::RISING,
            CaptureMode::Falling => pac::tmr::vals::Capmode::FALLING,
            CaptureMode::Both => pac::tmr::vals::Capmode::BOTH,
            CaptureMode::MeasurePwm => pac::tmr::vals::Capmode::MEASURE_PWM,
        }
    }
}

/// DMA request event source
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum DmaEvent {
    #[default]
    Cmp0 = 0,
    Cmp1 = 1,
    Capture = 2,
    Reload = 3,
}

impl From<DmaEvent> for pac::tmr::vals::Dmasel {
    fn from(event: DmaEvent) -> Self {
        match event {
            DmaEvent::Cmp0 => pac::tmr::vals::Dmasel::CMP0,
            DmaEvent::Cmp1 => pac::tmr::vals::Dmasel::CMP1,
            DmaEvent::Capture => pac::tmr::vals::Dmasel::CAPF,
            DmaEvent::Reload => pac::tmr::vals::Dmasel::RLD,
        }
    }
}

/// Sync input edge selection
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SyncEdge {
    #[default]
    None,
    Rising,
    Falling,
    Both,
}

/// Status flags for a channel
#[derive(Clone, Copy, Debug, Default)]
pub struct ChannelStatus {
    pub reload: bool,
    pub capture: bool,
    pub cmp0: bool,
    pub cmp1: bool,
}

/// Low-level timer for direct register access.
pub struct LowLevelTimer<'d, T: Instance> {
    _peri: Peri<'d, T>,
}

impl<'d, T: Instance> LowLevelTimer<'d, T> {
    /// Create a new low-level timer without enabling clock.
    /// Use this for timers without independent clock control (like NTMR1).
    pub fn new_raw(peri: Peri<'d, T>) -> Self {
        Self { _peri: peri }
    }
}

impl<'d, T: BasicInstance> LowLevelTimer<'d, T> {
    /// Create a new low-level timer and enable its clock.
    pub fn new(peri: Peri<'d, T>) -> Self {
        T::add_resource_group(0);
        Self { _peri: peri }
    }

    #[inline]
    fn regs() -> pac::tmr::Tmr {
        T::regs()
    }

    // === Counter Control ===

    /// Enable or disable a channel's counter
    #[inline]
    pub fn enable_channel(&self, ch: Channel, enable: bool) {
        Self::regs().channel(ch.index()).cr().modify(|w| {
            w.set_cen(enable);
        });
    }

    /// Start counter (alias for enable_channel(ch, true))
    #[inline]
    pub fn start(&self, ch: Channel) {
        self.enable_channel(ch, true);
    }

    /// Stop counter (alias for enable_channel(ch, false))
    #[inline]
    pub fn stop(&self, ch: Channel) {
        self.enable_channel(ch, false);
    }

    /// Reset counter to 0
    #[inline]
    pub fn reset_counter(&self, ch: Channel) {
        let cr = Self::regs().channel(ch.index()).cr();
        cr.modify(|w| w.set_cntrst(true));
        cr.modify(|w| w.set_cntrst(false));
    }

    /// Enable debug pause (counter pauses when chip is in debug mode)
    #[inline]
    pub fn set_debug_pause(&self, ch: Channel, enable: bool) {
        Self::regs()
            .channel(ch.index())
            .cr()
            .modify(|w| w.set_dbgpause(enable));
    }

    // === Counter Access ===

    /// Get current counter value
    #[inline]
    pub fn get_counter(&self, ch: Channel) -> u32 {
        Self::regs().channel(ch.index()).cnt().read()
    }

    /// Set counter value
    #[inline]
    pub fn set_counter(&self, ch: Channel, value: u32) {
        let ch_regs = Self::regs().channel(ch.index());
        // Hardware expects value-1 for non-zero, non-max values
        let hw_value = if value > 0 && value != 0xFFFF_FFFF {
            value - 1
        } else {
            value
        };
        ch_regs.cntuptval().write_value(hw_value);
        ch_regs.cr().modify(|w| w.set_cntupt(true));
    }

    /// Set reload value
    #[inline]
    pub fn set_reload(&self, ch: Channel, reload: u32) {
        let hw_value = if reload > 0 && reload != 0xFFFF_FFFF {
            reload - 1
        } else {
            reload
        };
        Self::regs().channel(ch.index()).rld().write_value(hw_value);
    }

    /// Get reload value
    #[inline]
    pub fn get_reload(&self, ch: Channel) -> u32 {
        Self::regs().channel(ch.index()).rld().read() + 1
    }

    // === Compare ===

    /// Set compare value (cmp_idx: 0 or 1)
    #[inline]
    pub fn set_compare(&self, ch: Channel, cmp_idx: usize, value: u32) {
        assert!(cmp_idx < 2);
        let hw_value = if value > 0 && value != 0xFFFF_FFFF {
            value - 1
        } else {
            value
        };
        Self::regs()
            .channel(ch.index())
            .cmp(cmp_idx)
            .write_value(hw_value);
    }

    /// Get compare value
    #[inline]
    pub fn get_compare(&self, ch: Channel, cmp_idx: usize) -> u32 {
        assert!(cmp_idx < 2);
        Self::regs().channel(ch.index()).cmp(cmp_idx).read() + 1
    }

    /// Enable compare output
    #[inline]
    pub fn enable_compare_output(&self, ch: Channel, enable: bool) {
        Self::regs()
            .channel(ch.index())
            .cr()
            .modify(|w| w.set_cmpen(enable));
    }

    /// Set compare output initial polarity (true = high, false = low)
    #[inline]
    pub fn set_compare_initial_high(&self, ch: Channel, high: bool) {
        Self::regs()
            .channel(ch.index())
            .cr()
            .modify(|w| w.set_cmpinit(high));
    }

    // === Capture ===

    /// Set capture mode
    #[inline]
    pub fn set_capture_mode(&self, ch: Channel, mode: CaptureMode) {
        Self::regs()
            .channel(ch.index())
            .cr()
            .modify(|w| w.set_capmode(mode.into()));
    }

    /// Get captured value at rising edge
    #[inline]
    pub fn get_capture_rising(&self, ch: Channel) -> u32 {
        Self::regs().channel(ch.index()).cappos().read()
    }

    /// Get captured value at falling edge
    #[inline]
    pub fn get_capture_falling(&self, ch: Channel) -> u32 {
        Self::regs().channel(ch.index()).capneg().read()
    }

    /// Get measured period (MeasurePwm mode)
    #[inline]
    pub fn get_capture_period(&self, ch: Channel) -> u32 {
        Self::regs().channel(ch.index()).capprd().read()
    }

    /// Get measured duty cycle (MeasurePwm mode)
    #[inline]
    pub fn get_capture_duty(&self, ch: Channel) -> u32 {
        Self::regs().channel(ch.index()).capdty().read()
    }

    // === DMA ===

    /// Enable DMA request
    #[inline]
    pub fn enable_dma(&self, ch: Channel, enable: bool) {
        Self::regs()
            .channel(ch.index())
            .cr()
            .modify(|w| w.set_dmaen(enable));
    }

    /// Set DMA trigger event
    #[inline]
    pub fn set_dma_event(&self, ch: Channel, event: DmaEvent) {
        Self::regs()
            .channel(ch.index())
            .cr()
            .modify(|w| w.set_dmasel(event.into()));
    }

    // === Sync ===

    /// Set sync input edge
    #[inline]
    pub fn set_sync_edge(&self, ch: Channel, edge: SyncEdge) {
        Self::regs().channel(ch.index()).cr().modify(|w| {
            match edge {
                SyncEdge::None => {
                    w.set_synciren(false);
                    w.set_syncifen(false);
                }
                SyncEdge::Rising => {
                    w.set_synciren(true);
                    w.set_syncifen(false);
                }
                SyncEdge::Falling => {
                    w.set_synciren(false);
                    w.set_syncifen(true);
                }
                SyncEdge::Both => {
                    w.set_synciren(true);
                    w.set_syncifen(true);
                }
            }
        });
    }

    /// Enable software sync
    #[inline]
    pub fn enable_software_sync(&self, ch: Channel, enable: bool) {
        Self::regs()
            .channel(ch.index())
            .cr()
            .modify(|w| w.set_swsyncien(enable));
    }

    /// Enable sync follow (reset together with previous channel)
    /// Note: Not valid for channel 0
    #[inline]
    pub fn enable_sync_follow(&self, ch: Channel, enable: bool) {
        Self::regs()
            .channel(ch.index())
            .cr()
            .modify(|w| w.set_syncflw(enable));
    }

    /// Trigger software sync for channels (bitmask)
    #[inline]
    pub fn trigger_software_sync(&self, ch_mask: u8) {
        Self::regs().gcr().modify(|w| w.set_swsynct(ch_mask));
    }

    // === Interrupts ===

    /// Enable reload interrupt
    #[inline]
    pub fn enable_reload_irq(&self, ch: Channel, enable: bool) {
        Self::regs()
            .irqen()
            .modify(|w| w.set_chrlden(ch.index(), enable));
    }

    /// Enable capture interrupt
    #[inline]
    pub fn enable_capture_irq(&self, ch: Channel, enable: bool) {
        Self::regs()
            .irqen()
            .modify(|w| w.set_chcapen(ch.index(), enable));
    }

    /// Enable compare 0 interrupt
    #[inline]
    pub fn enable_cmp0_irq(&self, ch: Channel, enable: bool) {
        Self::regs()
            .irqen()
            .modify(|w| w.set_chcmp0en(ch.index(), enable));
    }

    /// Enable compare 1 interrupt
    #[inline]
    pub fn enable_cmp1_irq(&self, ch: Channel, enable: bool) {
        Self::regs()
            .irqen()
            .modify(|w| w.set_chcmp1en(ch.index(), enable));
    }

    /// Get raw status register
    #[inline]
    pub fn get_status_raw(&self) -> u32 {
        Self::regs().sr().read().0
    }

    /// Clear status flags (write 1 to clear)
    #[inline]
    pub fn clear_status_raw(&self, mask: u32) {
        Self::regs().sr().write_value(pac::tmr::regs::Sr(mask));
    }

    /// Get channel status
    #[inline]
    pub fn get_channel_status(&self, ch: Channel) -> ChannelStatus {
        let sr = Self::regs().sr().read();
        ChannelStatus {
            reload: sr.chrldf(ch.index()),
            capture: sr.chcapf(ch.index()),
            cmp0: sr.chcmp0f(ch.index()),
            cmp1: sr.chcmp1f(ch.index()),
        }
    }

    /// Clear channel reload flag
    #[inline]
    pub fn clear_reload_flag(&self, ch: Channel) {
        Self::regs().sr().write(|w| w.set_chrldf(ch.index(), true));
    }

    /// Clear channel capture flag
    #[inline]
    pub fn clear_capture_flag(&self, ch: Channel) {
        Self::regs().sr().write(|w| w.set_chcapf(ch.index(), true));
    }

    /// Clear channel compare 0 flag
    #[inline]
    pub fn clear_cmp0_flag(&self, ch: Channel) {
        Self::regs().sr().write(|w| w.set_chcmp0f(ch.index(), true));
    }

    /// Clear channel compare 1 flag
    #[inline]
    pub fn clear_cmp1_flag(&self, ch: Channel) {
        Self::regs().sr().write(|w| w.set_chcmp1f(ch.index(), true));
    }

    /// Check if reload flag is set
    #[inline]
    pub fn is_reload_pending(&self, ch: Channel) -> bool {
        Self::regs().sr().read().chrldf(ch.index())
    }

    /// Check if capture flag is set
    #[inline]
    pub fn is_capture_pending(&self, ch: Channel) -> bool {
        Self::regs().sr().read().chcapf(ch.index())
    }

    /// Check if compare 0 flag is set
    #[inline]
    pub fn is_cmp0_pending(&self, ch: Channel) -> bool {
        Self::regs().sr().read().chcmp0f(ch.index())
    }

    /// Check if compare 1 flag is set
    #[inline]
    pub fn is_cmp1_pending(&self, ch: Channel) -> bool {
        Self::regs().sr().read().chcmp1f(ch.index())
    }

    // === Extended Features (conditional) ===

    /// Enable one-shot mode (timer stops at reload)
    #[cfg(any(tmr_v68, tmr_v6e))]
    #[inline]
    pub fn enable_oneshot(&self, ch: Channel, enable: bool) {
        Self::regs()
            .channel(ch.index())
            .cr()
            .modify(|w| w.set_opmode(enable));
    }

    /// Enable signal monitor
    #[cfg(any(tmr_v68, tmr_v6e))]
    #[inline]
    pub fn enable_monitor(&self, ch: Channel, enable: bool) {
        Self::regs()
            .channel(ch.index())
            .cr()
            .modify(|w| w.set_monitor_en(enable));
    }

    /// Set monitor type (true = high level time, false = period)
    #[cfg(any(tmr_v68, tmr_v6e))]
    #[inline]
    pub fn set_monitor_high_level(&self, ch: Channel, high_level: bool) {
        Self::regs()
            .channel(ch.index())
            .cr()
            .modify(|w| w.set_monitor_sel(high_level));
    }

    /// Set counter mode (internal clock or external input)
    #[cfg(tmr_v6e)]
    #[inline]
    pub fn set_external_count_mode(&self, ch: Channel, external: bool) {
        Self::regs()
            .channel(ch.index())
            .cr()
            .modify(|w| w.set_cnt_mode(external));
    }
}

// === Timer configuration ===

/// Timer channel configuration.
#[derive(Clone)]
pub struct Config {
    /// Reload value (counter period)
    pub reload: u32,
    /// Pause counter in debug mode
    pub debug_pause: bool,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            reload: u32::MAX,
            debug_pause: true,
        }
    }
}

// === High-level Timer ===

/// Basic timer for a single channel.
///
/// Provides simple timing functionality with blocking delay support.
pub struct Timer<'d, T: BasicInstance> {
    inner: LowLevelTimer<'d, T>,
    channel: Channel,
}

impl<'d, T: BasicInstance> Timer<'d, T> {
    /// Create a new timer on the specified channel.
    pub fn new(peri: Peri<'d, T>, channel: Channel, config: Config) -> Self {
        let inner = LowLevelTimer::new(peri);
        inner.set_reload(channel, config.reload);
        inner.set_debug_pause(channel, config.debug_pause);
        inner.reset_counter(channel);
        Self { inner, channel }
    }

    /// Get timer input frequency.
    #[inline]
    pub fn input_frequency(&self) -> Hertz {
        T::frequency()
    }

    /// Start the timer.
    #[inline]
    pub fn start(&mut self) {
        self.inner.start(self.channel);
    }

    /// Stop the timer.
    #[inline]
    pub fn stop(&mut self) {
        self.inner.stop(self.channel);
    }

    /// Reset counter to 0.
    #[inline]
    pub fn reset(&mut self) {
        self.inner.reset_counter(self.channel);
    }

    /// Get current counter value.
    #[inline]
    pub fn counter(&self) -> u32 {
        self.inner.get_counter(self.channel)
    }

    /// Set reload value.
    #[inline]
    pub fn set_reload(&mut self, reload: u32) {
        self.inner.set_reload(self.channel, reload);
    }

    /// Get reload value.
    #[inline]
    pub fn reload(&self) -> u32 {
        self.inner.get_reload(self.channel)
    }

    /// Check if overflow (reload) occurred.
    #[inline]
    pub fn is_overflow(&self) -> bool {
        self.inner.is_reload_pending(self.channel)
    }

    /// Clear overflow flag.
    #[inline]
    pub fn clear_overflow(&mut self) {
        self.inner.clear_reload_flag(self.channel);
    }

    /// Blocking wait for overflow.
    pub fn wait_overflow(&mut self) {
        while !self.is_overflow() {}
        self.clear_overflow();
    }

    /// Blocking delay for specified number of ticks.
    pub fn delay_ticks(&mut self, ticks: u32) {
        self.stop();
        self.set_reload(ticks);
        self.reset();
        self.clear_overflow();
        self.start();
        self.wait_overflow();
        self.stop();
    }
}

impl<T: BasicInstance> embedded_hal::delay::DelayNs for Timer<'_, T> {
    fn delay_ns(&mut self, ns: u32) {
        let freq = self.input_frequency().0;
        // ticks = ns * freq / 1_000_000_000
        // Use u64 to avoid overflow
        let ticks = (ns as u64 * freq as u64 / 1_000_000_000) as u32;
        if ticks > 0 {
            self.delay_ticks(ticks.max(1));
        }
    }
}

// === Input Capture ===

/// Input capture configuration.
#[derive(Clone)]
pub struct InputCaptureConfig {
    /// Capture mode
    pub mode: CaptureMode,
    /// Reload value (max capture range)
    pub reload: u32,
    /// Pause in debug mode
    pub debug_pause: bool,
}

impl Default for InputCaptureConfig {
    fn default() -> Self {
        Self {
            mode: CaptureMode::Rising,
            reload: u32::MAX,
            debug_pause: true,
        }
    }
}

/// Input capture driver.
///
/// Captures timer counter value on input signal edges.
pub struct InputCapture<'d, T: BasicInstance> {
    inner: LowLevelTimer<'d, T>,
    channel: Channel,
}

impl<'d, T: BasicInstance> InputCapture<'d, T> {
    /// Create input capture (signal from TRGM, no pin needed).
    pub fn new(peri: Peri<'d, T>, channel: Channel, config: InputCaptureConfig) -> Self {
        let inner = LowLevelTimer::new(peri);
        inner.set_reload(channel, config.reload);
        inner.set_debug_pause(channel, config.debug_pause);
        inner.set_capture_mode(channel, config.mode);
        inner.reset_counter(channel);
        inner.start(channel);
        Self { inner, channel }
    }

    /// Get timer input frequency.
    #[inline]
    pub fn input_frequency(&self) -> Hertz {
        T::frequency()
    }

    /// Get captured value at rising edge.
    #[inline]
    pub fn capture_rising(&self) -> u32 {
        self.inner.get_capture_rising(self.channel)
    }

    /// Get captured value at falling edge.
    #[inline]
    pub fn capture_falling(&self) -> u32 {
        self.inner.get_capture_falling(self.channel)
    }

    /// Get measured period (MeasurePwm mode).
    #[inline]
    pub fn measured_period(&self) -> u32 {
        self.inner.get_capture_period(self.channel)
    }

    /// Get measured duty cycle ticks (MeasurePwm mode).
    #[inline]
    pub fn measured_duty(&self) -> u32 {
        self.inner.get_capture_duty(self.channel)
    }

    /// Check if capture event occurred.
    #[inline]
    pub fn is_captured(&self) -> bool {
        self.inner.is_capture_pending(self.channel)
    }

    /// Clear capture flag.
    #[inline]
    pub fn clear_capture(&mut self) {
        self.inner.clear_capture_flag(self.channel);
    }

    /// Blocking wait for capture event.
    pub fn wait_capture(&mut self) {
        while !self.is_captured() {}
        self.clear_capture();
    }

    /// Set capture mode.
    #[inline]
    pub fn set_mode(&mut self, mode: CaptureMode) {
        self.inner.set_capture_mode(self.channel, mode);
    }

    /// Async wait for capture event (requires interrupt binding).
    pub async fn wait_capture_async(&mut self)
    where
        T: InterruptInstance,
    {
        use core::future::poll_fn;
        use core::task::Poll;

        self.inner.enable_capture_irq(self.channel, true);

        poll_fn(|cx| {
            T::state().wakers[self.channel.index()].register(cx.waker());

            if self.is_captured() {
                self.clear_capture();
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        })
        .await;

        self.inner.enable_capture_irq(self.channel, false);
    }

    /// Async get captured rising edge value.
    pub async fn async_capture_rising(&mut self) -> u32
    where
        T: InterruptInstance,
    {
        self.set_mode(CaptureMode::Rising);
        self.wait_capture_async().await;
        self.capture_rising()
    }

    /// Async get captured falling edge value.
    pub async fn async_capture_falling(&mut self) -> u32
    where
        T: InterruptInstance,
    {
        self.set_mode(CaptureMode::Falling);
        self.wait_capture_async().await;
        self.capture_falling()
    }
}

// === Compare Output ===

/// Compare output configuration.
#[derive(Clone)]
pub struct CompareOutputConfig {
    /// Reload value (period)
    pub reload: u32,
    /// Compare 0 value
    pub cmp0: u32,
    /// Compare 1 value (optional)
    pub cmp1: Option<u32>,
    /// Initial output level is high
    pub initial_high: bool,
    /// Pause in debug mode
    pub debug_pause: bool,
}

impl Default for CompareOutputConfig {
    fn default() -> Self {
        Self {
            reload: u32::MAX,
            cmp0: u32::MAX / 2,
            cmp1: None,
            initial_high: true,
            debug_pause: true,
        }
    }
}

/// Compare output driver.
///
/// Generates output signal based on counter compare values.
pub struct CompareOutput<'d, T: BasicInstance> {
    inner: LowLevelTimer<'d, T>,
    channel: Channel,
}

impl<'d, T: BasicInstance> CompareOutput<'d, T> {
    /// Create compare output (signal to TRGM, no pin needed).
    pub fn new(peri: Peri<'d, T>, channel: Channel, config: CompareOutputConfig) -> Self {
        let inner = LowLevelTimer::new(peri);
        inner.set_reload(channel, config.reload);
        inner.set_compare(channel, 0, config.cmp0);
        if let Some(cmp1) = config.cmp1 {
            inner.set_compare(channel, 1, cmp1);
        }
        inner.set_compare_initial_high(channel, config.initial_high);
        inner.set_debug_pause(channel, config.debug_pause);
        inner.enable_compare_output(channel, true);
        inner.reset_counter(channel);
        Self { inner, channel }
    }

    /// Get timer input frequency.
    #[inline]
    pub fn input_frequency(&self) -> Hertz {
        T::frequency()
    }

    /// Start output.
    #[inline]
    pub fn start(&mut self) {
        self.inner.start(self.channel);
    }

    /// Stop output.
    #[inline]
    pub fn stop(&mut self) {
        self.inner.stop(self.channel);
    }

    /// Enable or disable output.
    #[inline]
    pub fn enable(&mut self, enable: bool) {
        self.inner.enable_compare_output(self.channel, enable);
    }

    /// Set reload value (period).
    #[inline]
    pub fn set_reload(&mut self, reload: u32) {
        self.inner.set_reload(self.channel, reload);
    }

    /// Set compare value.
    #[inline]
    pub fn set_compare(&mut self, cmp_idx: usize, value: u32) {
        self.inner.set_compare(self.channel, cmp_idx, value);
    }

    /// Set initial output level.
    #[inline]
    pub fn set_initial_high(&mut self, high: bool) {
        self.inner.set_compare_initial_high(self.channel, high);
    }

    /// Set output frequency (adjusts reload), returns actual frequency.
    pub fn set_frequency(&mut self, freq: Hertz) -> Hertz {
        let timer_freq = T::frequency().0;
        let reload = timer_freq / freq.0;
        let reload = reload.max(1);
        self.set_reload(reload);
        Hertz(timer_freq / reload)
    }

    /// Set duty cycle (0-100%).
    pub fn set_duty_percent(&mut self, percent: u8) {
        let reload = self.inner.get_reload(self.channel);
        let cmp = (reload as u64 * percent as u64 / 100) as u32;
        self.set_compare(0, cmp);
    }
}

// === Interrupt Handler ===

use core::marker::PhantomData;

/// Interrupt handler for timer.
pub struct InterruptHandler<T: Instance> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> crate::interrupt::typelevel::Handler<T::Interrupt> for InterruptHandler<T>
where
    T: Instance + InterruptInstance,
{
    unsafe fn on_interrupt() {
        let regs = T::regs();
        let sr = regs.sr().read().0;

        // Clear all pending flags (write 1 to clear)
        regs.sr().write_value(pac::tmr::regs::Sr(sr));

        // Wake up tasks for each channel
        let state = T::state();
        for ch in 0..4 {
            // Each channel has 4 bits of status: reload, capture, cmp0, cmp1
            if (sr >> (ch * 4)) & 0x0F != 0 {
                state.wakers[ch].wake();
            }
        }
    }
}

/// Timer instance with interrupt support.
#[allow(private_bounds)]
pub trait InterruptInstance: Instance {
    type Interrupt: crate::interrupt::typelevel::Interrupt;
}

foreach_interrupt!(
    ($inst:ident, tmr, TMR, GLOBAL, $irq:ident) => {
        impl InterruptInstance for crate::peripherals::$inst {
            type Interrupt = crate::interrupt::typelevel::$irq;
        }
    };
);

// === Instance trait ===

pub(crate) struct State {
    wakers: [AtomicWaker; 4],
}

impl State {
    const fn new() -> Self {
        Self {
            wakers: [const { AtomicWaker::new() }; 4],
        }
    }
}

pub(crate) trait SealedInstance {
    fn regs() -> pac::tmr::Tmr;
    fn state() -> &'static State;
}

/// Timer peripheral instance.
///
/// Note: Some timer instances (like NTMR1) may not have independent clock control.
/// Use `BasicInstance` if you need clock peripheral functionality.
#[allow(private_bounds)]
pub trait Instance: SealedInstance + crate::PeripheralType + 'static {}

/// Timer instance with clock control.
#[allow(private_bounds)]
pub trait BasicInstance: Instance + crate::sysctl::ClockPeripheral {}

foreach_peripheral!(
    (tmr, $inst:ident) => {
        impl SealedInstance for crate::peripherals::$inst {
            fn regs() -> pac::tmr::Tmr {
                pac::$inst
            }
            fn state() -> &'static State {
                static STATE: State = State::new();
                &STATE
            }
        }

        impl Instance for crate::peripherals::$inst {}
    };
);

// Blanket implementation: any Instance with ClockPeripheral is a BasicInstance
impl<T: Instance + crate::sysctl::ClockPeripheral> BasicInstance for T {}

