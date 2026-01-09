//! SDXC (SD/MMC Card Interface) driver
//!
//! This driver supports both SD cards and eMMC devices, with both Async and Blocking modes.
//!
//! # Features
//!
//! - SD/SDHC/SDXC card support (up to 2TB)
//! - eMMC 4.5+ support
//! - 1-bit, 4-bit, and 8-bit bus widths
//! - Async and Blocking modes
//! - DMA transfers (ADMA2)
//! - High speed modes (up to SDR50)
//!
//! # Examples
//!
//! ## Async mode (recommended)
//!
//! ```no_run
//! use hpm_hal::sdxc;
//!
//! let mut sdxc = sdxc::Sdxc::new_4bit(
//!     p.SDXC0, Irqs, p.HDMA_CH0,
//!     p.PA00, p.PA01, p.PA02, p.PA03, p.PA04, p.PA05,
//!     Default::default()
//! );
//!
//! sdxc.init_sd_card(hz(25_000_000)).await?;
//!
//! let mut block = sdxc::DataBlock::new();
//! sdxc.read_block(0, &mut block).await?;
//! ```
//!
//! ## Blocking mode
//!
//! ```no_run
//! let mut sdxc = sdxc::Sdxc::new_blocking_4bit(
//!     p.SDXC0,
//!     p.PA00, p.PA01, p.PA02, p.PA03, p.PA04, p.PA05,
//!     Default::default()
//! );
//!
//! sdxc.blocking_init_sd_card(hz(25_000_000))?;
//!
//! let mut block = sdxc::DataBlock::new();
//! sdxc.blocking_read_block(0, &mut block)?;
//! ```

use core::future::poll_fn;
use core::marker::PhantomData;
use core::task::Poll;

use embassy_hal_internal::Peri;
use embassy_sync::waitqueue::AtomicWaker;

use crate::dma::ChannelAndRequest;
use crate::gpio::{AnyPin, SealedPin as _};
use crate::mode::{Async, Blocking, Mode};
use crate::pac;
use crate::sysctl::SealedClockPeripheral;
use crate::time::Hertz;
use crate::interrupt;

mod instance;
mod types;

pub use instance::{ClkPin, CmdPin, D0Pin, D1Pin, D2Pin, D3Pin, D4Pin, D5Pin, D6Pin, D7Pin, Instance, SdxcDma};
pub use types::*;

use instance::{Info, SealedInstance, State};

/// SDXC interrupt handler
pub struct InterruptHandler<T: Instance> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> interrupt::typelevel::Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        let info = T::info();
        let state = T::state();
        let regs = info.regs;

        let status = regs.int_stat().read();

        // Save error status
        if status.0 & 0xFFFF_8000 != 0 {
            // Any error bit set
            state.set_error(status.0);
        }

        // Clear interrupt flags
        regs.int_stat().write(|w| w.0 = status.0);

        // Disable interrupts that have fired
        regs.int_stat_en().modify(|w| {
            if status.cmd_complete() {
                w.set_cmd_complete(false);
            }
            if status.xfer_complete() {
                w.set_xfer_complete(false);
            }
            if status.dma_interrupt() {
                w.set_dma_interrupt(false);
            }
        });

        // Wake waiting task
        state.wake();
    }
}

/// SDXC driver
pub struct Sdxc<'d, M: Mode> {
    info: &'static Info,
    state: &'static State,
    kernel_clock: Hertz,

    clk: Peri<'d, AnyPin>,
    cmd: Peri<'d, AnyPin>,
    d0: Peri<'d, AnyPin>,
    d1: Option<Peri<'d, AnyPin>>,
    d2: Option<Peri<'d, AnyPin>>,
    d3: Option<Peri<'d, AnyPin>>,
    d4: Option<Peri<'d, AnyPin>>,
    d5: Option<Peri<'d, AnyPin>>,
    d6: Option<Peri<'d, AnyPin>>,
    d7: Option<Peri<'d, AnyPin>>,

    dma: Option<ChannelAndRequest<'d>>,

    card: Option<Card>,

    _phantom: PhantomData<M>,
}

impl<'d, M: Mode> Sdxc<'d, M> {
    fn new_inner<T: Instance>(
        _peri: Peri<'d, T>,
        clk: Peri<'d, AnyPin>,
        cmd: Peri<'d, AnyPin>,
        d0: Peri<'d, AnyPin>,
        d1: Option<Peri<'d, AnyPin>>,
        d2: Option<Peri<'d, AnyPin>>,
        d3: Option<Peri<'d, AnyPin>>,
        d4: Option<Peri<'d, AnyPin>>,
        d5: Option<Peri<'d, AnyPin>>,
        d6: Option<Peri<'d, AnyPin>>,
        d7: Option<Peri<'d, AnyPin>>,
        dma: Option<ChannelAndRequest<'d>>,
        _config: Config,
    ) -> Self {
        // Enable peripheral clock
        T::enable_clock();
        T::add_resource_group(0);

        let info = T::info();
        let state = T::state();
        let regs = info.regs;

        // Configure pins as alternate function
        clk.set_as_alt(clk.alt_num());
        cmd.set_as_alt(cmd.alt_num());
        d0.set_as_alt(d0.alt_num());
        if let Some(ref pin) = d1 {
            pin.set_as_alt(pin.alt_num());
        }
        if let Some(ref pin) = d2 {
            pin.set_as_alt(pin.alt_num());
        }
        if let Some(ref pin) = d3 {
            pin.set_as_alt(pin.alt_num());
        }
        if let Some(ref pin) = d4 {
            pin.set_as_alt(pin.alt_num());
        }
        if let Some(ref pin) = d5 {
            pin.set_as_alt(pin.alt_num());
        }
        if let Some(ref pin) = d6 {
            pin.set_as_alt(pin.alt_num());
        }
        if let Some(ref pin) = d7 {
            pin.set_as_alt(pin.alt_num());
        }

        // Reset controller
        regs.sys_ctrl().write(|w| w.set_sw_rst_all(true));
        while regs.sys_ctrl().read().sw_rst_all() {}

        // Configure timeout (longest timeout)
        regs.timeout_ctrl()
            .write(|w| w.set_data_timeout_counter_value(0x0E));

        // Disable all interrupts initially
        regs.int_stat_en().write(|_| {});
        regs.int_signal_en().write(|_| {});

        let kernel_clock = T::frequency();

        Self {
            info,
            state,
            kernel_clock,
            clk,
            cmd,
            d0,
            d1,
            d2,
            d3,
            d4,
            d5,
            d6,
            d7,
            dma,
            card: None,
            _phantom: PhantomData,
        }
    }

    /// Get the card information, if initialized
    pub fn card(&self) -> Option<&Card> {
        self.card.as_ref()
    }

    /// Check if a card is inserted (software detection)
    pub fn is_card_inserted(&self) -> bool {
        let regs = self.info.regs;
        regs.pstate().read().card_inserted()
    }
}

// Blocking mode constructors
impl<'d> Sdxc<'d, Blocking> {
    /// Create a new blocking SDXC driver with 1-bit bus width
    pub fn new_blocking_1bit<T: Instance>(
        peri: Peri<'d, T>,
        clk: Peri<'d, impl ClkPin<T>>,
        cmd: Peri<'d, impl CmdPin<T>>,
        d0: Peri<'d, impl D0Pin<T>>,
        config: Config,
    ) -> Self {
        Self::new_inner(
            peri,
            clk.into(),
            cmd.into(),
            d0.into(),
            None,
            None,
            None,
            None,
            None,
            None,
            None,
            None,
            config,
        )
    }

    /// Create a new blocking SDXC driver with 4-bit bus width
    pub fn new_blocking_4bit<T: Instance>(
        peri: Peri<'d, T>,
        clk: Peri<'d, impl ClkPin<T>>,
        cmd: Peri<'d, impl CmdPin<T>>,
        d0: Peri<'d, impl D0Pin<T>>,
        d1: Peri<'d, impl D1Pin<T>>,
        d2: Peri<'d, impl D2Pin<T>>,
        d3: Peri<'d, impl D3Pin<T>>,
        config: Config,
    ) -> Self {
        Self::new_inner(
            peri,
            clk.into(),
            cmd.into(),
            d0.into(),
            Some(d1.into()),
            Some(d2.into()),
            Some(d3.into()),
            None,
            None,
            None,
            None,
            None,
            config,
        )
    }
}

// Async mode constructors
impl<'d> Sdxc<'d, Async> {
    /// Create a new async SDXC driver with 1-bit bus width
    pub fn new_1bit<T: Instance>(
        peri: Peri<'d, T>,
        _irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        dma: Peri<'d, impl SdxcDma<T>>,
        clk: Peri<'d, impl ClkPin<T>>,
        cmd: Peri<'d, impl CmdPin<T>>,
        d0: Peri<'d, impl D0Pin<T>>,
        config: Config,
    ) -> Self {
        T::Interrupt::unpend();
        unsafe { T::Interrupt::enable() };

        Self::new_inner(
            peri,
            clk.into(),
            cmd.into(),
            d0.into(),
            None,
            None,
            None,
            None,
            None,
            None,
            None,
            Some(dma.into()),
            config,
        )
    }

    /// Create a new async SDXC driver with 4-bit bus width
    pub fn new_4bit<T: Instance>(
        peri: Peri<'d, T>,
        _irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        dma: Peri<'d, impl SdxcDma<T>>,
        clk: Peri<'d, impl ClkPin<T>>,
        cmd: Peri<'d, impl CmdPin<T>>,
        d0: Peri<'d, impl D0Pin<T>>,
        d1: Peri<'d, impl D1Pin<T>>,
        d2: Peri<'d, impl D2Pin<T>>,
        d3: Peri<'d, impl D3Pin<T>>,
        config: Config,
    ) -> Self {
        T::Interrupt::unpend();
        unsafe { T::Interrupt::enable() };

        Self::new_inner(
            peri,
            clk.into(),
            cmd.into(),
            d0.into(),
            Some(d1.into()),
            Some(d2.into()),
            Some(d3.into()),
            None,
            None,
            None,
            None,
            Some(dma.into()),
            config,
        )
    }
}

impl<'d, M: Mode> Drop for Sdxc<'d, M> {
    fn drop(&mut self) {
        // Disable interrupts
        let regs = self.info.regs;
        regs.int_stat_en().write(|_| {});
        regs.int_signal_en().write(|_| {});

        // Reset controller
        regs.sys_ctrl().write(|w| w.set_software_reset_all(true));
    }
}
