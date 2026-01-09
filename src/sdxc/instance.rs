//! SDXC instance trait and implementation

use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};

use embassy_hal_internal::{Peri, PeripheralType};
use embassy_sync::waitqueue::AtomicWaker;

use crate::interrupt;
use crate::pac;
use crate::sysctl::SealedClockPeripheral;

/// SDXC instance state
pub(crate) struct State {
    pub(crate) waker: AtomicWaker,
    pub(crate) last_error: AtomicU32,
    pub(crate) card_present: AtomicBool,
}

impl State {
    pub(crate) const fn new() -> Self {
        Self {
            waker: AtomicWaker::new(),
            last_error: AtomicU32::new(0),
            card_present: AtomicBool::new(false),
        }
    }

    pub(crate) fn wake(&self) {
        self.waker.wake();
    }

    pub(crate) fn set_error(&self, error: u32) {
        self.last_error.store(error, Ordering::Release);
    }

    pub(crate) fn get_error(&self) -> u32 {
        self.last_error.load(Ordering::Acquire)
    }

    pub(crate) fn clear_error(&self) {
        self.last_error.store(0, Ordering::Release);
    }
}

/// SDXC instance information
pub(crate) struct Info {
    pub(crate) regs: pac::sdxc::Sdxc,
    pub(crate) interrupt: pac::Interrupt,
}

pub(crate) trait SealedInstance {
    fn info() -> &'static Info;
    fn state() -> &'static State;
}

/// SDXC instance trait
#[allow(private_bounds)]
pub trait Instance: SealedInstance + SealedClockPeripheral + PeripheralType + 'static {
    /// Interrupt type for this instance
    type Interrupt: interrupt::typelevel::Interrupt;
}

#[cfg(sdxc)]
impl SealedInstance for crate::peripherals::SDXC0 {
    fn info() -> &'static Info {
        static INFO: Info = Info {
            regs: pac::SDXC0,
            interrupt: pac::Interrupt::SDXC0,
        };
        &INFO
    }

    fn state() -> &'static State {
        static STATE: State = State::new();
        &STATE
    }
}

#[cfg(sdxc)]
impl Instance for crate::peripherals::SDXC0 {
    type Interrupt = crate::interrupt::typelevel::SDXC0;
}

#[cfg(sdxc)]
impl SealedInstance for crate::peripherals::SDXC1 {
    fn info() -> &'static Info {
        static INFO: Info = Info {
            regs: pac::SDXC1,
            interrupt: pac::Interrupt::SDXC1,
        };
        &INFO
    }

    fn state() -> &'static State {
        static STATE: State = State::new();
        &STATE
    }
}

#[cfg(sdxc)]
impl Instance for crate::peripherals::SDXC1 {
    type Interrupt = crate::interrupt::typelevel::SDXC1;
}

// Pin traits
pub(crate) trait SealedPin {}

/// SDXC pin trait
#[allow(private_bounds)]
pub trait SdxcPin<T: Instance>: SealedPin + crate::gpio::Pin {}

macro_rules! impl_pin {
    ($inst:ident, $pin:ident, $func:ident) => {
        impl SealedPin for crate::peripherals::$pin {}
        impl SdxcPin<crate::peripherals::$inst> for crate::peripherals::$pin {}
    };
}

/// CLK pin
pub trait ClkPin<T: Instance>: SdxcPin<T> {}
/// CMD pin
pub trait CmdPin<T: Instance>: SdxcPin<T> {}
/// D0 pin
pub trait D0Pin<T: Instance>: SdxcPin<T> {}
/// D1 pin
pub trait D1Pin<T: Instance>: SdxcPin<T> {}
/// D2 pin
pub trait D2Pin<T: Instance>: SdxcPin<T> {}
/// D3 pin
pub trait D3Pin<T: Instance>: SdxcPin<T> {}
/// D4 pin (eMMC only)
pub trait D4Pin<T: Instance>: SdxcPin<T> {}
/// D5 pin (eMMC only)
pub trait D5Pin<T: Instance>: SdxcPin<T> {}
/// D6 pin (eMMC only)
pub trait D6Pin<T: Instance>: SdxcPin<T> {}
/// D7 pin (eMMC only)
pub trait D7Pin<T: Instance>: SdxcPin<T> {}

// DMA support
pub(crate) trait SealedDmaChannel {}

/// SDXC DMA channel trait
#[allow(private_bounds)]
pub trait SdxcDma<T: Instance>: SealedDmaChannel + crate::dma::Channel {}

// DMA channel implementations
// For now, manually implement for common HDMA channels
// TODO: Generate this automatically in build.rs

impl SealedDmaChannel for crate::peripherals::HDMA_CH0 {}
impl SdxcDma<crate::peripherals::SDXC0> for crate::peripherals::HDMA_CH0 {}
impl SdxcDma<crate::peripherals::SDXC1> for crate::peripherals::HDMA_CH0 {}
