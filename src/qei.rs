//! Quadrature Encoder Interface.
//!
// QEIv1 does not have any physical pins. All signals come from TRGM.

use embassy_hal_internal::{Peri, PeripheralType};

use crate::gpio::AnyPin;
use crate::pac;

/// Quadrature Encoder Interface driver.
pub struct Qei<'d, T: Instance + PeripheralType> {
    _peri: Peri<'d, T>,
    _a: Peri<'d, AnyPin>,
    _b: Peri<'d, AnyPin>,
    _z: Option<Peri<'d, AnyPin>>,
    _fault: Option<Peri<'d, AnyPin>>,
    _home0: Option<Peri<'d, AnyPin>>,
    _home1: Option<Peri<'d, AnyPin>>,
}

impl<'d, T: Instance + PeripheralType> Qei<'d, T> {
    /// Create a new QEI driver with only A and B pins (most common configuration).
    pub fn new(peri: Peri<'d, T>, a: Peri<'d, impl APin<T>>, b: Peri<'d, impl BPin<T>>) -> Self {
        // Configure pins before converting to AnyPin
        a.set_as_alt(a.alt_num());
        b.set_as_alt(b.alt_num());

        Self::new_inner(peri, a.into(), b.into(), None, None, None, None)
    }

    /// Create a new QEI driver with A, B, and Z (index) pins.
    pub fn new_with_z(
        peri: Peri<'d, T>,
        a: Peri<'d, impl APin<T>>,
        b: Peri<'d, impl BPin<T>>,
        z: Peri<'d, impl ZPin<T>>,
    ) -> Self {
        // Configure pins before converting to AnyPin
        a.set_as_alt(a.alt_num());
        b.set_as_alt(b.alt_num());
        z.set_as_alt(z.alt_num());

        Self::new_inner(peri, a.into(), b.into(), Some(z.into()), None, None, None)
    }

    /// Create a new QEI driver with all pins configured.
    pub fn new_full(
        peri: Peri<'d, T>,
        a: Peri<'d, impl APin<T>>,
        b: Peri<'d, impl BPin<T>>,
        z: Peri<'d, impl ZPin<T>>,
        fault: Peri<'d, impl FaultPin<T>>,
        home0: Peri<'d, impl Home0Pin<T>>,
        home1: Peri<'d, impl Home1Pin<T>>,
    ) -> Self {
        // Configure pins before converting to AnyPin
        a.set_as_alt(a.alt_num());
        b.set_as_alt(b.alt_num());
        z.set_as_alt(z.alt_num());
        fault.set_as_alt(fault.alt_num());
        home0.set_as_alt(home0.alt_num());
        home1.set_as_alt(home1.alt_num());

        Self::new_inner(
            peri,
            a.into(),
            b.into(),
            Some(z.into()),
            Some(fault.into()),
            Some(home0.into()),
            Some(home1.into()),
        )
    }

    /// Internal constructor - pins must already be configured before calling.
    fn new_inner(
        peri: Peri<'d, T>,
        a: Peri<'d, AnyPin>,
        b: Peri<'d, AnyPin>,
        z: Option<Peri<'d, AnyPin>>,
        fault: Option<Peri<'d, AnyPin>>,
        home0: Option<Peri<'d, AnyPin>>,
        home1: Option<Peri<'d, AnyPin>>,
    ) -> Self {
        T::add_resource_group(0);

        Self {
            _peri: peri,
            _a: a,
            _b: b,
            _z: z,
            _fault: fault,
            _home0: home0,
            _home1: home1,
        }
    }

    /// Get the raw QEI registers.
    pub fn regs(&self) -> pac::qei::Qei {
        T::REGS
    }
}

pub(crate) trait SealedInstance {
    const REGS: crate::pac::qei::Qei;
}

#[allow(private_bounds)]
pub trait Instance: SealedInstance + crate::sysctl::ClockPeripheral + 'static {
    /// Interrupt for this peripheral.
    type Interrupt: crate::interrupt::typelevel::Interrupt;
}

pin_trait!(APin, Instance);
pin_trait!(BPin, Instance);
pin_trait!(ZPin, Instance);
pin_trait!(FaultPin, Instance);
pin_trait!(Home0Pin, Instance);
pin_trait!(Home1Pin, Instance);

foreach_peripheral!(
    (qei, $inst:ident) => {
        impl SealedInstance for crate::peripherals::$inst {
            const REGS: crate::pac::qei::Qei = crate::pac::$inst;
        }

        impl Instance for crate::peripherals::$inst {
            type Interrupt = crate::interrupt::typelevel::$inst;
        }
    };
);
