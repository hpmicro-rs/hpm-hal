//! Quadrature Encoder Interface.
//!
// QEIv1 does not have any physical pins. All signals come from TRGM.

use embassy_hal_internal::{Peri, PeripheralType};

use crate::pac;

#[allow(unused)]
pub struct Qei<'d, T: Instance + PeripheralType> {
    _peri: Peri<'d, T>,
}

impl<'d, T: Instance + PeripheralType> Qei<'d, T> {
    pub fn new_uninited(
        peri: Peri<'d, T>,
        a: Peri<'d, impl APin<T>>,
        b: Peri<'d, impl BPin<T>>,
        z: Peri<'d, impl ZPin<T>>,
        fault: Peri<'d, impl FaultPin<T>>,
        home0: Peri<'d, impl Home0Pin<T>>,
        home1: Peri<'d, impl Home1Pin<T>>,
    ) -> Qei<'d, T> {
        T::add_resource_group(0);

        a.set_as_alt(a.alt_num());
        b.set_as_alt(b.alt_num());
        z.set_as_alt(z.alt_num());
        fault.set_as_alt(fault.alt_num());
        home0.set_as_alt(home0.alt_num());
        home1.set_as_alt(home1.alt_num());

        Qei { _peri: peri }
    }

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

// APin is not optional.

impl<T: Instance> BPin<T> for crate::gpio::NoPin {
    fn alt_num(&self) -> u8 {
        0
    }
}
impl<T: Instance> ZPin<T> for crate::gpio::NoPin {
    fn alt_num(&self) -> u8 {
        0
    }
}
impl<T: Instance> FaultPin<T> for crate::gpio::NoPin {
    fn alt_num(&self) -> u8 {
        0
    }
}
impl<T: Instance> Home0Pin<T> for crate::gpio::NoPin {
    fn alt_num(&self) -> u8 {
        0
    }
}
impl<T: Instance> Home1Pin<T> for crate::gpio::NoPin {
    fn alt_num(&self) -> u8 {
        0
    }
}

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
