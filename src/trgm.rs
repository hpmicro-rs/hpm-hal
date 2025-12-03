//! TRGM, Trigger Manager, Trigger Mux
//!
//! - MUX matrix
//! - Multiple input & output sources
//! - Input filtering
//! - Invetion, edge to pluse convertion
//! - DMA request generation: PWMT, QDEC, HALL

use embassy_hal_internal::{Peri, PeripheralType};

use crate::pac;

#[allow(unused)]
pub struct Trgm<'d, T: Instance + PeripheralType> {
    _peri: Peri<'d, T>,
}

impl<'d, T: Instance + PeripheralType> Trgm<'d, T> {
    pub fn new_uninited(peri: Peri<'d, T>) -> Trgm<'d, T> {
        Trgm { _peri: peri }
    }

    pub fn regs(&self) -> pac::trgm::Trgm {
        T::REGS
    }
}

pub(crate) trait SealedInstance {
    const REGS: crate::pac::trgm::Trgm;
}

#[allow(private_bounds)]
pub trait Instance: SealedInstance + 'static {}
