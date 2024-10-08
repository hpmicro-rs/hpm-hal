#[cfg(hpm53)]
#[path = "v53.rs"]
mod sysctl_impl;

#[cfg(hpm6e)]
#[path = "v6e.rs"]
mod sysctl_impl;

#[cfg(hpm67)]
#[path = "v67.rs"]
mod sysctl_impl;

#[cfg(hpm63)]
#[path = "v63.rs"]
mod sysctl_impl;

#[cfg(hpm62)]
#[path = "v62.rs"]
mod sysctl_impl;

#[cfg(hpm68)]
#[path = "v68.rs"]
mod sysctl_impl;

mod pll;

use core::ptr::addr_of;

pub use pll::*;
pub use sysctl_impl::*;

use crate::pac::SYSCTL;
use crate::time::Hertz;

/// System clock srouce
pub fn clocks() -> &'static Clocks {
    unsafe { &*addr_of!(sysctl_impl::CLOCKS) }
}

/// Add clock resource to a resource group
pub fn clock_add_to_group(resource: usize, group: usize) {
    const RESOURCE_START: usize = 256;
    if resource < RESOURCE_START || resource == usize::MAX {
        return;
    }
    let index = (resource - RESOURCE_START) / 32;
    let offset = (resource - RESOURCE_START) % 32;

    if group == 0 {
        if SYSCTL.group0(index).value().read().link() & (1 << offset) != 0 {
            return;
        }
        SYSCTL.group0(index).set().write(|w| w.set_link(1 << offset));
    } else {
        #[cfg(ip_feature_dual_core)]
        {
            if SYSCTL.group1(index).value().read().link() & (1 << offset) != 0 {
                return;
            }
            SYSCTL.group1(index).set().write(|w| w.set_link(1 << offset));
        }
    }

    while SYSCTL.resource(resource).read().loc_busy() {}
}

pub fn clock_remove_from_group(resource: usize, group: usize) {
    const RESOURCE_START: usize = 256;
    if resource < RESOURCE_START || resource == usize::MAX {
        return;
    }
    let index = (resource - RESOURCE_START) / 32;
    let offset = (resource - RESOURCE_START) % 32;

    if group == 0 {
        SYSCTL.group0(index).clear().write(|w| w.set_link(1 << offset));
    } else {
        #[cfg(any(hpm6e, hpm67, hpm62))]
        SYSCTL.group1(index).clear().write(|w| w.set_link(1 << offset));
    }

    while SYSCTL.resource(resource).read().loc_busy() {}
}

pub(crate) trait SealedClockPeripheral {
    const SYSCTL_CLOCK: usize = usize::MAX;
    const SYSCTL_RESOURCE: usize = usize::MAX;

    fn frequency() -> Hertz {
        clocks().get_clock_freq(Self::SYSCTL_CLOCK)
    }

    fn add_resource_group(group: usize) {
        if Self::SYSCTL_RESOURCE == usize::MAX {
            return;
        }

        clock_add_to_group(Self::SYSCTL_RESOURCE, group);
    }

    fn set_clock(cfg: ClockConfig) {
        if Self::SYSCTL_CLOCK == usize::MAX {
            return;
        }
        SYSCTL.clock(Self::SYSCTL_CLOCK).modify(|w| {
            w.set_mux(cfg.src);
            w.set_div(cfg.raw_div);
        });
        while SYSCTL.clock(Self::SYSCTL_CLOCK).read().loc_busy() {}
    }
}

#[allow(private_bounds)]
pub trait ClockPeripheral: SealedClockPeripheral + 'static {}

pub(crate) trait SealedAnalogClockPeripheral {
    const ANA_CLOCK: usize;
    const SYSCTL_RESOURCE: usize;

    fn add_resource_group(group: usize) {
        if Self::SYSCTL_RESOURCE == usize::MAX {
            return;
        }

        clock_add_to_group(Self::SYSCTL_RESOURCE, group);
    }

    fn frequency() -> Hertz;

    fn set_ahb_clock();

    fn set_ana_clock(cfg: ClockConfig);
}

#[allow(private_bounds)]
pub trait AnalogClockPeripheral: SealedAnalogClockPeripheral + 'static {}
