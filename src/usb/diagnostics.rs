//! Optional USB controller diagnostics for hardware qualification.
//!
//! The public counters are available with the `usb-diagnostics` feature. The
//! internal recording helpers compile to no-ops in normal HAL builds.

#[cfg(feature = "usb-diagnostics")]
use core::sync::atomic::{AtomicU32, Ordering};

use super::Instance;

#[cfg(feature = "usb-diagnostics")]
pub static STAGE: AtomicU32 = AtomicU32::new(0);
#[cfg(feature = "usb-diagnostics")]
pub static USBSTS: AtomicU32 = AtomicU32::new(0);
#[cfg(feature = "usb-diagnostics")]
pub static USBINTR: AtomicU32 = AtomicU32::new(0);
#[cfg(feature = "usb-diagnostics")]
pub static PORTSC1: AtomicU32 = AtomicU32::new(0);
#[cfg(feature = "usb-diagnostics")]
pub static OTGSC: AtomicU32 = AtomicU32::new(0);
#[cfg(feature = "usb-diagnostics")]
pub static PHY_CTRL0: AtomicU32 = AtomicU32::new(0);
#[cfg(feature = "usb-diagnostics")]
pub static USBCMD: AtomicU32 = AtomicU32::new(0);
#[cfg(feature = "usb-diagnostics")]
pub static USBMODE: AtomicU32 = AtomicU32::new(0);
#[cfg(feature = "usb-diagnostics")]
pub static IRQ_COUNT: AtomicU32 = AtomicU32::new(0);
#[cfg(feature = "usb-diagnostics")]
pub static QTD_DATA_PTR: AtomicU32 = AtomicU32::new(0);
#[cfg(feature = "usb-diagnostics")]
pub static QTD_SYS_ADDR: AtomicU32 = AtomicU32::new(0);
#[cfg(feature = "usb-diagnostics")]
pub static QTD_ADDR: AtomicU32 = AtomicU32::new(0);
#[cfg(feature = "usb-diagnostics")]
pub static QTD_LEN: AtomicU32 = AtomicU32::new(0);

#[inline(always)]
pub(crate) fn set_stage(stage: u32) {
    #[cfg(feature = "usb-diagnostics")]
    STAGE.store(stage, Ordering::Relaxed);

    #[cfg(not(feature = "usb-diagnostics"))]
    let _ = stage;
}

#[inline(always)]
pub(crate) fn snapshot<T: Instance>(stage: u32) {
    #[cfg(feature = "usb-diagnostics")]
    {
        let r = T::info().regs;
        STAGE.store(stage, Ordering::Relaxed);
        USBSTS.store(r.usbsts().read().0, Ordering::Relaxed);
        USBINTR.store(r.usbintr().read().0, Ordering::Relaxed);
        PORTSC1.store(r.portsc1().read().0, Ordering::Relaxed);
        OTGSC.store(r.otgsc().read().0, Ordering::Relaxed);
        PHY_CTRL0.store(r.phy_ctrl0().read().0, Ordering::Relaxed);
        USBCMD.store(r.usbcmd().read().0, Ordering::Relaxed);
        USBMODE.store(r.usbmode().read().0, Ordering::Relaxed);
    }

    #[cfg(not(feature = "usb-diagnostics"))]
    let _ = stage;
}

#[inline(always)]
pub(crate) fn record_interrupt<T: Instance>(status: u32, enabled: u32) {
    #[cfg(feature = "usb-diagnostics")]
    {
        let r = T::info().regs;
        IRQ_COUNT.fetch_add(1, Ordering::Relaxed);
        STAGE.store(0x700, Ordering::Relaxed);
        USBSTS.store(status, Ordering::Relaxed);
        USBINTR.store(enabled, Ordering::Relaxed);
        PORTSC1.store(r.portsc1().read().0, Ordering::Relaxed);
        OTGSC.store(r.otgsc().read().0, Ordering::Relaxed);
        PHY_CTRL0.store(r.phy_ctrl0().read().0, Ordering::Relaxed);
        USBCMD.store(r.usbcmd().read().0, Ordering::Relaxed);
        USBMODE.store(r.usbmode().read().0, Ordering::Relaxed);
    }

    #[cfg(not(feature = "usb-diagnostics"))]
    let _ = (status, enabled);
}

#[inline(always)]
pub(crate) fn record_qtd(qtd_addr: u32, data_ptr: u32, len: usize) {
    #[cfg(feature = "usb-diagnostics")]
    {
        QTD_ADDR.store(qtd_addr, Ordering::Relaxed);
        QTD_DATA_PTR.store(data_ptr, Ordering::Relaxed);
        QTD_LEN.store(len as u32, Ordering::Relaxed);
    }

    #[cfg(not(feature = "usb-diagnostics"))]
    let _ = (qtd_addr, data_ptr, len);
}

#[inline(always)]
pub(crate) fn record_qtd_system_address(address: u32) {
    #[cfg(feature = "usb-diagnostics")]
    QTD_SYS_ADDR.store(address, Ordering::Relaxed);

    #[cfg(not(feature = "usb-diagnostics"))]
    let _ = address;
}
