use core::future::poll_fn;
use core::marker::PhantomData;
use core::task::Poll;

use embassy_usb_driver::{Direction, EndpointAddress, EndpointIn, EndpointInfo, EndpointOut};

use super::{QTD_COUNT_EACH_QHD, get_active_ep_state, local_to_sys_address};
use crate::usb::{EP_IN_WAKERS, EP_OUT_WAKERS, Instance};

/// USB transfer error types
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum TransferError {
    /// Transfer size exceeds maximum (>128KB, needs >8 QTDs)
    BufferTooLarge,
    /// Buffer alignment requirement not met (>4K data needs 4K alignment)
    BufferAlignment,
}

pub(crate) struct EpConfig {
    /// Endpoint type
    pub(crate) transfer: u8,
    pub(crate) ep_addr: EndpointAddress,
    pub(crate) max_packet_size: u16,
}

/// Direction marker trait for endpoints.
pub trait Dir {
    /// Get the direction value.
    fn dir() -> Direction;
}

/// Marker type for IN direction (device to host).
pub enum In {}
impl Dir for In {
    fn dir() -> Direction {
        Direction::In
    }
}

/// Marker type for OUT direction (host to device).
pub enum Out {}
impl Dir for Out {
    fn dir() -> Direction {
        Direction::Out
    }
}

/// USB endpoint with direction type parameter.
#[derive(Copy, Clone)]
pub struct Endpoint<'d, T: Instance, D> {
    pub(crate) _phantom: PhantomData<(&'d mut T, D)>,
    pub(crate) info: EndpointInfo,
}

impl<'d, T: Instance, D> Endpoint<'d, T, D> {
    pub(crate) fn start_transfer(&mut self) {
        let ep_num = self.info.addr.index();

        let r = T::info().regs;
        r.endptprime().modify(|w| {
            if self.info.addr.is_in() {
                w.set_petb(1 << ep_num);
            } else {
                w.set_perb(1 << ep_num);
            }
        });
    }

    /// Schedule the transfer
    ///
    /// # Errors
    /// - `TransferError::BufferTooLarge` - Data exceeds 128KB (needs >8 QTDs)
    /// - `TransferError::BufferAlignment` - Data >4K but not 4K-aligned
    pub(crate) fn transfer(&mut self, data: &[u8]) -> Result<(), TransferError> {
        let r = T::info().regs;

        let ep_num = self.info.addr.index();
        let ep_idx = 2 * ep_num + self.info.addr.is_in() as usize;

        //  Setup packet handling using setup lockout mechanism
        //  wait until ENDPTSETUPSTAT before priming data/status in response
        if ep_num == 0 {
            while (r.endptsetupstat().read().endptsetupstat() & 0b1) == 1 {}
        }

        // Check transfer size limit (8 QTDs * 16KB each = 128KB max)
        let qtd_num = (data.len() + 0x3FFF) / 0x4000;
        if qtd_num > 8 {
            return Err(TransferError::BufferTooLarge);
        }

        // Check alignment for >4K transfers (buffer[1-4] must be 4K aligned)
        if data.len() > 0x1000 && (data.as_ptr() as usize) % 0x1000 != 0 {
            return Err(TransferError::BufferAlignment);
        }

        // Add all data to the circular queue
        let mut prev_qtd: Option<usize> = None;
        let mut first_qtd: Option<usize> = None;
        let mut i = 0;
        let mut data_offset = 0;
        let mut remaining_bytes = data.len();
        loop {
            let qtd_idx = ep_idx * QTD_COUNT_EACH_QHD + i;
            i += 1;

            // If the transfer size > 0x4000, then there should be multiple qtds in the linked list
            let transfer_bytes = if remaining_bytes > 0x4000 {
                remaining_bytes -= 0x4000;
                0x4000
            } else {
                remaining_bytes = 0;
                data.len()
            };

            // Initialize qtd with the data (address conversion is done inside reinit_with)
            unsafe {
                let ep_state = get_active_ep_state();
                ep_state
                    .qtd_list()
                    .qtd(qtd_idx)
                    .reinit_with(&data[data_offset..], transfer_bytes)
            };

            // Last chunk of the data
            if remaining_bytes == 0 {
                unsafe {
                    let ep_state = get_active_ep_state();
                    ep_state.qtd_list().qtd(qtd_idx).qtd_token().modify(|w| w.set_ioc(true));
                };
            }

            data_offset += transfer_bytes;

            // Set qtd linked list
            // Note: C SDK does NOT convert QTD->QTD address, only QHD->QTD address
            if let Some(prev_qtd) = prev_qtd {
                unsafe {
                    let ep_state = get_active_ep_state();
                    let qtd_addr = ep_state.qtd_list().qtd(qtd_idx).as_ptr() as u32;
                    ep_state
                        .qtd_list()
                        .qtd(prev_qtd)
                        .next_dtd()
                        .modify(|w| w.set_next_dtd_addr(qtd_addr >> 5));
                }
            } else {
                first_qtd = Some(qtd_idx);
            }

            prev_qtd = Some(qtd_idx);

            // Check the remaining_bytes
            if remaining_bytes == 0 {
                break;
            }
        }

        // Link qtd to qhd (convert to system address for DMA)
        let first_idx = first_qtd.unwrap();

        unsafe {
            let ep_state = get_active_ep_state();
            if ep_num == 0 {
                ep_state.qhd_list().qhd(ep_idx).cap().modify(|w| {
                    w.set_ios(true);
                });
            }
            let qtd_addr = local_to_sys_address(ep_state.qtd_list().qtd(first_idx).as_ptr() as u32);
            ep_state.qhd_list().qhd(ep_idx).next_dtd().modify(|w| {
                w.set_next_dtd_addr(qtd_addr >> 5);
                // T **MUST** be set to 0
                w.set_t(false);
            });
        }

        // Start transfer
        self.start_transfer();

        Ok(())
    }

    pub(crate) fn set_stall(&mut self) {
        let r = T::info().regs;
        if self.info.addr.is_in() {
            r.endptctrl(self.info.addr.index() as usize).modify(|w| w.set_txs(true));
        } else {
            r.endptctrl(self.info.addr.index() as usize).modify(|w| w.set_rxs(true));
        }
    }
}

impl<'d, T: Instance> embassy_usb_driver::Endpoint for Endpoint<'d, T, In> {
    fn info(&self) -> &embassy_usb_driver::EndpointInfo {
        &self.info
    }

    async fn wait_enabled(&mut self) {
        let i = self.info.addr.index();
        let r = T::info().regs;
        if r.endptctrl(i).read().txe() {
            return;
        }
        poll_fn(|cx| {
            EP_IN_WAKERS[i].register(cx.waker());
            if r.endptctrl(i).read().txe() {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        })
        .await;
    }
}

impl<'d, T: Instance> embassy_usb_driver::Endpoint for Endpoint<'d, T, Out> {
    fn info(&self) -> &embassy_usb_driver::EndpointInfo {
        &self.info
    }

    async fn wait_enabled(&mut self) {
        let i = self.info.addr.index();
        let r = T::info().regs;
        if r.endptctrl(i).read().rxe() {
            return;
        }
        poll_fn(|cx| {
            EP_OUT_WAKERS[i].register(cx.waker());
            if r.endptctrl(i).read().rxe() {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        })
        .await;
    }
}

impl<'d, T: Instance> EndpointOut for Endpoint<'d, T, Out> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, embassy_usb_driver::EndpointError> {
        let r = T::info().regs;
        let ep_num = self.info.addr.index();

        if !r.endptctrl(ep_num).read().rxe() {
            return Err(embassy_usb_driver::EndpointError::Disabled);
        }

        // Start read and wait
        self.transfer(buf).map_err(|_| embassy_usb_driver::EndpointError::BufferOverflow)?;
        poll_fn(|cx| {
            EP_OUT_WAKERS[ep_num].register(cx.waker());

            if r.endptcomplete().read().erce() & (1 << ep_num) != 0 {
                r.endptcomplete().modify(|w| w.set_erce(1 << ep_num));
                Poll::Ready(())
            } else if !r.endptctrl(ep_num).read().rxe() {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        })
        .await;

        // Get the actual length of the packet (OUT endpoint: ep_idx = 2 * ep_num)
        let ep_idx = 2 * ep_num;
        let len = unsafe {
            let ep_state = get_active_ep_state();
            ep_state.qhd_list().qhd(ep_idx).qtd_token().read().total_bytes() as usize
        };
        Ok(buf.len() - len)
    }
}

impl<'d, T: Instance> EndpointIn for Endpoint<'d, T, In> {
    async fn write(&mut self, buf: &[u8]) -> Result<(), embassy_usb_driver::EndpointError> {
        let r = T::info().regs;
        let ep_num = self.info.addr.index();

        if !r.endptctrl(ep_num).read().txe() {
            return Err(embassy_usb_driver::EndpointError::Disabled);
        }

        // Start write and wait
        self.transfer(buf).map_err(|_| embassy_usb_driver::EndpointError::BufferOverflow)?;
        poll_fn(|cx| {
            EP_IN_WAKERS[ep_num].register(cx.waker());
            if r.endptcomplete().read().etce() & (1 << ep_num) != 0 {
                r.endptcomplete().modify(|w| w.set_etce(1 << ep_num));
                Poll::Ready(())
            } else if !r.endptctrl(ep_num).read().txe() {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        })
        .await;

        // Send zlt packet(if needed)
        if buf.len() == self.info.max_packet_size as usize {
            let _ = self.transfer(&[]);
            poll_fn(|cx| {
                EP_IN_WAKERS[ep_num].register(cx.waker());
                if r.endptcomplete().read().etce() & (1 << ep_num) != 0 {
                    r.endptcomplete().modify(|w| w.set_etce(1 << ep_num));
                    Poll::Ready(())
                } else {
                    Poll::Pending
                }
            })
            .await;
        }

        Ok(())
    }
}
