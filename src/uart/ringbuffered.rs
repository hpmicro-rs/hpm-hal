//! Ring-buffered UART RX using DMA circular mode
//!
//! This module provides a DMA-backed ring buffer for continuous UART reception.

use core::future::poll_fn;
use core::sync::atomic::{Ordering, compiler_fence};
use core::task::Poll;

use embassy_hal_internal::Peri;

use super::{Config, ConfigError, Error, Info, State, UartRx, configure};
use crate::dma::ReadableRingBuffer;
use crate::dma::ringbuffer::Error as RingbufError;
use crate::gpio::AnyPin;
use crate::interrupt::InterruptExt;
use crate::mode::Async;
use crate::time::Hertz;

/// Ring-buffered UART RX driver
///
/// Created via [`UartRx::into_ring_buffered`].
///
/// This driver uses DMA circular mode for continuous reception without CPU intervention.
/// Data is automatically transferred to the ring buffer by DMA, and the application
/// is woken on:
/// - DMA half transfer (buffer half full)
/// - DMA transfer complete (buffer wrapped)
/// - UART IDLE line detection
pub struct RingBufferedUartRx<'d> {
    info: &'static Info,
    state: &'static State,
    kernel_clock: Hertz,
    _rx: Option<Peri<'d, AnyPin>>,
    ring_buf: ReadableRingBuffer<'d, u8>,
}

impl<'d> UartRx<'d, Async> {
    /// Convert to a ring-buffered UART RX
    ///
    /// The `dma_buf` is used as the DMA ring buffer and should be sized based on
    /// expected data rates and processing latency.
    pub fn into_ring_buffered(mut self, dma_buf: &'d mut [u8]) -> RingBufferedUartRx<'d> {
        assert!(!dma_buf.is_empty() && dma_buf.len() <= 0xFFFF);

        let info = self.info;
        let state = self.state;
        let kernel_clock = self.kernel_clock;

        // Safety: we consume self and rebuild with the ring buffer
        let rx_dma = self.rx_dma.take().unwrap();
        let ring_buf = unsafe {
            ReadableRingBuffer::new(
                rx_dma.channel,
                rx_dma.request,
                info.regs.rbr().as_ptr() as *mut u8,
                dma_buf,
                Default::default(),
            )
        };

        let rx = unsafe { self.rx.as_ref().map(|x| x.clone_unchecked()) };

        // Prevent drop from running cleanup
        core::mem::forget(self);

        RingBufferedUartRx {
            info,
            state,
            kernel_clock,
            _rx: rx,
            ring_buf,
        }
    }
}

impl<'d> RingBufferedUartRx<'d> {
    /// Reconfigure the UART
    pub fn set_config(&mut self, config: &Config) -> Result<(), ConfigError> {
        configure(self.info, self.kernel_clock, config, false, true)
    }

    /// Start the ring buffer reception
    pub fn start(&mut self) {
        compiler_fence(Ordering::SeqCst);

        // Set ring buffered mode flag - this prevents the standard UART interrupt
        // handler from disabling IDLE detection after first trigger, and keeps
        // DMAE enabled after TX DMA operations
        self.state.ring_buffered_mode.store(true, Ordering::Release);

        // Clear ring buffer state
        self.ring_buf.clear();
        self.ring_buf.start();

        let r = self.info.regs;

        // Enable UART IDLE interrupt for wake-up on short messages
        r.ier().modify(|w| {
            #[cfg(ip_feature_uart_rx_idle_detect)]
            w.set_erxidle(true);
        });

        // Enable RX IDLE detection
        #[cfg(ip_feature_uart_rx_idle_detect)]
        r.idle_cfg().modify(|w| {
            w.set_rx_idle_en(true);
            w.set_rx_idle_thr(20);
        });

        // Enable DMA for UART RX - ensure FIFO is properly configured for DMA
        #[cfg(ip_feature_uart_fine_fifo_thrld)]
        r.fcrr().modify(|w| {
            w.set_fifoe(true); // Enable FIFO
            w.set_fifot4en(true); // Enable fine FIFO threshold
            w.set_rfifot4(0); // RX trigger at 1 byte (0 = 1 byte)
            w.set_dmae(true); // Enable DMA
        });
        #[cfg(not(ip_feature_uart_fine_fifo_thrld))]
        {
            use crate::pac;
            let mut fcr = pac::uart::regs::Fcr(r.gpr().read().data() as _);
            fcr.set_fifoe(true);
            fcr.set_dmae(true);
            r.fcr().write_value(fcr);
        }

        // Ensure UART interrupt is enabled
        self.info.interrupt.unpend();
        unsafe { self.info.interrupt.enable() };
    }

    /// Stop the ring buffer reception
    fn stop(&mut self) {
        // Clear ring buffered mode flag
        self.state.ring_buffered_mode.store(false, Ordering::Release);

        self.ring_buf.request_pause();

        let r = self.info.regs;

        // Disable UART IDLE interrupt
        r.ier().modify(|w| {
            #[cfg(ip_feature_uart_rx_idle_detect)]
            w.set_erxidle(false);
        });

        // Disable RX IDLE detection
        #[cfg(ip_feature_uart_rx_idle_detect)]
        r.idle_cfg().modify(|w| w.set_rx_idle_en(false));

        // Disable DMA
        #[cfg(ip_feature_uart_fine_fifo_thrld)]
        r.fcrr().modify(|w| w.set_dmae(false));
        #[cfg(not(ip_feature_uart_fine_fifo_thrld))]
        {
            use crate::pac;
            let mut fcr = pac::uart::regs::Fcr(r.gpr().read().data() as _);
            fcr.set_dmae(false);
            r.fcr().write_value(fcr);
        }

        compiler_fence(Ordering::SeqCst);
    }

    fn check_errors(&self) -> Result<(), Error> {
        let r = self.info.regs;
        let lsr = r.lsr().read();

        if lsr.pe() {
            Err(Error::Parity)
        } else if lsr.fe() {
            Err(Error::Framing)
        } else if lsr.oe() {
            Err(Error::Overrun)
        } else {
            Ok(())
        }
    }

    /// Read bytes from the ring buffer
    ///
    /// Returns the number of bytes read. This will wait for data to be available
    /// via DMA or IDLE detection.
    pub async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Error> {
        // Start if not running
        if !self.ring_buf.is_running() {
            self.start();
        }

        loop {
            // Check for errors first (PE, FE only - OE is ignored in ring_buffered mode)
            if let Err(e) = self.check_errors() {
                return Err(e);
            }

            // Try to read from ring buffer
            match self.ring_buf.read(buf) {
                Ok((0, _)) => {
                    // No data yet, wait for DMA/IDLE
                }
                Ok((len, _)) => {
                    return Ok(len);
                }
                Err(RingbufError::Overrun) => {
                    self.stop();
                    return Err(Error::Overrun);
                }
                Err(RingbufError::DmaUnsynced) => {
                    self.stop();
                    return Err(Error::Overrun);
                }
            }

            // Wait for data or idle (woken by DMA half/complete interrupt or UART IDLE)
            self.wait_for_data_or_idle().await?;
        }
    }

    async fn wait_for_data_or_idle(&mut self) -> Result<(), Error> {
        compiler_fence(Ordering::SeqCst);

        let state = self.state;

        poll_fn(|cx| {
            // Register both DMA waker and UART waker
            // - DMA waker: triggered on half transfer / transfer complete
            // - UART waker: triggered on IDLE line detection
            self.ring_buf.set_waker(cx.waker());
            state.rx_waker.register(cx.waker());

            compiler_fence(Ordering::SeqCst);

            // Check if DMA has data - return Ready when we have data
            match self.ring_buf.len() {
                Ok(len) if len > 0 => Poll::Ready(Ok(())),
                Ok(_) => Poll::Pending,
                Err(_) => Poll::Ready(Err(Error::Overrun)),
            }
        })
        .await
    }

    /// Get the current readable length
    pub fn len(&mut self) -> Result<usize, Error> {
        self.ring_buf.len().map_err(|_| Error::Overrun)
    }

    /// Check if ring buffer is empty
    pub fn is_empty(&mut self) -> Result<bool, Error> {
        Ok(self.len()? == 0)
    }

    /// Get the buffer capacity
    pub const fn capacity(&self) -> usize {
        self.ring_buf.capacity()
    }
}

impl Drop for RingBufferedUartRx<'_> {
    fn drop(&mut self) {
        self.stop();
    }
}

// embedded_io_async traits
impl embedded_io_async::ErrorType for RingBufferedUartRx<'_> {
    type Error = Error;
}

impl embedded_io_async::Read for RingBufferedUartRx<'_> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        self.read(buf).await
    }
}
