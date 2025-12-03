//! Buffered UART driver using interrupt-driven ring buffers.

use core::future::poll_fn;
use core::marker::PhantomData;
use core::slice;
use core::sync::atomic::{AtomicBool, AtomicU8, Ordering};
use core::task::Poll;

use embassy_hal_internal::Peri;
use embassy_hal_internal::atomic_ring_buffer::RingBuffer;
use embassy_sync::waitqueue::AtomicWaker;

use super::{AnyPin, Config, ConfigError, Error, Info, Instance, SealedPin, configure, reconfigure};
use crate::interrupt::{self, InterruptExt};
use crate::pac;
use crate::time::Hertz;

/// Interrupt handler for buffered UART.
pub struct BufferedInterruptHandler<T: Instance> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> interrupt::typelevel::Handler<T::Interrupt> for BufferedInterruptHandler<T> {
    unsafe fn on_interrupt() {
        on_interrupt(T::info().regs, T::buffered_state())
    }
}

unsafe fn on_interrupt(r: pac::uart::Uart, state: &'static BufferedState) {
    let mut lsr = r.lsr().read();

    // Handle errors
    if lsr.pe() {
        defmt::warn!("Parity error");
    }
    if lsr.fe() {
        defmt::warn!("Framing error");
    }
    if lsr.oe() {
        defmt::warn!("Overrun error");
    }

    // RX: read ALL available data from UART FIFO into software ring buffer
    // This prevents hardware FIFO overrun by moving data to larger software buffer
    let mut rx_count = 0u32;
    while lsr.dr() {
        let mut rx_writer = state.rx_buf.writer();
        let buf = rx_writer.push_slice();
        if !buf.is_empty() {
            buf[0] = r.rbr().read().rbr();
            rx_writer.push_done(1);
            rx_count += 1;
        } else {
            // Software ring buffer full, stop reading
            break;
        }
        lsr = r.lsr().read();
    }

    // Determine when to wake the application
    let mut should_wake = false;
    let mut wake_reason: &str = "";

    // IDLE line detection - wake when sender stops
    #[cfg(ip_feature_uart_rx_idle_detect)]
    {
        #[cfg(ip_feature_uart_e00018_fix)]
        {
            let iir2 = r.iir2().read();
            if iir2.rxidle_flag() {
                r.iir2().modify(|w| w.set_rxidle_flag(true)); // W1C
                should_wake = true;
                wake_reason = "IDLE(iir2)";
            }
        }

        #[cfg(all(not(ip_feature_uart_e00018_fix), ip_feature_uart_9bit_mode))]
        if lsr.rxidle() {
            should_wake = true;
            wake_reason = "IDLE(lsr)";
        }
    }

    // For chips without IDLE detection, wake when any data received
    #[cfg(not(ip_feature_uart_rx_idle_detect))]
    if !state.rx_buf.is_empty() {
        should_wake = true;
        wake_reason = "no_idle_detect";
    }

    // Debug: disabled to avoid RTT interference
    let _ = (rx_count, wake_reason);

    if should_wake {
        state.rx_woken.store(true, Ordering::Release);
        state.rx_waker.wake();
    }

    // TX: transmitter holding register empty - send next byte
    if lsr.thre() {
        let mut tx_reader = state.tx_buf.reader();
        let buf = tx_reader.pop_slice();

        if !buf.is_empty() {
            // Write byte to THR
            r.thr().write(|w| w.set_thr(buf[0]));
            tx_reader.pop_done(1);

            // Enable THRE interrupt to send remaining bytes
            r.ier().modify(|w| w.set_ethei(true));
        } else {
            // No more data to send, disable THRE interrupt
            r.ier().modify(|w| w.set_ethei(false));
            state.tx_done.store(true, Ordering::Release);
            state.tx_waker.wake();
        }
    }
}

pub(super) struct BufferedState {
    rx_waker: AtomicWaker,
    rx_buf: RingBuffer,
    rx_woken: AtomicBool, // Track if rx_waker was explicitly woken
    tx_waker: AtomicWaker,
    tx_buf: RingBuffer,
    tx_done: AtomicBool,
    tx_rx_refcount: AtomicU8,
}

impl BufferedState {
    pub(super) const fn new() -> Self {
        Self {
            rx_buf: RingBuffer::new(),
            tx_buf: RingBuffer::new(),
            rx_waker: AtomicWaker::new(),
            rx_woken: AtomicBool::new(false),
            tx_waker: AtomicWaker::new(),
            tx_done: AtomicBool::new(true),
            tx_rx_refcount: AtomicU8::new(0),
        }
    }
}

/// Bidirectional buffered UART
pub struct BufferedUart<'d> {
    rx: BufferedUartRx<'d>,
    tx: BufferedUartTx<'d>,
}

/// Tx-only buffered UART
pub struct BufferedUartTx<'d> {
    info: &'static Info,
    state: &'static BufferedState,
    kernel_clock: Hertz,
    tx: Option<Peri<'d, AnyPin>>,
    is_borrowed: bool,
}

/// Rx-only buffered UART
pub struct BufferedUartRx<'d> {
    info: &'static Info,
    state: &'static BufferedState,
    kernel_clock: Hertz,
    rx: Option<Peri<'d, AnyPin>>,
    is_borrowed: bool,
}

impl<'d> BufferedUart<'d> {
    /// Create a new bidirectional buffered UART driver
    pub fn new<T: Instance>(
        peri: Peri<'d, T>,
        rx: Peri<'d, impl super::RxPin<T>>,
        tx: Peri<'d, impl super::TxPin<T>>,
        _irq: impl interrupt::typelevel::Binding<T::Interrupt, BufferedInterruptHandler<T>> + 'd,
        tx_buffer: &'d mut [u8],
        rx_buffer: &'d mut [u8],
        config: Config,
    ) -> Result<Self, ConfigError> {
        rx.set_as_alt(rx.alt_num());
        tx.set_as_alt(tx.alt_num());

        Self::new_inner(peri, Some(rx.into()), Some(tx.into()), tx_buffer, rx_buffer, config)
    }

    fn new_inner<T: Instance>(
        _peri: Peri<'d, T>,
        rx: Option<Peri<'d, AnyPin>>,
        tx: Option<Peri<'d, AnyPin>>,
        tx_buffer: &'d mut [u8],
        rx_buffer: &'d mut [u8],
        config: Config,
    ) -> Result<Self, ConfigError> {
        use crate::sysctl::*;
        T::set_clock(ClockConfig::new(ClockMux::CLK_24M, 1));
        T::add_resource_group(0);

        let info = T::info();
        let state = T::buffered_state();
        let kernel_clock = T::frequency();

        let mut this = Self {
            rx: BufferedUartRx {
                info,
                state,
                kernel_clock,
                rx,
                is_borrowed: false,
            },
            tx: BufferedUartTx {
                info,
                state,
                kernel_clock,
                tx,
                is_borrowed: false,
            },
        };
        this.enable_and_configure(tx_buffer, rx_buffer, &config)?;
        Ok(this)
    }

    fn enable_and_configure(
        &mut self,
        tx_buffer: &'d mut [u8],
        rx_buffer: &'d mut [u8],
        config: &Config,
    ) -> Result<(), ConfigError> {
        let info = self.rx.info;
        let state = self.rx.state;
        state.tx_rx_refcount.store(2, Ordering::Relaxed);

        assert!(!tx_buffer.is_empty());
        let len = tx_buffer.len();
        unsafe { state.tx_buf.init(tx_buffer.as_mut_ptr(), len) };
        assert!(!rx_buffer.is_empty());
        let len = rx_buffer.len();
        unsafe { state.rx_buf.init(rx_buffer.as_mut_ptr(), len) };

        configure(info, self.rx.kernel_clock, config, true, true)?;

        // Enable RX interrupts
        // ERBI moves data from hardware FIFO to software ring buffer (prevents overrun)
        // ERXIDLE triggers wake when sender stops (for batch processing)
        info.regs.ier().modify(|w| {
            w.set_erbi(true); // Enable Received Data Available Interrupt
            w.set_elsi(true); // Enable Receiver Line Status Interrupt (errors)
            #[cfg(ip_feature_uart_rx_idle_detect)]
            w.set_erxidle(true); // Enable RX Idle Interrupt
        });

        // Enable RX idle detection
        #[cfg(ip_feature_uart_rx_idle_detect)]
        info.regs.idle_cfg().modify(|w| {
            w.set_rx_idle_en(true);
            w.set_rx_idle_thr(20); // 20 bit times
        });

        info.interrupt.unpend();
        unsafe { info.interrupt.enable() };

        Ok(())
    }

    /// Split the driver into a Tx and Rx part
    pub fn split(self) -> (BufferedUartTx<'d>, BufferedUartRx<'d>) {
        (self.tx, self.rx)
    }

    /// Split the Uart into a transmitter and receiver by reference
    pub fn split_ref(&mut self) -> (BufferedUartTx<'_>, BufferedUartRx<'_>) {
        (
            BufferedUartTx {
                info: self.tx.info,
                state: self.tx.state,
                kernel_clock: self.tx.kernel_clock,
                tx: self.tx.tx.as_mut().map(Peri::reborrow),
                is_borrowed: true,
            },
            BufferedUartRx {
                info: self.rx.info,
                state: self.rx.state,
                kernel_clock: self.rx.kernel_clock,
                rx: self.rx.rx.as_mut().map(Peri::reborrow),
                is_borrowed: true,
            },
        )
    }

    /// Reconfigure the driver
    pub fn set_config(&mut self, config: &Config) -> Result<(), ConfigError> {
        reconfigure(self.rx.info, self.rx.kernel_clock, config)?;

        self.rx.info.regs.ier().modify(|w| {
            w.set_erbi(true);
            w.set_elsi(true);
            #[cfg(ip_feature_uart_rx_idle_detect)]
            w.set_erxidle(true);
        });

        Ok(())
    }
}

impl<'d> BufferedUartRx<'d> {
    async fn read(&self, buf: &mut [u8]) -> Result<usize, Error> {
        poll_fn(move |cx| {
            let state = self.state;

            // Only proceed if we were explicitly woken (IDLE detected)
            // This prevents returning data on every executor poll
            if !state.rx_woken.swap(false, Ordering::AcqRel) {
                state.rx_waker.register(cx.waker());
                return Poll::Pending;
            }

            // We were woken (IDLE detected), now read all available data
            let mut rx_reader = unsafe { state.rx_buf.reader() };
            let mut buf_len = 0;
            let mut data = rx_reader.pop_slice();

            while !data.is_empty() && buf_len < buf.len() {
                let data_len = data.len().min(buf.len() - buf_len);
                buf[buf_len..buf_len + data_len].copy_from_slice(&data[..data_len]);
                buf_len += data_len;

                let do_pend = state.rx_buf.is_full();
                rx_reader.pop_done(data_len);

                if do_pend {
                    self.info.interrupt.pend();
                }

                data = rx_reader.pop_slice();
            }

            if buf_len != 0 {
                Poll::Ready(Ok(buf_len))
            } else {
                // Woken but no data yet, wait again
                state.rx_waker.register(cx.waker());
                Poll::Pending
            }
        })
        .await
    }

    fn blocking_read(&self, buf: &mut [u8]) -> Result<usize, Error> {
        loop {
            let state = self.state;
            let mut rx_reader = unsafe { state.rx_buf.reader() };
            let mut buf_len = 0;
            let mut data = rx_reader.pop_slice();

            while !data.is_empty() && buf_len < buf.len() {
                let data_len = data.len().min(buf.len() - buf_len);
                buf[buf_len..buf_len + data_len].copy_from_slice(&data[..data_len]);
                buf_len += data_len;

                let do_pend = state.rx_buf.is_full();
                rx_reader.pop_done(data_len);

                if do_pend {
                    self.info.interrupt.pend();
                }

                data = rx_reader.pop_slice();
            }
            if buf_len != 0 {
                return Ok(buf_len);
            }
        }
    }

    async fn fill_buf(&self) -> Result<&[u8], Error> {
        poll_fn(move |cx| {
            let state = self.state;
            let mut rx_reader = unsafe { state.rx_buf.reader() };
            let (p, n) = rx_reader.pop_buf();
            if n == 0 {
                state.rx_waker.register(cx.waker());
                return Poll::Pending;
            }

            let buf = unsafe { slice::from_raw_parts(p, n) };
            Poll::Ready(Ok(buf))
        })
        .await
    }

    fn consume(&self, amt: usize) {
        let state = self.state;
        let mut rx_reader = unsafe { state.rx_buf.reader() };
        let full = state.rx_buf.is_full();
        rx_reader.pop_done(amt);
        if full {
            self.info.interrupt.pend();
        }
    }

    fn read_ready(&mut self) -> Result<bool, Error> {
        let state = self.state;
        Ok(!state.rx_buf.is_empty())
    }

    /// Reconfigure the driver
    pub fn set_config(&mut self, config: &Config) -> Result<(), ConfigError> {
        reconfigure(self.info, self.kernel_clock, config)?;

        self.info.regs.ier().modify(|w| {
            w.set_erbi(true);
            w.set_elsi(true);
            #[cfg(ip_feature_uart_rx_idle_detect)]
            w.set_erxidle(true);
        });

        Ok(())
    }
}

impl<'d> BufferedUartTx<'d> {
    async fn write(&self, buf: &[u8]) -> Result<usize, Error> {
        poll_fn(move |cx| {
            let r = self.info.regs;
            let state = self.state;
            state.tx_done.store(false, Ordering::Release);

            let empty = state.tx_buf.is_empty();

            let mut tx_writer = unsafe { state.tx_buf.writer() };
            let data = tx_writer.push_slice();
            if data.is_empty() {
                state.tx_waker.register(cx.waker());
                return Poll::Pending;
            }

            let n = data.len().min(buf.len());
            data[..n].copy_from_slice(&buf[..n]);
            tx_writer.push_done(n);

            if empty {
                // Buffer was empty, write first byte directly to start transmission
                critical_section::with(|_| {
                    let mut tx_reader = unsafe { state.tx_buf.reader() };
                    let first_byte = tx_reader.pop_slice();
                    if !first_byte.is_empty() && r.lsr().read().thre() {
                        r.thr().write(|w| w.set_thr(first_byte[0]));
                        tx_reader.pop_done(1);
                        // Enable THRE interrupt for remaining bytes
                        r.ier().modify(|w| w.set_ethei(true));
                    }
                });
            }

            Poll::Ready(Ok(n))
        })
        .await
    }

    async fn flush(&self) -> Result<(), Error> {
        poll_fn(move |cx| {
            let state = self.state;

            if !state.tx_done.load(Ordering::Acquire) {
                state.tx_waker.register(cx.waker());
                return Poll::Pending;
            }

            Poll::Ready(Ok(()))
        })
        .await
    }

    fn blocking_write(&self, buf: &[u8]) -> Result<usize, Error> {
        loop {
            let r = self.info.regs;
            let state = self.state;
            state.tx_done.store(false, Ordering::Release);

            let empty = state.tx_buf.is_empty();

            let mut tx_writer = unsafe { state.tx_buf.writer() };
            let data = tx_writer.push_slice();
            if !data.is_empty() {
                let n = data.len().min(buf.len());
                data[..n].copy_from_slice(&buf[..n]);
                tx_writer.push_done(n);

                if empty {
                    // Buffer was empty, write first byte directly to start transmission
                    critical_section::with(|_| {
                        let mut tx_reader = unsafe { state.tx_buf.reader() };
                        let first_byte = tx_reader.pop_slice();
                        if !first_byte.is_empty() && r.lsr().read().thre() {
                            r.thr().write(|w| w.set_thr(first_byte[0]));
                            tx_reader.pop_done(1);
                            // Enable THRE interrupt for remaining bytes
                            r.ier().modify(|w| w.set_ethei(true));
                        }
                    });
                }

                return Ok(n);
            }
        }
    }

    fn blocking_flush(&self) -> Result<(), Error> {
        loop {
            let state = self.state;
            if state.tx_done.load(Ordering::Acquire) {
                return Ok(());
            }
        }
    }

    /// Reconfigure the driver
    pub fn set_config(&mut self, config: &Config) -> Result<(), ConfigError> {
        reconfigure(self.info, self.kernel_clock, config)
    }
}

impl<'d> Drop for BufferedUartRx<'d> {
    fn drop(&mut self) {
        if !self.is_borrowed {
            let state = self.state;
            unsafe {
                state.rx_buf.deinit();

                if state.tx_buf.len() == 0 {
                    self.info.interrupt.disable();
                }
            }

            self.rx.as_ref().map(|x| x.set_as_default());
            buffered_drop_tx_rx(self.info, state);
        }
    }
}

impl<'d> Drop for BufferedUartTx<'d> {
    fn drop(&mut self) {
        if !self.is_borrowed {
            let state = self.state;
            unsafe {
                state.tx_buf.deinit();

                if state.rx_buf.len() == 0 {
                    self.info.interrupt.disable();
                }
            }

            self.tx.as_ref().map(|x| x.set_as_default());
            buffered_drop_tx_rx(self.info, state);
        }
    }
}

fn buffered_drop_tx_rx(info: &Info, state: &BufferedState) {
    let is_last_drop = critical_section::with(|_| {
        let refcount = state.tx_rx_refcount.load(Ordering::Relaxed);
        assert!(refcount >= 1);
        state.tx_rx_refcount.store(refcount - 1, Ordering::Relaxed);
        refcount == 1
    });
    if is_last_drop {
        crate::sysctl::clock_remove_from_group(info.resource, 0);
    }
}

// embedded-io-async traits

impl<'d> embedded_io_async::ErrorType for BufferedUart<'d> {
    type Error = Error;
}

impl<'d> embedded_io_async::ErrorType for BufferedUartRx<'d> {
    type Error = Error;
}

impl<'d> embedded_io_async::ErrorType for BufferedUartTx<'d> {
    type Error = Error;
}

impl<'d> embedded_io_async::Read for BufferedUart<'d> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        self.rx.read(buf).await
    }
}

impl<'d> embedded_io_async::Read for BufferedUartRx<'d> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        Self::read(self, buf).await
    }
}

impl<'d> embedded_io_async::ReadReady for BufferedUart<'d> {
    fn read_ready(&mut self) -> Result<bool, Self::Error> {
        BufferedUartRx::<'d>::read_ready(&mut self.rx)
    }
}

impl<'d> embedded_io_async::ReadReady for BufferedUartRx<'d> {
    fn read_ready(&mut self) -> Result<bool, Self::Error> {
        Self::read_ready(self)
    }
}

impl<'d> embedded_io_async::BufRead for BufferedUart<'d> {
    async fn fill_buf(&mut self) -> Result<&[u8], Self::Error> {
        self.rx.fill_buf().await
    }

    fn consume(&mut self, amt: usize) {
        self.rx.consume(amt)
    }
}

impl<'d> embedded_io_async::BufRead for BufferedUartRx<'d> {
    async fn fill_buf(&mut self) -> Result<&[u8], Self::Error> {
        Self::fill_buf(self).await
    }

    fn consume(&mut self, amt: usize) {
        Self::consume(self, amt)
    }
}

impl<'d> embedded_io_async::Write for BufferedUart<'d> {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        self.tx.write(buf).await
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        self.tx.flush().await
    }
}

impl<'d> embedded_io_async::Write for BufferedUartTx<'d> {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        Self::write(self, buf).await
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        Self::flush(self).await
    }
}

// embedded-io (blocking) traits

impl<'d> embedded_io::Read for BufferedUart<'d> {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        self.rx.blocking_read(buf)
    }
}

impl<'d> embedded_io::Read for BufferedUartRx<'d> {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        self.blocking_read(buf)
    }
}

impl<'d> embedded_io::Write for BufferedUart<'d> {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        self.tx.blocking_write(buf)
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        self.tx.blocking_flush()
    }
}

impl<'d> embedded_io::Write for BufferedUartTx<'d> {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        Self::blocking_write(self, buf)
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        Self::blocking_flush(self)
    }
}

// embedded-hal-nb traits

impl<'d> embedded_hal_nb::serial::ErrorType for BufferedUart<'d> {
    type Error = Error;
}

impl<'d> embedded_hal_nb::serial::ErrorType for BufferedUartTx<'d> {
    type Error = Error;
}

impl<'d> embedded_hal_nb::serial::ErrorType for BufferedUartRx<'d> {
    type Error = Error;
}

impl<'d> embedded_hal_nb::serial::Read for BufferedUartRx<'d> {
    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        let state = self.state;
        let mut rx_reader = unsafe { state.rx_buf.reader() };

        let do_pend = state.rx_buf.is_full();
        if let Some(data) = rx_reader.pop_one() {
            if do_pend {
                self.info.interrupt.pend();
            }
            Ok(data)
        } else {
            Err(nb::Error::WouldBlock)
        }
    }
}

impl<'d> embedded_hal_nb::serial::Write for BufferedUartTx<'d> {
    fn write(&mut self, char: u8) -> nb::Result<(), Self::Error> {
        self.blocking_write(&[char]).map(drop).map_err(nb::Error::Other)
    }

    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        self.blocking_flush().map_err(nb::Error::Other)
    }
}

impl<'d> embedded_hal_nb::serial::Read for BufferedUart<'d> {
    fn read(&mut self) -> Result<u8, nb::Error<Self::Error>> {
        embedded_hal_nb::serial::Read::read(&mut self.rx)
    }
}

impl<'d> embedded_hal_nb::serial::Write for BufferedUart<'d> {
    fn write(&mut self, char: u8) -> nb::Result<(), Self::Error> {
        self.tx.blocking_write(&[char]).map(drop).map_err(nb::Error::Other)
    }

    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        self.tx.blocking_flush().map_err(nb::Error::Other)
    }
}
