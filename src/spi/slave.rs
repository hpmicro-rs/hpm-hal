//! SPI Slave driver
//!
//! Provides an independent `SpiSlave` type for SPI slave mode operation.
//! The slave is passive — it responds to a master driving SCK and CS.
//!
//! Two modes are supported:
//! - **Data-only mode** (`data_only: true`): Pure data pipe, works with WriteReadTogether.
//! - **Command-aware mode** (`data_only: false`): Supports cmd/addr/status phases.

use core::marker::PhantomData;
use core::ptr;

use embassy_futures::join::join;
use embassy_hal_internal::Peri;

use super::consts::*;
use super::{
    flush_rx_fifo, BitOrder, CsPin, Info, Instance, MisoPin, MosiPin, SclkPin, SealedWord,
    SlaveConfig, SlaveStatus, State, Word,
};
use crate::dma::{self, ChannelAndRequest};
use crate::gpio::AnyPin;
use crate::mode::{Async, Blocking, Mode as PeriMode};
use crate::pac::spi::vals::TransMode;

/// SPI slave error.
#[derive(Debug, PartialEq, Eq, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// Buffer too large for a single transfer
    BufferTooLong,
    /// RX overrun — master clocked in data faster than slave consumed it
    Overrun,
    /// TX underrun — master clocked out before slave had data ready
    Underrun,
}

/// SPI Slave driver.
///
/// Unlike the master `Spi` type, this is passive — transfers are initiated
/// by the external SPI master driving SCK and pulling CS low.
#[allow(unused)]
pub struct SpiSlave<'d, M: PeriMode> {
    info: &'static Info,
    state: &'static State,
    sclk: Option<Peri<'d, AnyPin>>,
    mosi: Option<Peri<'d, AnyPin>>,
    miso: Option<Peri<'d, AnyPin>>,
    cs: Peri<'d, AnyPin>,
    tx_dma: Option<ChannelAndRequest<'d>>,
    rx_dma: Option<ChannelAndRequest<'d>>,
    _phantom: PhantomData<M>,
    current_word_size: super::word_impl::Config,
    data_only: bool,
}

// - MARK: Blocking constructors

impl<'d> SpiSlave<'d, Blocking> {
    /// Create a new blocking SPI slave driver.
    ///
    /// All four pins (SCLK, MOSI, MISO, CS) are required for slave mode.
    /// Pin directions are configured automatically:
    /// - SCLK, MOSI, CS: input (loop_back enabled)
    /// - MISO: output (alt function)
    pub fn new_blocking<T: Instance>(
        peri: Peri<'d, T>,
        sclk: Peri<'d, impl SclkPin<T>>,
        mosi: Peri<'d, impl MosiPin<T>>,
        miso: Peri<'d, impl MisoPin<T>>,
        cs: Peri<'d, impl CsPin<T>>,
        config: SlaveConfig,
    ) -> Self {
        T::add_resource_group(0);

        // SCLK: input in slave mode
        sclk.ioc_pad().func_ctl().modify(|w| {
            w.set_alt_select(sclk.alt_num());
            w.set_loop_back(true);
        });
        // MOSI: input (slave receives on MOSI)
        mosi.ioc_pad().func_ctl().modify(|w| {
            w.set_alt_select(mosi.alt_num());
            w.set_loop_back(true);
        });
        // MISO: output (slave transmits on MISO)
        miso.set_as_alt(miso.alt_num());
        // CS: input (master controls CS)
        cs.ioc_pad().func_ctl().modify(|w| {
            w.set_alt_select(cs.alt_num());
            w.set_loop_back(true);
        });

        Self::new_inner(
            peri,
            Some(sclk.into()),
            Some(mosi.into()),
            Some(miso.into()),
            cs.into(),
            None,
            None,
            config,
        )
    }

    /// Create a new blocking SPI slave driver in RX-only mode (no MISO).
    pub fn new_blocking_rxonly<T: Instance>(
        peri: Peri<'d, T>,
        sclk: Peri<'d, impl SclkPin<T>>,
        mosi: Peri<'d, impl MosiPin<T>>,
        cs: Peri<'d, impl CsPin<T>>,
        config: SlaveConfig,
    ) -> Self {
        T::add_resource_group(0);

        sclk.ioc_pad().func_ctl().modify(|w| {
            w.set_alt_select(sclk.alt_num());
            w.set_loop_back(true);
        });
        mosi.ioc_pad().func_ctl().modify(|w| {
            w.set_alt_select(mosi.alt_num());
            w.set_loop_back(true);
        });
        cs.ioc_pad().func_ctl().modify(|w| {
            w.set_alt_select(cs.alt_num());
            w.set_loop_back(true);
        });

        Self::new_inner(
            peri,
            Some(sclk.into()),
            Some(mosi.into()),
            None,
            cs.into(),
            None,
            None,
            config,
        )
    }
}

// - MARK: Async constructors

impl<'d> SpiSlave<'d, Async> {
    /// Create a new async SPI slave driver with DMA.
    pub fn new<T: Instance>(
        peri: Peri<'d, T>,
        sclk: Peri<'d, impl SclkPin<T>>,
        mosi: Peri<'d, impl MosiPin<T>>,
        miso: Peri<'d, impl MisoPin<T>>,
        cs: Peri<'d, impl CsPin<T>>,
        tx_dma: Peri<'d, impl super::TxDma<T>>,
        rx_dma: Peri<'d, impl super::RxDma<T>>,
        config: SlaveConfig,
    ) -> Self {
        T::add_resource_group(0);

        sclk.ioc_pad().func_ctl().modify(|w| {
            w.set_alt_select(sclk.alt_num());
            w.set_loop_back(true);
        });
        mosi.ioc_pad().func_ctl().modify(|w| {
            w.set_alt_select(mosi.alt_num());
            w.set_loop_back(true);
        });
        miso.set_as_alt(miso.alt_num());
        cs.ioc_pad().func_ctl().modify(|w| {
            w.set_alt_select(cs.alt_num());
            w.set_loop_back(true);
        });

        Self::new_inner(
            peri,
            Some(sclk.into()),
            Some(mosi.into()),
            Some(miso.into()),
            cs.into(),
            new_dma!(tx_dma),
            new_dma!(rx_dma),
            config,
        )
    }
}

// - MARK: Async transfer methods

impl<'d> SpiSlave<'d, Async> {
    /// Async bidirectional slave transfer using DMA.
    ///
    /// Prepares TX data and waits for the master to drive the transfer.
    /// `write` data is sent on MISO, received data from MOSI goes into `read`.
    pub async fn transfer<W: Word>(&mut self, read: &mut [W], write: &[W]) -> Result<(), Error> {
        if read.is_empty() && write.is_empty() {
            return Ok(());
        }

        let r = self.info.regs;

        self.set_word_size(W::CONFIG);
        self.configure_slave_transfer(write.len(), read.len())?;

        // Cache coherency for DMA
        let tx_addr = write.as_ptr() as u32;
        let tx_size = (write.len() * core::mem::size_of::<W>()) as u32;
        let rx_addr = read.as_ptr() as u32;
        let rx_size = (read.len() * core::mem::size_of::<W>()) as u32;

        if tx_size > 0 {
            let tx_aligned_start = andes_riscv::l1c::cacheline_align_down(tx_addr);
            let tx_aligned_size =
                andes_riscv::l1c::cacheline_align_up(tx_size + (tx_addr - tx_aligned_start));
            unsafe {
                andes_riscv::l1c::dc_writeback(tx_aligned_start, tx_aligned_size);
            }
        }

        let rx_aligned_start;
        let rx_aligned_size;
        if rx_size > 0 {
            rx_aligned_start = andes_riscv::l1c::cacheline_align_down(rx_addr);
            rx_aligned_size =
                andes_riscv::l1c::cacheline_align_up(rx_size + (rx_addr - rx_aligned_start));
            unsafe {
                andes_riscv::l1c::dc_invalidate(rx_aligned_start, rx_aligned_size);
            }
        } else {
            rx_aligned_start = 0;
            rx_aligned_size = 0;
        }

        let mut opts = dma::TransferOptions::default();
        opts.burst = dma::Burst::Exponential(0);

        let tx_dst = r.data().as_ptr() as *mut W;
        let rx_src = r.data().as_ptr() as *mut W;

        let tx_f = unsafe { self.tx_dma.as_mut().unwrap().write(write, tx_dst, opts) };
        let rx_f = unsafe { self.rx_dma.as_mut().unwrap().read(rx_src, read, opts) };

        r.ctrl().modify(|w| {
            w.set_rxdmaen(true);
            w.set_txdmaen(true);
        });

        // Set slave ready
        r.slv_st().modify(|w| w.set_ready(true));

        join(tx_f, rx_f).await;

        if rx_size > 0 {
            unsafe {
                andes_riscv::l1c::dc_invalidate(rx_aligned_start, rx_aligned_size);
            }
        }

        r.ctrl().modify(|w| {
            w.set_rxdmaen(false);
            w.set_txdmaen(false);
        });

        self.check_and_clear_slave_errors()
    }

    /// Async write-only slave transfer using DMA (MISO only).
    ///
    /// NOTE: In data-only mode (`data_only: true`), the hardware requires TransMode to remain
    /// WRITE_READ_TOGETHER. TransMode is not switched to WRITE_ONLY. See HPM SDK:
    /// "slave data only mode only works on write read together transfer mode".
    pub async fn write<W: Word>(&mut self, data: &[W]) -> Result<(), Error> {
        if data.is_empty() {
            return Ok(());
        }

        let r = self.info.regs;

        self.set_word_size(W::CONFIG);
        self.configure_slave_transfer(data.len(), 0)?;

        let tx_addr = data.as_ptr() as u32;
        let tx_size = (data.len() * core::mem::size_of::<W>()) as u32;
        let tx_aligned_start = andes_riscv::l1c::cacheline_align_down(tx_addr);
        let tx_aligned_size =
            andes_riscv::l1c::cacheline_align_up(tx_size + (tx_addr - tx_aligned_start));
        unsafe {
            andes_riscv::l1c::dc_writeback(tx_aligned_start, tx_aligned_size);
        }

        let mut opts = dma::TransferOptions::default();
        opts.burst = dma::Burst::Exponential(0);
        let tx_dst = r.data().as_ptr() as *mut W;
        let tx_f = unsafe { self.tx_dma.as_mut().unwrap().write(data, tx_dst, opts) };

        r.ctrl().modify(|w| w.set_txdmaen(true));
        r.slv_st().modify(|w| w.set_ready(true));

        tx_f.await;

        r.ctrl().modify(|w| w.set_txdmaen(false));

        self.check_and_clear_slave_errors()
    }

    /// Async read-only slave transfer using DMA (MOSI only).
    ///
    /// NOTE: In data-only mode (`data_only: true`), the hardware requires TransMode to remain
    /// WRITE_READ_TOGETHER. TransMode is not switched to READ_ONLY. See HPM SDK:
    /// "slave data only mode only works on write read together transfer mode".
    pub async fn read<W: Word>(&mut self, data: &mut [W]) -> Result<(), Error> {
        if data.is_empty() {
            return Ok(());
        }

        let r = self.info.regs;

        self.set_word_size(W::CONFIG);
        self.configure_slave_transfer(0, data.len())?;

        let rx_addr = data.as_ptr() as u32;
        let rx_size = (data.len() * core::mem::size_of::<W>()) as u32;
        let rx_aligned_start = andes_riscv::l1c::cacheline_align_down(rx_addr);
        let rx_aligned_size =
            andes_riscv::l1c::cacheline_align_up(rx_size + (rx_addr - rx_aligned_start));
        unsafe {
            andes_riscv::l1c::dc_invalidate(rx_aligned_start, rx_aligned_size);
        }

        let mut opts = dma::TransferOptions::default();
        opts.burst = dma::Burst::Exponential(0);
        let rx_src = r.data().as_ptr() as *mut W;
        let rx_f = unsafe { self.rx_dma.as_mut().unwrap().read(rx_src, data, opts) };

        r.ctrl().modify(|w| w.set_rxdmaen(true));
        r.slv_st().modify(|w| w.set_ready(true));

        rx_f.await;

        unsafe {
            andes_riscv::l1c::dc_invalidate(rx_aligned_start, rx_aligned_size);
        }

        r.ctrl().modify(|w| w.set_rxdmaen(false));

        self.check_and_clear_slave_errors()
    }
}

// - MARK: Common methods (Blocking + Async)

impl<'d, M: PeriMode> SpiSlave<'d, M> {
    fn new_inner<T: Instance>(
        _peri: Peri<'d, T>,
        sclk: Option<Peri<'d, AnyPin>>,
        mosi: Option<Peri<'d, AnyPin>>,
        miso: Option<Peri<'d, AnyPin>>,
        cs: Peri<'d, AnyPin>,
        tx_dma: Option<ChannelAndRequest<'d>>,
        rx_dma: Option<ChannelAndRequest<'d>>,
        config: SlaveConfig,
    ) -> Self {
        let info = T::info();
        let state = T::state();

        let mut this = Self {
            info,
            state,
            sclk,
            mosi,
            miso,
            cs,
            tx_dma,
            rx_dma,
            current_word_size: <u8 as SealedWord>::CONFIG,
            _phantom: PhantomData,
            data_only: config.data_only,
        };

        this.enable_and_configure(&config);
        this
    }

    fn enable_and_configure(&mut self, config: &SlaveConfig) {
        let r = self.info.regs;

        let cpol = config.mode.polarity == embedded_hal::spi::Polarity::IdleHigh;
        let cpha = config.mode.phase == embedded_hal::spi::Phase::CaptureOnSecondTransition;

        // Configure slave transfer format
        r.trans_fmt().write(|w| {
            w.set_slvmode(true);
            w.set_datalen(<u8 as SealedWord>::CONFIG);
            w.set_datamerge(false);
            w.set_mosibidir(false);
            w.set_lsb(config.bit_order == BitOrder::LsbFirst);
            w.set_cpha(cpha);
            w.set_cpol(cpol);
        });

        // Configure slave transfer control
        r.trans_ctrl().write(|w| {
            w.set_slvdataonly(config.data_only);
            w.set_cmden(false);
            w.set_addren(false);
            w.set_transmode(TransMode::WRITE_READ_TOGETHER);
        });

        // Clear any pending slave status flags
        // OVERRUN is RW — write 0 to clear
        // UNDERRUN is W1C — write 1 to clear
        r.slv_st().write(|w| {
            w.set_overrun(false);
            w.set_underrun(true);
            w.set_ready(false);
        });

        // Reset FIFOs
        r.ctrl().modify(|w| {
            w.set_txfiforst(true);
            w.set_rxfiforst(true);
            w.set_spirst(true);
        });
        while r.ctrl().read().txfiforst() || r.ctrl().read().rxfiforst() || r.ctrl().read().spirst()
        {
        }
    }

    fn set_word_size(&mut self, word_size: super::word_impl::Config) {
        if self.current_word_size == word_size {
            return;
        }
        self.info.regs.trans_fmt().modify(|w| {
            w.set_datalen(word_size);
        });
        self.current_word_size = word_size;
    }

    /// Configure slave transfer parameters (FIFO reset, transfer count).
    fn configure_slave_transfer(&mut self, write_len: usize, read_len: usize) -> Result<(), Error> {
        let max_len = write_len.max(read_len);
        if max_len > TRANSFER_COUNT_MAX {
            return Err(Error::BufferTooLong);
        }

        let r = self.info.regs;

        // Always set both transfer counts unconditionally to avoid stale values
        // from previous transfers. SDK does the same — sets wrtrancnt/rdtrancnt
        // regardless of transfer direction.
        #[cfg(not(ip_feature_spi_new_trans_count))]
        {
            r.trans_ctrl().modify(|w| {
                w.set_wrtrancnt(if write_len > 0 {
                    write_len as u16 - 1
                } else {
                    0
                });
                w.set_rdtrancnt(if read_len > 0 {
                    read_len as u16 - 1
                } else {
                    0
                });
            });
        }
        #[cfg(ip_feature_spi_new_trans_count)]
        {
            r.wr_trans_cnt().write(|w| {
                w.set_wrtrancnt(if write_len > 0 {
                    write_len as u32 - 1
                } else {
                    0
                })
            });
            r.rd_trans_cnt().write(|w| {
                w.set_rdtrancnt(if read_len > 0 {
                    read_len as u32 - 1
                } else {
                    0
                })
            });
        }

        // Reset FIFOs and SPI controller
        r.ctrl().modify(|w| {
            w.set_txfiforst(true);
            w.set_rxfiforst(true);
            w.set_spirst(true);
        });
        while r.ctrl().read().txfiforst() || r.ctrl().read().rxfiforst() || r.ctrl().read().spirst()
        {
        }

        Ok(())
    }

    /// Check and clear slave error flags after a transfer.
    ///
    /// OVERRUN is RW (write 0 to clear), UNDERRUN is W1C (write 1 to clear).
    fn check_and_clear_slave_errors(&self) -> Result<(), Error> {
        let r = self.info.regs;
        let slv_st = r.slv_st().read();

        if slv_st.overrun() {
            // OVERRUN is RW: write 0 to clear the flag
            r.slv_st().modify(|w| w.set_overrun(false));
            return Err(Error::Overrun);
        }
        if slv_st.underrun() {
            // UNDERRUN is W1C: write 1 to clear the flag
            r.slv_st().modify(|w| w.set_underrun(true));
            return Err(Error::Underrun);
        }

        Ok(())
    }

    /// Blocking bidirectional slave transfer (in-place).
    ///
    /// Pre-fills TX FIFO with `data`, sets READY, then waits for the master
    /// to clock the transfer. Received data overwrites `data`.
    pub fn blocking_transfer_in_place<W: Word>(&mut self, data: &mut [W]) -> Result<(), Error> {
        if data.is_empty() {
            return Ok(());
        }

        let r = self.info.regs;

        self.set_word_size(W::CONFIG);
        self.configure_slave_transfer(data.len(), data.len())?;

        flush_rx_fifo(r);

        let len = data.len();
        let mut tx_idx = 0;
        let mut rx_idx = 0;

        // Pre-fill TX FIFO
        while tx_idx < len && tx_idx < FIFO_SIZE {
            if r.status().read().txfull() {
                break;
            }
            unsafe {
                ptr::write_volatile(r.data().as_ptr() as *mut W, data[tx_idx]);
            }
            tx_idx += 1;
        }

        // Signal ready to master
        r.slv_st().modify(|w| w.set_ready(true));

        // Wait for master to start (CS active)
        while !r.status().read().spiactive() {}

        // Transfer loop: interleave TX and RX
        loop {
            let status = r.status().read();

            if tx_idx < len && !status.txfull() {
                unsafe {
                    ptr::write_volatile(r.data().as_ptr() as *mut W, data[tx_idx]);
                }
                tx_idx += 1;
            }

            if rx_idx < len && !status.rxempty() {
                data[rx_idx] = unsafe { ptr::read_volatile(r.data().as_ptr() as *const W) };
                rx_idx += 1;
            }

            if tx_idx >= len && rx_idx >= len {
                break;
            }

            // Safety: if transfer ended and we still have RX to drain
            if !status.spiactive() && tx_idx >= len {
                while rx_idx < len && !r.status().read().rxempty() {
                    data[rx_idx] = unsafe { ptr::read_volatile(r.data().as_ptr() as *const W) };
                    rx_idx += 1;
                }
                break;
            }
        }

        self.check_and_clear_slave_errors()
    }

    /// Blocking bidirectional slave transfer.
    ///
    /// `write` data is sent on MISO, received data from MOSI goes into `read`.
    pub fn blocking_transfer<W: Word>(
        &mut self,
        read: &mut [W],
        write: &[W],
    ) -> Result<(), Error> {
        let r = self.info.regs;

        let write_len = write.len();
        let read_len = read.len();
        if write_len == 0 && read_len == 0 {
            return Ok(());
        }

        self.set_word_size(W::CONFIG);
        self.configure_slave_transfer(write_len, read_len)?;

        flush_rx_fifo(r);

        let mut tx_idx = 0;
        let mut rx_idx = 0;

        // Pre-fill TX FIFO
        while tx_idx < write_len && tx_idx < FIFO_SIZE {
            if r.status().read().txfull() {
                break;
            }
            unsafe {
                ptr::write_volatile(r.data().as_ptr() as *mut W, write[tx_idx]);
            }
            tx_idx += 1;
        }

        r.slv_st().modify(|w| w.set_ready(true));

        // Wait for master to start
        while !r.status().read().spiactive() {}

        loop {
            let status = r.status().read();

            if tx_idx < write_len && !status.txfull() {
                unsafe {
                    ptr::write_volatile(r.data().as_ptr() as *mut W, write[tx_idx]);
                }
                tx_idx += 1;
            }

            if rx_idx < read_len && !status.rxempty() {
                read[rx_idx] = unsafe { ptr::read_volatile(r.data().as_ptr() as *const W) };
                rx_idx += 1;
            }

            if tx_idx >= write_len && rx_idx >= read_len {
                break;
            }

            if !status.spiactive() && tx_idx >= write_len {
                while rx_idx < read_len && !r.status().read().rxempty() {
                    read[rx_idx] = unsafe { ptr::read_volatile(r.data().as_ptr() as *const W) };
                    rx_idx += 1;
                }
                break;
            }
        }

        self.check_and_clear_slave_errors()
    }

    /// Blocking write-only slave transfer (TX on MISO).
    ///
    /// NOTE: In data-only mode (`data_only: true`), the hardware requires TransMode to remain
    /// WRITE_READ_TOGETHER. TransMode is not switched to WRITE_ONLY. See HPM SDK:
    /// "slave data only mode only works on write read together transfer mode".
    pub fn blocking_write<W: Word>(&mut self, data: &[W]) -> Result<(), Error> {
        if data.is_empty() {
            return Ok(());
        }

        let r = self.info.regs;

        self.set_word_size(W::CONFIG);

        // Only switch TransMode when NOT in data-only mode.
        // In data-only mode, hardware mandates WRITE_READ_TOGETHER.
        if !self.data_only {
            r.trans_ctrl().modify(|w| {
                w.set_transmode(TransMode::WRITE_ONLY);
            });
        }

        self.configure_slave_transfer(data.len(), 0)?;

        let mut tx_idx = 0;

        while tx_idx < data.len() && tx_idx < FIFO_SIZE {
            if r.status().read().txfull() {
                break;
            }
            unsafe {
                ptr::write_volatile(r.data().as_ptr() as *mut W, data[tx_idx]);
            }
            tx_idx += 1;
        }

        r.slv_st().modify(|w| w.set_ready(true));
        while !r.status().read().spiactive() {}

        while tx_idx < data.len() {
            while r.status().read().txfull() {}
            unsafe {
                ptr::write_volatile(r.data().as_ptr() as *mut W, data[tx_idx]);
            }
            tx_idx += 1;
        }

        while r.status().read().spiactive() {}

        // Restore default transmode if changed
        if !self.data_only {
            r.trans_ctrl().modify(|w| {
                w.set_transmode(TransMode::WRITE_READ_TOGETHER);
            });
        }

        self.check_and_clear_slave_errors()
    }

    /// Blocking read-only slave transfer (RX from MOSI).
    ///
    /// NOTE: In data-only mode (`data_only: true`), the hardware requires TransMode to remain
    /// WRITE_READ_TOGETHER. TransMode is not switched to READ_ONLY. See HPM SDK:
    /// "slave data only mode only works on write read together transfer mode".
    pub fn blocking_read<W: Word>(&mut self, data: &mut [W]) -> Result<(), Error> {
        if data.is_empty() {
            return Ok(());
        }

        let r = self.info.regs;

        self.set_word_size(W::CONFIG);

        // Only switch TransMode when NOT in data-only mode.
        if !self.data_only {
            r.trans_ctrl().modify(|w| {
                w.set_transmode(TransMode::READ_ONLY);
            });
        }

        self.configure_slave_transfer(0, data.len())?;

        flush_rx_fifo(r);

        r.slv_st().modify(|w| w.set_ready(true));
        while !r.status().read().spiactive() {}

        for b in data.iter_mut() {
            while r.status().read().rxempty() {}
            *b = unsafe { ptr::read_volatile(r.data().as_ptr() as *const W) };
        }

        // Restore default transmode if changed
        if !self.data_only {
            r.trans_ctrl().modify(|w| {
                w.set_transmode(TransMode::WRITE_READ_TOGETHER);
            });
        }

        self.check_and_clear_slave_errors()
    }

    /// Read the current slave status.
    pub fn slave_status(&self) -> SlaveStatus {
        let r = self.info.regs;
        let slv_st = r.slv_st().read();
        let slv_data_cnt = r.slv_data_cnt().read();

        SlaveStatus {
            ready: slv_st.ready(),
            overrun: slv_st.overrun(),
            underrun: slv_st.underrun(),
            user_status: slv_st.usr_status(),
            rx_count: slv_data_cnt.rcnt(),
            tx_count: slv_data_cnt.wcnt(),
        }
    }

    /// Set user-defined status visible to the master (16-bit).
    ///
    /// In command-aware mode, the master can read this via a status read command.
    pub fn set_user_status(&mut self, status: u16) {
        self.info.regs.slv_st().modify(|w| {
            w.set_usr_status(status);
        });
    }

    /// Set slave ready flag.
    ///
    /// When ready=true, the slave indicates it can accept the next transaction.
    pub fn set_ready(&mut self, ready: bool) {
        self.info.regs.slv_st().modify(|w| {
            w.set_ready(ready);
        });
    }
}
