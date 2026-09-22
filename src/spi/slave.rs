//! Blocking SPI slave driver.
//!
//! This driver implements the HPM SDK data-only polling model: each transaction
//! is a raw, equal-length, full-duplex transfer using
//! [`TransMode::WRITE_READ_TOGETHER`]. The external master owns SCLK and CS.
//! Command-framed transfers and DMA are outside this API's current scope.
//!
//! The register sequence follows `samples/drivers/spi/polling/slave` and
//! `drivers/src/hpm_spi_drv.c` from HPM SDK. This Rust API deliberately exposes
//! one equal-length full-duplex operation and validates the hardware count
//! constraint before touching the peripheral.

use core::marker::PhantomData;
use core::ptr;

use embassy_hal_internal::Peri;

use super::consts::*;
use super::{BitOrder, CsIndexPin, CsPin, Info, Instance, MisoPin, MosiPin, SclkPin, SealedWord, Word};
use crate::gpio::AnyPin;
use crate::mode::{Blocking, Mode as PeriMode};
use crate::pac::spi::vals::TransMode;

/// SPI slave configuration.
#[derive(Clone, Copy)]
pub struct Config {
    /// SPI mode (CPOL/CPHA). This must match the external master.
    pub mode: embedded_hal::spi::Mode,
    /// Bit transmission order. This must match the external master.
    pub bit_order: BitOrder,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            mode: super::MODE_0,
            bit_order: BitOrder::MsbFirst,
        }
    }
}

/// SPI slave transfer error.
#[derive(Debug, PartialEq, Eq, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// The buffers have different lengths.
    LengthMismatch,
    /// The transfer length exceeds the hardware counter range.
    BufferTooLong,
    /// The master clocked more receive data than the peripheral could hold.
    Overrun,
    /// The master requested transmit data before it was available.
    Underrun,
    /// The master released CS before transferring the requested number of words.
    ShortTransfer,
}

/// Actual hardware transfer counters.
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Status {
    /// Number of words received in the current or last transaction.
    pub rx_count: u32,
    /// Number of words transmitted in the current or last transaction.
    pub tx_count: u32,
}

/// Blocking raw data-only SPI slave.
///
/// Transfers are equal-length and full-duplex. The driver sets READY after the
/// FIFOs and transfer counters are prepared, then waits for a complete CS
/// transaction boundary.
#[allow(unused)]
pub struct SpiSlave<'d, M: PeriMode = Blocking> {
    info: &'static Info,
    sclk: Peri<'d, AnyPin>,
    mosi: Peri<'d, AnyPin>,
    miso: Peri<'d, AnyPin>,
    cs: Peri<'d, AnyPin>,
    current_word_size: super::word_impl::Config,
    _mode: PhantomData<M>,
}

impl<'d> SpiSlave<'d, Blocking> {
    /// Create a blocking raw data-only SPI slave.
    ///
    /// SCLK, MOSI, and CS are configured as inputs. MISO is configured as an
    /// output. On SPI IPs with selectable chip selects, the selected CS input is
    /// derived from the supplied CS pin.
    pub fn new<T: Instance>(
        _peri: Peri<'d, T>,
        sclk: Peri<'d, impl SclkPin<T>>,
        mosi: Peri<'d, impl MosiPin<T>>,
        miso: Peri<'d, impl MisoPin<T>>,
        cs: Peri<'d, impl CsPin<T> + CsIndexPin<T>>,
        config: Config,
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

        #[cfg(ip_feature_spi_cs_select)]
        let cs_index = cs.cs_index();

        let mut this = Self {
            info: T::info(),
            sclk: sclk.into(),
            mosi: mosi.into(),
            miso: miso.into(),
            cs: cs.into(),
            current_word_size: <u8 as SealedWord>::CONFIG,
            _mode: PhantomData,
        };

        #[cfg(ip_feature_spi_cs_select)]
        this.info.regs.ctrl().modify(|w| w.set_cs_en(cs_index));

        this.configure(config);
        this
    }

    /// Perform one equal-length full-duplex data-only transaction.
    ///
    /// `write` is transmitted on MISO while MOSI is copied into `read`. The
    /// method returns after the master releases CS. A short CS transaction
    /// returns [`Error::ShortTransfer`] after restoring the peripheral for the
    /// next call. Short-transfer detection requires the polling loop to observe
    /// either CS active or the transaction END flag.
    pub fn blocking_transfer<W: Word>(&mut self, read: &mut [W], write: &[W]) -> Result<(), Error> {
        if read.len() != write.len() {
            return Err(Error::LengthMismatch);
        }
        if read.is_empty() {
            return Ok(());
        }
        let count = u32::try_from(read.len()).map_err(|_| Error::BufferTooLong)?;
        if read.len() > TRANSFER_COUNT_MAX {
            return Err(Error::BufferTooLong);
        }

        self.set_word_size(W::CONFIG);
        self.prepare_transaction(count);

        let r = self.info.regs;
        let len = read.len();
        let mut tx_index = 0;
        let mut rx_index = 0;

        while tx_index < len && tx_index < FIFO_SIZE && !r.status().read().txfull() {
            unsafe { ptr::write_volatile(r.data().as_ptr() as *mut W, write[tx_index]) };
            tx_index += 1;
        }

        self.set_ready_preserving_status(true);

        let mut active_seen = false;
        loop {
            let peripheral_status = r.status().read();
            let transaction_ended = r.intr_st().read().endint();

            if peripheral_status.spiactive() {
                active_seen = true;
            }

            if !peripheral_status.spiactive() && (active_seen || transaction_ended) {
                break;
            }

            while !transaction_ended && tx_index < len && !r.status().read().txfull() {
                unsafe { ptr::write_volatile(r.data().as_ptr() as *mut W, write[tx_index]) };
                tx_index += 1;
            }

            while rx_index < len && !r.status().read().rxempty() {
                read[rx_index] = unsafe { ptr::read_volatile(r.data().as_ptr() as *const W) };
                rx_index += 1;
            }
        }

        while rx_index < len && !r.status().read().rxempty() {
            read[rx_index] = unsafe { ptr::read_volatile(r.data().as_ptr() as *const W) };
            rx_index += 1;
        }

        let slave_status = r.slv_st().read();
        let status = self.status();
        self.finish_transaction(slave_status.underrun());

        if slave_status.overrun() {
            return Err(Error::Overrun);
        }
        if slave_status.underrun() {
            return Err(Error::Underrun);
        }
        if rx_index < len || status.rx_count < len as u32 || status.tx_count < len as u32 {
            return Err(Error::ShortTransfer);
        }

        Ok(())
    }
}

impl<'d, M: PeriMode> SpiSlave<'d, M> {
    fn configure(&mut self, config: Config) {
        let r = self.info.regs;
        let cpol = config.mode.polarity == embedded_hal::spi::Polarity::IdleHigh;
        let cpha = config.mode.phase == embedded_hal::spi::Phase::CaptureOnSecondTransition;

        r.trans_fmt().write(|w| {
            w.set_slvmode(true);
            w.set_datalen(<u8 as SealedWord>::CONFIG);
            w.set_datamerge(false);
            w.set_mosibidir(false);
            w.set_lsb(config.bit_order == BitOrder::LsbFirst);
            w.set_cpha(cpha);
            w.set_cpol(cpol);
        });

        r.trans_ctrl().write(|w| {
            w.set_slvdataonly(true);
            w.set_cmden(false);
            w.set_addren(false);
            w.set_transmode(TransMode::WRITE_READ_TOGETHER);
        });

        let user_status = r.slv_st().read().usr_status();
        self.write_slave_status(user_status, false, false, true);
        self.reset_controller_and_fifos();
    }

    fn set_word_size(&mut self, word_size: super::word_impl::Config) {
        if self.current_word_size != word_size {
            self.info.regs.trans_fmt().modify(|w| w.set_datalen(word_size));
            self.current_word_size = word_size;
        }
    }

    fn prepare_transaction(&mut self, count: u32) {
        let r = self.info.regs;

        r.trans_ctrl().modify(|w| {
            w.set_slvdataonly(true);
            w.set_cmden(false);
            w.set_addren(false);
            w.set_transmode(TransMode::WRITE_READ_TOGETHER);
            #[cfg(not(ip_feature_spi_new_trans_count))]
            {
                w.set_wrtrancnt(count as u16 - 1);
                w.set_rdtrancnt(count as u16 - 1);
            }
        });

        #[cfg(ip_feature_spi_new_trans_count)]
        {
            r.wr_trans_cnt().write(|w| w.set_wrtrancnt(count - 1));
            r.rd_trans_cnt().write(|w| w.set_rdtrancnt(count - 1));
        }

        self.reset_controller_and_fifos();

        let user_status = r.slv_st().read().usr_status();
        self.write_slave_status(user_status, false, false, true);
        r.intr_st().write(|w| w.set_endint(true));
    }

    fn reset_controller_and_fifos(&self) {
        let r = self.info.regs;
        r.ctrl().modify(|w| {
            w.set_txfiforst(true);
            w.set_rxfiforst(true);
            w.set_spirst(true);
        });
        while {
            let ctrl = r.ctrl().read();
            ctrl.txfiforst() || ctrl.rxfiforst() || ctrl.spirst()
        } {}
    }

    fn write_slave_status(&self, user_status: u16, ready: bool, overrun: bool, clear_underrun: bool) {
        self.info.regs.slv_st().write(|w| {
            w.set_usr_status(user_status);
            w.set_ready(ready);
            w.set_overrun(overrun);
            w.set_underrun(clear_underrun);
        });
    }

    fn set_ready_preserving_status(&self, ready: bool) {
        let status = self.info.regs.slv_st().read();
        self.write_slave_status(status.usr_status(), ready, status.overrun(), false);
    }

    fn finish_transaction(&self, clear_underrun: bool) {
        let user_status = self.info.regs.slv_st().read().usr_status();
        self.write_slave_status(user_status, false, false, clear_underrun);
        self.info.regs.intr_st().write(|w| w.set_endint(true));
    }

    /// Read the actual hardware transfer counts for the current or last transaction.
    pub fn status(&self) -> Status {
        let r = self.info.regs;

        #[cfg(ip_feature_spi_new_trans_count)]
        let (rx_count, tx_count) = (r.slv_data_rcnt().read().val(), r.slv_data_wcnt().read().val());
        #[cfg(not(ip_feature_spi_new_trans_count))]
        let (rx_count, tx_count) = {
            let count = r.slv_data_cnt().read();
            (count.rcnt() as u32, count.wcnt() as u32)
        };

        Status { rx_count, tx_count }
    }
}
