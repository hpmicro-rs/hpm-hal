//! I2S, Inter-IC Sound
//!
//! HPM I2S features:
//! - 4 data lines (TXD[0-3], RXD[0-3])
//! - TDM mode support
//! - Master/Slave mode
//! - DMA transfer support

use embassy_hal_internal::{Peri, PeripheralType};

use crate::pac::i2s::vals::{ChannelSize, DataSize, Std};
use crate::pac::i2s::I2s;

// - MARK: Config types

/// I2S operating mode
#[derive(Copy, Clone, Debug, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Mode {
    #[default]
    Master,
    Slave,
}

/// I2S protocol standard
#[derive(Copy, Clone, Debug, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Standard {
    /// I2S Philips standard
    #[default]
    Philips,
    /// MSB justified (left justified)
    MsbJustified,
    /// LSB justified (right justified)
    LsbJustified,
    /// PCM standard
    Pcm,
}

impl Standard {
    fn to_pac(self) -> Std {
        match self {
            Standard::Philips => Std::PHILIPS,
            Standard::MsbJustified => Std::MSB,
            Standard::LsbJustified => Std::LSB,
            Standard::Pcm => Std::PCM,
        }
    }
}

/// Data format (data bits / channel bits)
#[derive(Copy, Clone, Debug, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Format {
    /// 16-bit data, 16-bit channel
    Data16Channel16,
    /// 16-bit data, 32-bit channel
    #[default]
    Data16Channel32,
    /// 24-bit data, 32-bit channel
    Data24Channel32,
    /// 32-bit data, 32-bit channel
    Data32Channel32,
}

impl Format {
    fn data_size(self) -> DataSize {
        match self {
            Format::Data16Channel16 | Format::Data16Channel32 => DataSize::_16BIT,
            Format::Data24Channel32 => DataSize::_24BIT,
            Format::Data32Channel32 => DataSize::_32BIT,
        }
    }

    fn channel_size(self) -> ChannelSize {
        match self {
            Format::Data16Channel16 => ChannelSize::_16BIT,
            _ => ChannelSize::_32BIT,
        }
    }

    /// Returns the number of bytes per sample
    pub fn bytes_per_sample(self) -> usize {
        match self {
            Format::Data16Channel16 | Format::Data16Channel32 => 2,
            Format::Data24Channel32 | Format::Data32Channel32 => 4,
        }
    }
}

/// Data line selection
#[derive(Copy, Clone, Debug, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DataLine {
    #[default]
    Line0 = 0,
    Line1 = 1,
    Line2 = 2,
    Line3 = 3,
}

/// I2S error types
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// Write called on RX-only instance
    NotATransmitter,
    /// Read called on TX-only instance
    NotAReceiver,
    /// RX FIFO overrun
    Overrun,
    /// TX FIFO underrun
    Underrun,
}

/// I2S configuration
#[derive(Clone, Copy)]
pub struct Config {
    /// Sample rate in Hz (e.g., 44100, 48000)
    pub sample_rate: u32,
    /// Master or Slave mode
    pub mode: Mode,
    /// Protocol standard
    pub standard: Standard,
    /// Data format
    pub format: Format,
    /// Enable master clock output
    pub master_clock: bool,
    /// TX FIFO threshold (0-31)
    pub tx_fifo_threshold: u8,
    /// RX FIFO threshold (0-31)
    pub rx_fifo_threshold: u8,
    /// Enable TDM mode
    pub enable_tdm: bool,
    /// Number of channels per frame (2-16, must be even)
    pub channel_num: u8,
    /// Channel slot mask (which slots are active)
    pub channel_slot_mask: u32,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            sample_rate: 48000,
            mode: Mode::Master,
            standard: Standard::Philips,
            format: Format::Data16Channel32,
            master_clock: true,
            tx_fifo_threshold: 4,
            rx_fifo_threshold: 4,
            enable_tdm: false,
            channel_num: 2,
            channel_slot_mask: 0x3, // channels 0 and 1
        }
    }
}

// - MARK: Pin traits

/// Trait for MCLK pins
pub trait MclkPin<T: Instance>: crate::gpio::Pin {
    fn alt_num(&self) -> u8;
}

/// Trait for BCLK pins
pub trait BclkPin<T: Instance>: crate::gpio::Pin {
    fn alt_num(&self) -> u8;
}

/// Trait for FCLK (frame clock / LRCK) pins
pub trait FclkPin<T: Instance>: crate::gpio::Pin {
    fn alt_num(&self) -> u8;
}

/// Trait for TXD pins
pub trait TxdPin<T: Instance>: crate::gpio::Pin {
    fn alt_num(&self) -> u8;
    fn line(&self) -> DataLine;
}

/// Trait for RXD pins
pub trait RxdPin<T: Instance>: crate::gpio::Pin {
    fn alt_num(&self) -> u8;
    fn line(&self) -> DataLine;
}

// - MARK: DMA traits

/// Trait for I2S TX DMA
#[allow(private_bounds)]
pub trait TxDma<T: Instance>: crate::dma::Channel {}

/// Trait for I2S RX DMA
#[allow(private_bounds)]
pub trait RxDma<T: Instance>: crate::dma::Channel {}

// - MARK: Instance trait

pub(crate) trait SealedInstance {
    fn regs() -> I2s;
}

/// I2S instance trait
#[allow(private_bounds)]
pub trait Instance: SealedInstance + PeripheralType + crate::sysctl::ClockPeripheral + 'static {
    /// Interrupt for this I2S instance
    type Interrupt: crate::interrupt::typelevel::Interrupt;
}

// - MARK: Driver

/// I2S driver (dry-run / blocking mode for now)
pub struct I2S<'d, T: Instance> {
    _peri: Peri<'d, T>,
    tx_line: Option<DataLine>,
    rx_line: Option<DataLine>,
}

impl<'d, T: Instance> I2S<'d, T> {
    /// Create a new I2S driver in TX-only mode (blocking, for dry-run testing)
    pub fn new_txonly_blocking(
        peri: Peri<'d, T>,
        _txd: Peri<'d, impl TxdPin<T>>,
        _bclk: Peri<'d, impl BclkPin<T>>,
        _fclk: Peri<'d, impl FclkPin<T>>,
        _mclk: Peri<'d, impl MclkPin<T>>,
        config: Config,
    ) -> Self {
        // Enable peripheral clock
        T::add_resource_group(0);

        let regs = T::regs();

        // Disable I2S first
        regs.ctrl().modify(|w| w.set_i2s_en(false));

        // Reset TX
        regs.ctrl().modify(|w| {
            w.set_sftrst_tx(true);
            w.set_txfifoclr(true);
        });
        regs.ctrl().modify(|w| {
            w.set_sftrst_tx(false);
            w.set_txfifoclr(false);
        });

        // Configure FIFO threshold
        regs.fifo_thresh().modify(|w| {
            w.set_tx(config.tx_fifo_threshold);
            w.set_rx(config.rx_fifo_threshold);
        });

        // Configure I2S format
        regs.cfgr().modify(|w| {
            w.set_datsiz(config.format.data_size());
            w.set_chsiz(config.format.channel_size());
            w.set_std(config.standard.to_pac());
            w.set_tdm_en(config.enable_tdm);
            w.set_ch_max(config.channel_num);
        });

        // Configure MCLK output
        regs.misc_cfgr().modify(|w| {
            w.set_mclkoe(config.master_clock);
        });

        // Set slot mask for TX line 0
        regs.txdslot(0).write(|w| w.set_en(config.channel_slot_mask as u16));

        // Enable TX line 0
        regs.ctrl().modify(|w| {
            w.set_tx_en(1); // Enable line 0
        });

        defmt::info!("I2S configured: sample_rate={}, format={:?}", config.sample_rate, config.format);

        Self {
            _peri: peri,
            tx_line: Some(DataLine::Line0),
            rx_line: None,
        }
    }

    /// Start I2S transfer
    pub fn start(&mut self) {
        let regs = T::regs();
        regs.ctrl().modify(|w| w.set_i2s_en(true));
        defmt::info!("I2S started");
    }

    /// Stop I2S transfer
    pub fn stop(&mut self) {
        let regs = T::regs();
        regs.ctrl().modify(|w| w.set_i2s_en(false));
        defmt::info!("I2S stopped");
    }

    /// Get TX FIFO level for the configured line
    pub fn tx_fifo_level(&self) -> u8 {
        let regs = T::regs();
        let fillings = regs.tfifo_fillings().read();
        match self.tx_line.unwrap_or(DataLine::Line0) {
            DataLine::Line0 => fillings.tx0(),
            DataLine::Line1 => fillings.tx1(),
            DataLine::Line2 => fillings.tx2(),
            DataLine::Line3 => fillings.tx3(),
        }
    }

    /// Send a single data word (blocking)
    pub fn send_blocking(&mut self, data: u32) -> Result<(), Error> {
        let regs = T::regs();
        let line = self.tx_line.ok_or(Error::NotATransmitter)?;

        // Wait for FIFO space
        while self.tx_fifo_level() >= 8 {
            core::hint::spin_loop();
        }

        regs.txd(line as usize).write(|w| w.set_d(data));
        Ok(())
    }

    /// Check if I2S is enabled
    pub fn is_enabled(&self) -> bool {
        T::regs().ctrl().read().i2s_en()
    }
}

impl<'d, T: Instance> Drop for I2S<'d, T> {
    fn drop(&mut self) {
        self.stop();
    }
}

// - MARK: Instance macro

macro_rules! impl_i2s {
    ($inst:ident, $irq:ident) => {
        impl SealedInstance for crate::peripherals::$inst {
            fn regs() -> I2s {
                crate::pac::$inst
            }
        }

        impl Instance for crate::peripherals::$inst {
            type Interrupt = crate::interrupt::typelevel::$irq;
        }
    };
}

// HPM67xx I2S instances
#[cfg(peri_i2s0)]
impl_i2s!(I2S0, I2S0);
#[cfg(peri_i2s1)]
impl_i2s!(I2S1, I2S1);
#[cfg(peri_i2s2)]
impl_i2s!(I2S2, I2S2);
#[cfg(peri_i2s3)]
impl_i2s!(I2S3, I2S3);

