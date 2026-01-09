//! SDXC types and data structures

// Re-export sdio-host types
pub use sdio_host::sd::{CardCapacity, CardType, SCR, SDStatus};
pub use sdio_host::emmc::ExtCSD;
pub use sdio_host::common_cmd::ResponseLen;

// Simplified Card type (will use sdio-host's later when we implement full protocol)
#[derive(Debug)]
pub struct Card {
    pub card_type: CardCapacity,
    pub rca: u16,
    pub card_info: CardType,
    pub scr: SCR,
}

// Simplified Emmc type
#[derive(Debug)]
pub struct Emmc {
    pub ext_csd: ExtCSD,
}

/// Data block, aligned to 4 bytes for DMA transfers
#[repr(align(4))]
#[derive(Clone, Copy)]
pub struct DataBlock(pub [u8; 512]);

impl DataBlock {
    /// Create a new data block with all zeros
    pub const fn new() -> Self {
        Self([0u8; 512])
    }

    /// Create a data block from a slice
    pub fn from_slice(slice: &[u8]) -> Self {
        let mut block = Self::new();
        let len = slice.len().min(512);
        block.0[..len].copy_from_slice(&slice[..len]);
        block
    }

    /// Get a reference to the data as a slice
    pub fn as_slice(&self) -> &[u8] {
        &self.0
    }

    /// Get a mutable reference to the data as a slice
    pub fn as_mut_slice(&mut self) -> &mut [u8] {
        &mut self.0
    }
}

impl Default for DataBlock {
    fn default() -> Self {
        Self::new()
    }
}

impl core::ops::Index<usize> for DataBlock {
    type Output = u8;

    fn index(&self, index: usize) -> &Self::Output {
        &self.0[index]
    }
}

impl core::ops::IndexMut<usize> for DataBlock {
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        &mut self.0[index]
    }
}

/// SDXC configuration
#[non_exhaustive]
#[derive(Clone, Copy)]
pub struct Config {
    /// Data timeout in clock cycles
    pub data_timeout: u32,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            data_timeout: 5_000_000,
        }
    }
}

/// Bus width
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum BusWidth {
    /// 1-bit mode
    One = 0,
    /// 4-bit mode
    Four = 1,
    /// 8-bit mode (eMMC only)
    Eight = 2,
}

/// Reset type
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ResetType {
    /// Reset all
    All,
    /// Reset command line
    Cmd,
    /// Reset data line
    Data,
}

/// SDXC errors
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// Timeout during command execution
    CmdTimeout,
    /// CRC error in command response
    CmdCrc,
    /// Command index error
    CmdIndex,
    /// Command end bit error
    CmdEndBit,
    /// Timeout during data transfer
    DataTimeout,
    /// CRC error in data transfer
    DataCrc,
    /// Data end bit error
    DataEndBit,
    /// Current limit error
    CurrentLimit,
    /// Auto CMD error
    AutoCmd,
    /// ADMA error
    Adma,
    /// Tuning error
    TuningFailed,
    /// General timeout
    Timeout,
    /// No card inserted
    NoCard,
    /// Unsupported card type
    UnsupportedCard,
    /// Invalid address
    InvalidAddress,
    /// ADMA descriptor table not set
    NoAdmaTable,
    /// Invalid parameter
    InvalidParameter,
}
