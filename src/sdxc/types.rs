//! SDXC types and data structures

// Re-export sdio-host types
pub use sdio_host::common_cmd::{self, Cmd, Resp, ResponseLen};
pub use sdio_host::sd::{CardCapacity, CardStatus, CIC, CID, CSD, OCR, RCA, SCR, SD, SDStatus};
pub use sdio_host::sd_cmd;

/// SD Card information
#[derive(Debug, Clone, Default)]
pub struct Card {
    /// Card type (SDSC/SDHC/SDXC)
    pub card_type: CardCapacity,
    /// Operation Conditions Register
    pub ocr: OCR<SD>,
    /// Relative Card Address
    pub rca: u16,
    /// Card ID
    pub cid: CID<SD>,
    /// Card Specific Data
    pub csd: CSD<SD>,
    /// SD Card Configuration Register
    pub scr: SCR,
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

/// SDXC errors
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// Hardware timeout (card didn't respond)
    Timeout,
    /// Software timeout (polling loop exceeded)
    SoftwareTimeout,
    /// CRC error
    Crc,
    /// Command end bit error
    CmdEndBit,
    /// Command index error
    CmdIndex,
    /// Data CRC error
    DataCrc,
    /// Data timeout
    DataTimeout,
    /// No card inserted
    NoCard,
    /// Unsupported card version
    UnsupportedCardVersion,
    /// ADMA error
    AdmaError,
}

// ============================================================================
// ADMA2 Descriptor support
// ============================================================================

/// ADMA2 descriptor action type
#[repr(u8)]
#[derive(Clone, Copy, Debug)]
pub enum Adma2Action {
    /// No operation
    Nop = 0,
    /// Set data length (reserved)
    SetLen = 1,
    /// Transfer data
    Transfer = 2,
    /// Link to another descriptor
    Link = 3,
}

/// ADMA2 descriptor (8 bytes, must be 4-byte aligned)
///
/// Layout:
/// - Bits 0: Valid (must be 1 for valid descriptor)
/// - Bit 1: End (last descriptor in chain)
/// - Bit 2: Int (generate interrupt after this descriptor)
/// - Bits 5:4: Action (0=Nop, 1=SetLen, 2=Transfer, 3=Link)
/// - Bits 15:6: Reserved
/// - Bits 31:16: Length (lower 16 bits)
/// - Bits 63:32: Address (32-bit)
#[repr(C, align(4))]
#[derive(Clone, Copy, Debug)]
pub struct Adma2Descriptor {
    /// Attribute and length field
    pub attr_len: u32,
    /// Address field (32-bit)
    pub addr: u32,
}

impl Adma2Descriptor {
    /// ADMA2 descriptor flags
    const VALID: u32 = 1 << 0;
    const END: u32 = 1 << 1;
    const INT: u32 = 1 << 2;
    const ACT_TRANSFER: u32 = 2 << 4;
    const ACT_LINK: u32 = 3 << 4;

    /// Create a new empty (invalid) descriptor
    pub const fn new() -> Self {
        Self { attr_len: 0, addr: 0 }
    }

    /// Create a transfer descriptor
    ///
    /// - `addr`: Buffer address (must be 4-byte aligned)
    /// - `len`: Transfer length in bytes (max 65535 in standard mode, or 26-bit in extended mode)
    /// - `end`: Set to true for last descriptor in chain
    pub const fn transfer(addr: u32, len: u16, end: bool) -> Self {
        let mut attr_len = Self::VALID | Self::ACT_TRANSFER | ((len as u32) << 16);
        if end {
            attr_len |= Self::END;
        }
        Self { attr_len, addr }
    }

    /// Create a transfer descriptor with interrupt
    pub const fn transfer_with_int(addr: u32, len: u16, end: bool) -> Self {
        let mut desc = Self::transfer(addr, len, end);
        desc.attr_len |= Self::INT;
        desc
    }

    /// Create a link descriptor (to chain descriptor tables)
    pub const fn link(next_desc_addr: u32) -> Self {
        Self {
            attr_len: Self::VALID | Self::ACT_LINK,
            addr: next_desc_addr,
        }
    }

    /// Check if descriptor is valid
    pub const fn is_valid(&self) -> bool {
        (self.attr_len & Self::VALID) != 0
    }

    /// Check if this is the last descriptor
    pub const fn is_end(&self) -> bool {
        (self.attr_len & Self::END) != 0
    }
}

impl Default for Adma2Descriptor {
    fn default() -> Self {
        Self::new()
    }
}

/// ADMA2 descriptor table for multi-block transfers
///
/// Supports up to 16 descriptors = 16 blocks = 8KB per transfer
/// Must be placed in DMA-accessible memory (not in TCM on some chips)
#[repr(C, align(4))]
pub struct Adma2Table<const N: usize = 16> {
    /// Descriptor array
    pub descriptors: [Adma2Descriptor; N],
}

impl<const N: usize> Adma2Table<N> {
    /// Create a new empty descriptor table
    pub const fn new() -> Self {
        Self {
            descriptors: [Adma2Descriptor::new(); N],
        }
    }

    /// Setup descriptors for reading multiple blocks into a contiguous buffer
    ///
    /// Returns the number of descriptors used
    pub fn setup_read(&mut self, buffer: &mut [DataBlock]) -> usize {
        let count = buffer.len().min(N);
        for (i, block) in buffer.iter_mut().take(count).enumerate() {
            let is_last = i == count - 1;
            self.descriptors[i] = Adma2Descriptor::transfer(
                block.0.as_ptr() as u32,
                512,
                is_last,
            );
        }
        count
    }

    /// Setup descriptors for writing multiple blocks from a contiguous buffer
    ///
    /// Returns the number of descriptors used
    pub fn setup_write(&mut self, buffer: &[DataBlock]) -> usize {
        let count = buffer.len().min(N);
        for (i, block) in buffer.iter().take(count).enumerate() {
            let is_last = i == count - 1;
            self.descriptors[i] = Adma2Descriptor::transfer(
                block.0.as_ptr() as u32,
                512,
                is_last,
            );
        }
        count
    }

    /// Get pointer to first descriptor (for setting ADMA_SYS_ADDR)
    pub fn as_ptr(&self) -> *const Adma2Descriptor {
        self.descriptors.as_ptr()
    }
}

impl<const N: usize> Default for Adma2Table<N> {
    fn default() -> Self {
        Self::new()
    }
}
