//! Type-level SDRAM bus geometry.
//!
//! These marker types connect an [`SdramChip`](super::SdramChip) definition to
//! the address, data, and bank pin tuples accepted by
//! [`Sdram::new_cs0`](super::Sdram::new_cs0). This lets the compiler reject a
//! pin set whose bus geometry does not match the selected SDRAM chip.

use super::{Bank2Sel, SdramPortSize};

/// Marker for an SDRAM requiring address pins A0-A10.
pub struct Address11;

/// Marker for an SDRAM requiring address pins A0-A11.
pub struct Address12;

/// Marker for an SDRAM requiring address pins A0-A12.
pub struct Address13;

/// Marker for an 8-bit SDRAM data port.
pub struct Data8;

/// Marker for a 16-bit SDRAM data port.
pub struct Data16;

/// Marker for a 32-bit SDRAM data port.
pub struct Data32;

/// Marker for an SDRAM with two banks.
pub struct Banks2;

/// Marker for an SDRAM with four banks.
pub struct Banks4;

/// Type-level SDRAM address width.
pub trait SdramAddressWidth {}

impl SdramAddressWidth for Address11 {}
impl SdramAddressWidth for Address12 {}
impl SdramAddressWidth for Address13 {}

/// Type-level SDRAM data port width.
pub trait SdramDataWidth {
    const PORT_SIZE: SdramPortSize;
}

impl SdramDataWidth for Data8 {
    const PORT_SIZE: SdramPortSize = SdramPortSize::_8BIT;
}

impl SdramDataWidth for Data16 {
    const PORT_SIZE: SdramPortSize = SdramPortSize::_16BIT;
}

impl SdramDataWidth for Data32 {
    const PORT_SIZE: SdramPortSize = SdramPortSize::_32BIT;
}

/// Type-level SDRAM bank count.
pub trait SdramBankCount {
    const BANK_NUM: Bank2Sel;
}

impl SdramBankCount for Banks2 {
    const BANK_NUM: Bank2Sel = Bank2Sel::BANK_NUM_2;
}

impl SdramBankCount for Banks4 {
    const BANK_NUM: Bank2Sel = Bank2Sel::BANK_NUM_4;
}
