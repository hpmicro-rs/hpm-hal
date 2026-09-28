#![no_main]
#![no_std]

use core::sync::atomic::{Ordering, compiler_fence};

use defmt_rtt as _;
use hpm_hal as hal;
use panic_halt as _;

const RESULT_MAGIC: u32 = 0x4553_4949;
const RESULT_PASS: u32 = 0x5041_5353;
const ESC_BASE: usize = 0xF170_0000;
const EEPROM_CTRL_STAT: usize = 0x0502;
const EEPROM_ADDR: usize = 0x0504;
const EEPROM_DATA: usize = 0x0508;
const EEPROM_BUSY: u16 = 1 << 15;
const EEPROM_ACK_ERROR: u16 = 1 << 13;
const EEPROM_CHECKSUM_ERROR: u16 = 1 << 11;
const EEPROM_CMD_SHIFT: u16 = 8;
const EEPROM_CMD_MASK: u16 = 0x7;
const EEPROM_CMD_READ: u16 = 1;
const EEPROM_CMD_WRITE: u16 = 2;
const EEPROM_CMD_RELOAD: u16 = 4;

const SII_LEN: usize = 2048;
const SII_SOURCE: &str = include_str!("../../../../../hpm_sdk/samples/ethercat/ecat_io/SSC/Src/eeprom.h");

#[unsafe(no_mangle)]
#[unsafe(link_section = ".uninit.esc_sii_result")]
static mut ESC_SII_RESULT: [u32; 24] = [0; 24];

static mut SII_RAM: [u8; SII_LEN] = [0; SII_LEN];

#[hal::entry]
fn main() -> ! {
    clear_result();
    let loaded = load_sii_template();
    record(0, RESULT_MAGIC);
    record(1, 0x01);
    record(23, loaded as u32);
    if loaded != SII_LEN {
        fail(0xC000, SII_LEN as u32, loaded as u32);
    }

    let _peripherals = hal::init(Default::default());
    hal::sysctl::clock_add_to_group(hal::pac::resources::ESC0, 0);
    riscv::asm::fence();

    let esc = hal::pac::ESC;
    record(4, esc.gpr_cfg0().read().0);
    esc.gpr_cfg0().modify(|w| w.set_clk100_en(true));
    riscv::asm::fence();
    spin(10_000);
    record(5, esc.gpr_cfg0().read().0);

    let checksum = config_checksum();
    record(10, u32::from(checksum) | (u32::from(sii_byte(14)) << 8));
    record(11, sii_u32(16));
    record(12, sii_u32(20));
    record(13, sii_u32(24));
    record(14, sii_u32(28));
    record(15, sii_digest());
    if checksum != sii_byte(14) {
        fail(0xC001, u32::from(sii_byte(14)), u32::from(checksum));
    }

    let before = read16(EEPROM_CTRL_STAT);
    record(2, u32::from(before));
    if before & EEPROM_BUSY == 0 || command(before) != EEPROM_CMD_RELOAD {
        fail(0xC002, 0x9400, u32::from(before));
    }

    if service_eeprom_command().is_err() {
        fail(0xC003, EEPROM_CMD_RELOAD.into(), 0);
    }
    record(16, 1);

    let mut after = read16(EEPROM_CTRL_STAT);
    for _ in 0..100_000 {
        if after & EEPROM_BUSY == 0 {
            break;
        }
        after = read16(EEPROM_CTRL_STAT);
    }
    record(3, u32::from(after));

    let station_alias = read16(0x0012);
    record(6, reload_data());
    record(7, 0x3412 | (u32::from(station_alias) << 16));
    record(
        8,
        u32::from(read8(0x0140)) | (u32::from(read8(0x0150)) << 8) | (u32::from(read16(0x0130)) << 16),
    );
    record(9, u32::from(read16(0x0110)));
    record(
        17,
        u32::from(esc.ram_size().read().0)
            | (u32::from(esc.fmmu_num().read().0) << 8)
            | (u32::from(esc.syncm_num().read().0) << 16),
    );
    record(18, u32::from(esc.feature().read().0));
    record(19, hal::sysctl::clocks().cpu0.0);

    if after & (EEPROM_BUSY | EEPROM_ACK_ERROR | EEPROM_CHECKSUM_ERROR) != 0 {
        fail(0xC004, 0, u32::from(after));
    }
    if station_alias != 0x3412 {
        fail(0xC005, 0x3412, u32::from(station_alias));
    }
    if read16(0x0110) & 1 == 0 {
        fail(0xC006, 1, u32::from(read16(0x0110)));
    }

    record(1, RESULT_PASS);
    defmt::info!(
        "HPM5E ESC SII reload PASS: status={=u16:#06x} alias={=u16:#06x} vendor={=u32:#010x}",
        after,
        station_alias,
        sii_u32(16)
    );

    loop {
        if read16(EEPROM_CTRL_STAT) & EEPROM_BUSY != 0 {
            if service_eeprom_command().is_ok() {
                record(16, result(16).wrapping_add(1));
            }
        }
    }
}

fn service_eeprom_command() -> Result<(), ()> {
    let status = read16(EEPROM_CTRL_STAT);
    let cmd = command(status);
    match cmd {
        EEPROM_CMD_READ => {
            let byte_address = read32(EEPROM_ADDR) as usize * 2;
            if byte_address + 8 > SII_LEN {
                acknowledge(cmd, true, false);
                return Err(());
            }
            write32(EEPROM_DATA, sii_u32(byte_address));
            write32(EEPROM_DATA + 4, sii_u32(byte_address + 4));
            acknowledge(cmd, false, false);
        }
        EEPROM_CMD_WRITE => {
            let byte_address = read32(EEPROM_ADDR) as usize * 2;
            if byte_address + 2 > SII_LEN {
                acknowledge(cmd, true, false);
                return Err(());
            }
            let data = read16(EEPROM_DATA).to_le_bytes();
            sii_set_byte(byte_address, data[0]);
            sii_set_byte(byte_address + 1, data[1]);
            acknowledge(cmd, false, false);
        }
        EEPROM_CMD_RELOAD => {
            let checksum = config_checksum();
            if checksum != sii_byte(14) {
                acknowledge(cmd, false, true);
                return Err(());
            }
            write32(EEPROM_DATA, reload_data());
            write32(EEPROM_DATA + 4, 0);
            acknowledge(cmd, false, false);
        }
        _ => {
            acknowledge(cmd, true, false);
            return Err(());
        }
    }
    Ok(())
}

fn reload_data() -> u32 {
    let link = sii_byte(1);
    u32::from(sii_u16(8))
        | (u32::from((link >> 1) & 1) << 16)
        | (u32::from((link >> 4) & 1) << 17)
        | (u32::from((link >> 5) & 1) << 18)
        | (u32::from((link >> 6) & 1) << 19)
        | (u32::from((link >> 7) & 1) << 20)
}

fn acknowledge(cmd: u16, ack_error: bool, checksum_error: bool) {
    let mut value = cmd << EEPROM_CMD_SHIFT;
    if ack_error {
        value |= EEPROM_ACK_ERROR;
    }
    if checksum_error {
        value |= EEPROM_CHECKSUM_ERROR;
    }
    write16(EEPROM_CTRL_STAT, value);
}

fn command(status: u16) -> u16 {
    (status >> EEPROM_CMD_SHIFT) & EEPROM_CMD_MASK
}

fn config_checksum() -> u8 {
    let mut remainder = 0xFFu8;
    for index in 0..14 {
        remainder ^= sii_byte(index);
        for _ in 0..8 {
            remainder = if remainder & 0x80 != 0 {
                (remainder << 1) ^ 0x07
            } else {
                remainder << 1
            };
        }
    }
    remainder
}

fn sii_digest() -> u32 {
    let mut digest = 0x811C_9DC5u32;
    for index in 0..SII_LEN {
        digest = digest.rotate_left(5) ^ u32::from(sii_byte(index)) ^ index as u32;
    }
    digest
}

fn sii_u16(offset: usize) -> u16 {
    u16::from_le_bytes([sii_byte(offset), sii_byte(offset + 1)])
}

fn sii_u32(offset: usize) -> u32 {
    u32::from_le_bytes([
        sii_byte(offset),
        sii_byte(offset + 1),
        sii_byte(offset + 2),
        sii_byte(offset + 3),
    ])
}

fn load_sii_template() -> usize {
    let source = SII_SOURCE.as_bytes();
    let mut source_index = 0;
    let mut output_index = 0;
    while source_index + 3 < source.len() && output_index < SII_LEN {
        if source[source_index] == b'0' && source[source_index + 1] == b'x' {
            let high = hex_nibble(source[source_index + 2]);
            let low = hex_nibble(source[source_index + 3]);
            if let (Some(high), Some(low)) = (high, low) {
                sii_set_byte(output_index, (high << 4) | low);
                output_index += 1;
                source_index += 4;
                continue;
            }
        }
        source_index += 1;
    }
    output_index
}

fn hex_nibble(value: u8) -> Option<u8> {
    match value {
        b'0'..=b'9' => Some(value - b'0'),
        b'a'..=b'f' => Some(value - b'a' + 10),
        b'A'..=b'F' => Some(value - b'A' + 10),
        _ => None,
    }
}

fn sii_byte(index: usize) -> u8 {
    unsafe { core::ptr::addr_of!(SII_RAM).cast::<u8>().add(index).read_volatile() }
}

fn sii_set_byte(index: usize, value: u8) {
    unsafe {
        core::ptr::addr_of_mut!(SII_RAM)
            .cast::<u8>()
            .add(index)
            .write_volatile(value)
    }
}

fn read8(offset: usize) -> u8 {
    compiler_fence(Ordering::SeqCst);
    let value = unsafe { ((ESC_BASE + offset) as *const u8).read_volatile() };
    compiler_fence(Ordering::SeqCst);
    value
}

fn read16(offset: usize) -> u16 {
    compiler_fence(Ordering::SeqCst);
    let value = unsafe { ((ESC_BASE + offset) as *const u16).read_volatile() };
    compiler_fence(Ordering::SeqCst);
    value
}

fn read32(offset: usize) -> u32 {
    compiler_fence(Ordering::SeqCst);
    let value = unsafe { ((ESC_BASE + offset) as *const u32).read_volatile() };
    compiler_fence(Ordering::SeqCst);
    value
}

fn write16(offset: usize, value: u16) {
    compiler_fence(Ordering::SeqCst);
    unsafe { ((ESC_BASE + offset) as *mut u16).write_volatile(value) };
    riscv::asm::fence();
}

fn write32(offset: usize, value: u32) {
    compiler_fence(Ordering::SeqCst);
    unsafe { ((ESC_BASE + offset) as *mut u32).write_volatile(value) };
    riscv::asm::fence();
}

fn fail(code: u32, expected: u32, actual: u32) -> ! {
    record(20, code);
    record(21, expected);
    record(22, actual);
    record(1, 0xFA11_0000 | code);
    loop {
        riscv::asm::nop();
    }
}

fn clear_result() {
    for index in 0..24 {
        record(index, 0);
    }
}

fn record(index: usize, value: u32) {
    unsafe {
        core::ptr::addr_of_mut!(ESC_SII_RESULT)
            .cast::<u32>()
            .add(index)
            .write_volatile(value)
    }
}

fn result(index: usize) -> u32 {
    unsafe {
        core::ptr::addr_of!(ESC_SII_RESULT)
            .cast::<u32>()
            .add(index)
            .read_volatile()
    }
}

fn spin(cycles: u32) {
    for _ in 0..cycles {
        riscv::asm::nop();
    }
}
