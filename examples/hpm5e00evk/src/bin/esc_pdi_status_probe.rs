#![no_main]
#![no_std]

use core::sync::atomic::{Ordering, compiler_fence};

use defmt_rtt as _;
use hpm_hal as hal;
use panic_halt as _;

const RESULT_MAGIC: u32 = 0x4553_4353;
const RESULT_PASS: u32 = 0x5041_5353;
const ESC_BASE: usize = 0xF170_0000;

#[unsafe(no_mangle)]
#[unsafe(link_section = ".uninit.esc_pdi_status")]
static mut ESC_PDI_STATUS: [u32; 16] = [0; 16];

#[hal::entry]
fn main() -> ! {
    clear_result();
    record(0, RESULT_MAGIC);
    record(1, 0x01);

    let _peripherals = hal::init(Default::default());
    hal::sysctl::clock_add_to_group(hal::pac::resources::ESC0, 0);
    riscv::asm::fence();

    let esc = hal::pac::ESC;
    esc.gpr_cfg0().modify(|w| w.set_clk100_en(true));
    riscv::asm::fence();
    spin(10_000);
    record(1, 0x20);

    let pdi_ctrl = read8(0x0140);
    let pdi_cfg = read8(0x0150);
    let al_status = read16(0x0130);
    let eeprom_config = read8(0x0500);
    let eeprom_status = read16(0x0502);

    record(
        2,
        u32::from(pdi_ctrl) | (u32::from(pdi_cfg) << 8) | (u32::from(al_status) << 16),
    );
    record(3, u32::from(eeprom_config) | (u32::from(eeprom_status) << 16));
    record(15, u32::from(read16(0x014E)));

    let primary = (ESC_BASE + 0x1000) as *mut u32;
    let alias = (ESC_BASE + 0x1_0000) as *mut u32;
    record(4, read32_ptr(primary));
    record(5, read32_ptr(alias));

    write32_ptr(primary, 0x1122_3344);
    record(6, read32_ptr(primary));
    record(7, read32_ptr(alias));

    write32_ptr(alias, 0x5566_7788);
    record(8, read32_ptr(primary));
    record(9, read32_ptr(alias));

    write32_ptr(alias, 0);
    write32_ptr(primary, 0);
    record(10, read32_ptr(primary));
    record(11, read32_ptr(alias));
    record(12, u32::from(esc.feature().read().0));
    record(13, esc.gpr_cfg0().read().0);
    record(14, hal::sysctl::clocks().cpu0.0);
    record(1, RESULT_PASS);

    defmt::info!(
        "HPM5E ESC PDI status PASS: PDI_CTRL={=u8:#04x} PDI_CFG={=u8:#04x} AL={=u16:#06x} EEPROM={=u16:#06x}",
        pdi_ctrl,
        pdi_cfg,
        al_status,
        eeprom_status
    );

    loop {
        riscv::asm::nop();
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

fn read32_ptr(address: *mut u32) -> u32 {
    compiler_fence(Ordering::SeqCst);
    let value = unsafe { address.read_volatile() };
    compiler_fence(Ordering::SeqCst);
    value
}

fn write32_ptr(address: *mut u32, value: u32) {
    compiler_fence(Ordering::SeqCst);
    unsafe { address.write_volatile(value) };
    riscv::asm::fence();
    compiler_fence(Ordering::SeqCst);
}

fn clear_result() {
    for index in 0..16 {
        record(index, 0);
    }
}

#[inline(never)]
fn record(index: usize, value: u32) {
    unsafe {
        core::ptr::write_volatile(core::ptr::addr_of_mut!(ESC_PDI_STATUS).cast::<u32>().add(index), value);
    }
}

#[inline(never)]
fn spin(cycles: u32) {
    for _ in 0..cycles {
        riscv::asm::nop();
    }
}
