#![no_main]
#![no_std]

use hpm_hal as hal;
use panic_halt as _;

#[unsafe(no_mangle)]
#[unsafe(link_section = ".uninit.main_marker_stage")]
static mut MAIN_MARKER_STAGE: u32 = 0;

#[hal::entry]
fn main() -> ! {
    stage(0x5100);

    loop {
        riscv::asm::nop();
    }
}

#[inline(never)]
fn stage(value: u32) {
    unsafe {
        let base = core::ptr::addr_of_mut!(MAIN_MARKER_STAGE) as *mut u32;
        core::arch::asm!(
            "sw {value}, 0({base})",
            base = in(reg) base,
            value = in(reg) value,
            options(nostack),
        );
    }
}
