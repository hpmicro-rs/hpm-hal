#![no_main]
#![no_std]

use core::arch::asm;

use defmt::info;
use defmt_rtt as _;
use hpm_hal as hal;
use panic_halt as _;

#[unsafe(no_mangle)]
pub static mut CPU_FEATURES_READY: u32 = 0;

#[unsafe(no_mangle)]
pub static mut CPU_FEATURES_WORDS: [u32; 4] = [0; 4];

#[hal::entry]
fn main() -> ! {
    let _p = hal::init(Default::default());

    let misa = read_misa();
    let mmsc_cfg = read_mmsc_cfg();
    let mmsc_cfg2 = read_mmsc_cfg2();
    let mrvarch_cfg = read_mrvarch_cfg();

    store_results(misa, mmsc_cfg, mmsc_cfg2, mrvarch_cfg);

    info!("hpm5e cpu feature csr probe");
    info!("compiled target: riscv32imafc-unknown-none-elf");
    info!("compiled extra target-feature: +zba,+zbb,+zbc,+zbs,+d,+zfh");

    info!("misa raw: 0x{:08x}", misa);
    info!("misa mxl: {}", misa >> 30);
    info!(
        "misa std: A={} C={} D={} F={} I={} M={} S={} U={} X={}",
        bit(misa, 0),
        bit(misa, 2),
        bit(misa, 3),
        bit(misa, 5),
        bit(misa, 8),
        bit(misa, 12),
        bit(misa, 18),
        bit(misa, 20),
        bit(misa, 23)
    );

    info!("mmsc_cfg raw: 0x{:08x}", mmsc_cfg);
    info!(
        "mmsc_cfg: CodeDense={} ACE={} CCTLCSR={} EFHW={} CLIC={} ECLIC={} EDSP={} MSC_EXT={}",
        bit(mmsc_cfg, 3),
        bit(mmsc_cfg, 6),
        bit(mmsc_cfg, 16),
        bit(mmsc_cfg, 17),
        bit(mmsc_cfg, 27),
        bit(mmsc_cfg, 28),
        bit(mmsc_cfg, 29),
        bit(mmsc_cfg, 31)
    );

    info!("mmsc_cfg2 raw: 0x{:08x}", mmsc_cfg2);
    info!(
        "mmsc_cfg2: BF16CVT={} ZFH={} RVARCH={}",
        bit(mmsc_cfg2, 0),
        bit(mmsc_cfg2, 1),
        bit(mmsc_cfg2, 20)
    );

    info!("mrvarch_cfg raw: 0x{:08x}", mrvarch_cfg);
    info!(
        "mrvarch_cfg bitmanip: ZBA={} ZBB={} ZBC={} ZBS={}",
        bit(mrvarch_cfg, 0),
        bit(mrvarch_cfg, 1),
        bit(mrvarch_cfg, 2),
        bit(mrvarch_cfg, 3)
    );
    info!(
        "mrvarch_cfg cache/security: ZICBOM={} ZICBOP={} ZICBOZ={} ZBK={} ZKN={} ZKS={} ZKT={} ZKR={} ZIHINTPAUSE={}",
        bit(mrvarch_cfg, 9),
        bit(mrvarch_cfg, 10),
        bit(mrvarch_cfg, 11),
        bit(mrvarch_cfg, 12),
        bit(mrvarch_cfg, 13),
        bit(mrvarch_cfg, 14),
        bit(mrvarch_cfg, 15),
        bit(mrvarch_cfg, 16),
        bit(mrvarch_cfg, 25)
    );

    loop {
        unsafe {
            asm!("wfi", options(nomem, nostack));
        }
    }
}

#[inline(always)]
fn bit(value: u32, n: u32) -> bool {
    value & (1 << n) != 0
}

fn store_results(misa: u32, mmsc_cfg: u32, mmsc_cfg2: u32, mrvarch_cfg: u32) {
    unsafe {
        let words = core::ptr::addr_of_mut!(CPU_FEATURES_WORDS).cast::<u32>();
        words.add(0).write_volatile(misa);
        words.add(1).write_volatile(mmsc_cfg);
        words.add(2).write_volatile(mmsc_cfg2);
        words.add(3).write_volatile(mrvarch_cfg);
        core::ptr::addr_of_mut!(CPU_FEATURES_READY).write_volatile(0x5e31_600d);
    }
}

#[inline(always)]
fn read_misa() -> u32 {
    let value: usize;
    unsafe {
        asm!("csrrs {0}, misa, x0", out(reg) value, options(nomem, nostack, preserves_flags));
    }
    value as u32
}

#[inline(always)]
fn read_mmsc_cfg() -> u32 {
    let value: usize;
    unsafe {
        asm!("csrrs {0}, 0xfc2, x0", out(reg) value, options(nomem, nostack, preserves_flags));
    }
    value as u32
}

#[inline(always)]
fn read_mmsc_cfg2() -> u32 {
    let value: usize;
    unsafe {
        asm!("csrrs {0}, 0xfc3, x0", out(reg) value, options(nomem, nostack, preserves_flags));
    }
    value as u32
}

#[inline(always)]
fn read_mrvarch_cfg() -> u32 {
    let value: usize;
    unsafe {
        asm!("csrrs {0}, 0xfca, x0", out(reg) value, options(nomem, nostack, preserves_flags));
    }
    value as u32
}
