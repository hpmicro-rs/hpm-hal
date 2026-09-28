#![no_main]
#![no_std]

use core::sync::atomic::{Ordering, compiler_fence};

use defmt_rtt as _;
use hpm_hal as hal;
use panic_halt as _;

const RESULT_MAGIC: u32 = 0x4553_4352;
const RESULT_PASS: u32 = 0x5041_5353;
const ESC_BASE: usize = 0xF170_0000;
const PDRAM_OFFSET: usize = 0x1000;
const PDRAM_ALIAS_OFFSET: usize = 0x1_0000;
const MAX_PDRAM_BYTES: usize = 60 * 1024;

// Retained result layout:
//  0 magic, 1 stage/final status, 2 passes, 3 words/pass,
//  4 total verified words, 5 failure ESC offset, 6 expected, 7 actual,
//  8 initial alias digest, 9 final alias digest, 10 reserved,
// 11 RAM_SIZE/identity, 12 FEATURE, 13 GPR_CFG0, 14 CPU Hz, 15 reserved.
#[unsafe(no_mangle)]
#[unsafe(link_section = ".uninit.esc_pdram_result")]
static mut ESC_PDRAM_RESULT: [u32; 16] = [0; 16];

#[derive(Clone, Copy)]
enum Pattern {
    Constant(u32),
    Address,
    InverseAddress,
    WalkingOne(u32),
    WalkingZero(u32),
}

#[derive(Clone, Copy)]
struct Failure {
    offset: usize,
    expected: u32,
    actual: u32,
}

#[hal::entry]
fn main() -> ! {
    clear_result();
    record(0, RESULT_MAGIC);
    record(1, 0x01);

    let _peripherals = hal::init(Default::default());
    record(1, 0x10);

    // Follow the HPM SDK ordering: expose ESC0 through SYSCTL before touching
    // the integration window, then enable the ESC core's 100 MHz clock.
    hal::sysctl::clock_add_to_group(hal::pac::resources::ESC0, 0);
    riscv::asm::fence();
    record(1, 0x20);

    let esc = hal::pac::ESC;
    esc.gpr_cfg0().modify(|w| w.set_clk100_en(true));
    riscv::asm::fence();
    spin(10_000);

    let ram_kib = usize::from(esc.ram_size().read().0);
    let pdram_bytes = core::cmp::min(ram_kib * 1024, MAX_PDRAM_BYTES);
    let words = pdram_bytes / size_of::<u32>();
    let identity = u32::from(esc.ram_size().read().0)
        | (u32::from(esc.fmmu_num().read().0) << 8)
        | (u32::from(esc.syncm_num().read().0) << 16)
        | (u32::from(esc.port_desc().read().0) << 24);

    record(3, words as u32);
    record(11, identity);
    record(12, u32::from(esc.feature().read().0));
    record(13, esc.gpr_cfg0().read().0);
    record(14, hal::sysctl::clocks().cpu0.0);

    if words == 0 || pdram_bytes & 3 != 0 {
        fail(Failure {
            offset: PDRAM_OFFSET,
            expected: MAX_PDRAM_BYTES as u32,
            actual: pdram_bytes as u32,
        });
    }

    // In INIT with no active SyncManager the standard 0x1000 PDI window is
    // ownership-controlled. HPM's 0x10000 PDRAM alias exposes the raw 60 KiB
    // process RAM for initialization and qualification.
    let pdram_alias = (ESC_BASE + PDRAM_ALIAS_OFFSET) as *mut u32;
    record(8, digest(pdram_alias, words));

    let fixed = [
        Pattern::Constant(0x0000_0000),
        Pattern::Constant(0xFFFF_FFFF),
        Pattern::Constant(0xAAAA_AAAA),
        Pattern::Constant(0x5555_5555),
        Pattern::Constant(0xA5A5_A5A5),
        Pattern::Constant(0x5A5A_5A5A),
        Pattern::Address,
        Pattern::InverseAddress,
    ];

    for pattern in fixed {
        run_alias_pass(pdram_alias, words, pattern, 0x100);
    }

    for bit in 0..32 {
        run_alias_pass(pdram_alias, words, Pattern::WalkingOne(bit), 0x200 + bit);
    }
    for bit in 0..32 {
        run_alias_pass(pdram_alias, words, Pattern::WalkingZero(bit), 0x300 + bit);
    }

    // Leave the process RAM in a deterministic INIT-state baseline.
    record(1, 0x500);
    write_pattern(pdram_alias, words, Pattern::Constant(0));
    verify_or_fail(pdram_alias, words, Pattern::Constant(0), PDRAM_ALIAS_OFFSET);
    complete_pass(words);

    record(9, digest(pdram_alias, words));
    record(1, RESULT_PASS);

    defmt::info!(
        "HPM5E ESC PDRAM PASS: bytes={} passes={} verified_words={}",
        pdram_bytes,
        result(2),
        result(4)
    );

    loop {
        riscv::asm::nop();
    }
}

fn run_alias_pass(pdram_alias: *mut u32, words: usize, pattern: Pattern, stage: u32) {
    record(1, stage);
    write_pattern(pdram_alias, words, pattern);
    verify_or_fail(pdram_alias, words, pattern, PDRAM_ALIAS_OFFSET);
    complete_pass(words);
}

fn write_pattern(base: *mut u32, words: usize, pattern: Pattern) {
    compiler_fence(Ordering::SeqCst);
    for index in 0..words {
        unsafe { base.add(index).write_volatile(pattern_value(pattern, index)) };
    }
    riscv::asm::fence();
    compiler_fence(Ordering::SeqCst);
}

fn verify_or_fail(base: *mut u32, words: usize, pattern: Pattern, esc_offset: usize) {
    compiler_fence(Ordering::SeqCst);
    for index in 0..words {
        let expected = pattern_value(pattern, index);
        let actual = unsafe { base.add(index).read_volatile() };
        if actual != expected {
            fail(Failure {
                offset: esc_offset + index * size_of::<u32>(),
                expected,
                actual,
            });
        }
    }
    compiler_fence(Ordering::SeqCst);
}

fn pattern_value(pattern: Pattern, index: usize) -> u32 {
    let address = (PDRAM_OFFSET + index * size_of::<u32>()) as u32;
    match pattern {
        Pattern::Constant(value) => value,
        Pattern::Address => address ^ 0x3C5A_A5C3,
        Pattern::InverseAddress => !(address ^ 0x3C5A_A5C3),
        Pattern::WalkingOne(bit) => (1u32 << bit) ^ address.rotate_left(bit),
        Pattern::WalkingZero(bit) => !((1u32 << bit) ^ address.rotate_left(bit)),
    }
}

fn complete_pass(verified_words: usize) {
    record(2, result(2).wrapping_add(1));
    record(4, result(4).wrapping_add(verified_words as u32));
}

fn digest(base: *mut u32, words: usize) -> u32 {
    let mut value = 0x811C_9DC5u32;
    for index in 0..words {
        let word = unsafe { base.add(index).read_volatile() };
        value = value.rotate_left(5) ^ word ^ index as u32;
    }
    value
}

fn fail(failure: Failure) -> ! {
    record(5, failure.offset as u32);
    record(6, failure.expected);
    record(7, failure.actual);
    record(1, 0xFA11_0000 | (result(1) & 0xFFFF));
    defmt::error!(
        "HPM5E ESC PDRAM FAIL: offset={=u32:#07x} expected={=u32:#010x} actual={=u32:#010x}",
        failure.offset as u32,
        failure.expected,
        failure.actual
    );
    loop {
        riscv::asm::nop();
    }
}

fn clear_result() {
    for index in 0..16 {
        record(index, 0);
    }
}

#[inline(never)]
fn record(index: usize, value: u32) {
    unsafe {
        core::ptr::write_volatile(
            core::ptr::addr_of_mut!(ESC_PDRAM_RESULT).cast::<u32>().add(index),
            value,
        );
    }
}

#[inline(never)]
fn result(index: usize) -> u32 {
    unsafe { core::ptr::read_volatile(core::ptr::addr_of!(ESC_PDRAM_RESULT).cast::<u32>().add(index)) }
}

#[inline(never)]
fn spin(cycles: u32) {
    for _ in 0..cycles {
        riscv::asm::nop();
    }
}
