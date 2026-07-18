#![no_main]
#![no_std]

use defmt_rtt as _;
use hpm_hal as hal;
use panic_halt as _;

#[unsafe(no_mangle)]
#[unsafe(link_section = ".uninit.rtt_defmt_probe_stage")]
static mut RTT_DEFMT_PROBE_STAGE: u32 = 0;

#[hal::entry]
fn main() -> ! {
    stage(0x10);
    let config = Default::default();
    stage(0x18);
    let _p = hal::init(config);
    stage(0x20);

    defmt::info!("rtt defmt probe start");
    stage(0x30);
    defmt::info!("CPU0 clock: {}Hz", hal::sysctl::clocks().cpu0.0);
    stage(0x40);

    for index in 0..16u32 {
        stage(0x100 + index);
        defmt::info!("rtt defmt probe sample {}", index);
        spin(1_000_000);
    }

    defmt::info!("rtt defmt probe done");
    stage(0x200);

    loop {
        riscv::asm::nop();
    }
}

#[inline(never)]
fn stage(value: u32) {
    unsafe {
        let base = core::ptr::addr_of_mut!(RTT_DEFMT_PROBE_STAGE) as *mut u32;
        core::arch::asm!(
            "sw {value}, 0({base})",
            base = in(reg) base,
            value = in(reg) value,
            options(nostack),
        );
    }
}

#[inline(never)]
fn spin(cycles: u32) {
    for _ in 0..cycles {
        riscv::asm::nop();
    }
}
