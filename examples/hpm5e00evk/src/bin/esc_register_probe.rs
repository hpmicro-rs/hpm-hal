#![no_main]
#![no_std]

use defmt_rtt as _;
use hpm_hal as hal;
use panic_halt as _;

const RESULT_MAGIC: u32 = 0x4553_4350;

#[unsafe(no_mangle)]
#[unsafe(link_section = ".uninit.esc_probe_result")]
static mut ESC_PROBE_RESULT: [u32; 8] = [0; 8];

#[hal::entry]
fn main() -> ! {
    record(0, RESULT_MAGIC);
    record(1, 0x01);

    let _peripherals = hal::init(Default::default());
    record(1, 0x10);

    // Match HPM SDK hpm5e00evk init_esc0_clock(): make the ESC0 resource
    // accessible before touching the ESC integration register window.
    hal::sysctl::clock_add_to_group(hal::pac::resources::ESC0, 0);
    riscv::asm::fence();
    record(1, 0x20);

    let esc = hal::pac::ESC;
    let gpr_cfg0_before = esc.gpr_cfg0().read().0;
    record(2, gpr_cfg0_before);

    // Match HPM SDK esc_core_enable_clock(HPM_ESC, true).
    esc.gpr_cfg0().modify(|w| w.set_clk100_en(true));
    riscv::asm::fence();
    spin(10_000);

    let gpr_cfg0_after = esc.gpr_cfg0().read().0;
    record(3, gpr_cfg0_after);
    record(1, 0x30);

    let identity0 = u32::from(esc.type_().read().0)
        | (u32::from(esc.revision().read().0) << 8)
        | (u32::from(esc.build().read().0) << 16);
    let identity1 = u32::from(esc.fmmu_num().read().0)
        | (u32::from(esc.syncm_num().read().0) << 8)
        | (u32::from(esc.ram_size().read().0) << 16)
        | (u32::from(esc.port_desc().read().0) << 24);
    let feature = u32::from(esc.feature().read().0);

    record(4, identity0);
    record(5, identity1);
    record(6, feature);
    record(7, hal::sysctl::clocks().cpu0.0);
    record(1, 0x5e31);

    defmt::info!("HPM5E ESC register probe PASS");
    defmt::info!(
        "GPR_CFG0 before={=u32:#010x} after={=u32:#010x}",
        gpr_cfg0_before,
        gpr_cfg0_after
    );
    defmt::info!(
        "TYPE={=u8:#04x} REVISION={=u8:#04x} BUILD={=u16:#06x}",
        identity0 as u8,
        (identity0 >> 8) as u8,
        (identity0 >> 16) as u16
    );
    defmt::info!(
        "FMMU={} SYNCM={} RAM_SIZE={=u8:#04x} PORT_DESC={=u8:#04x} FEATURE={=u16:#06x}",
        identity1 as u8,
        (identity1 >> 8) as u8,
        (identity1 >> 16) as u8,
        (identity1 >> 24) as u8,
        feature as u16
    );

    loop {
        riscv::asm::nop();
    }
}

#[inline(never)]
fn record(index: usize, value: u32) {
    unsafe {
        core::ptr::write_volatile(
            core::ptr::addr_of_mut!(ESC_PROBE_RESULT).cast::<u32>().add(index),
            value,
        );
    }
}

#[inline(never)]
fn spin(cycles: u32) {
    for _ in 0..cycles {
        riscv::asm::nop();
    }
}
