#![no_main]
#![no_std]

use andes_riscv::register;
use assign_resources::assign_resources;
use defmt::info;
use embedded_hal::delay::DelayNs;
use hal::peripherals;
use riscv::delay::McycleDelay;
use {defmt_rtt as _, hpm_hal as hal};

assign_resources! {
    // FT2232 UART
    uart0: Uart0Resources {
        tx: PY06,
        rx: PY07,
        uart: UART0,
    },
    rgb_led: RgbLedResources {
        r: PA27,
        g: PB01,
        b: PB19,
    },
    buttons: ButtonResources {
        boot0: PA20,
        boot1: PA21,
        sw3_pbut_n: PZ02,
        sw2_rst_n: PZ01,
    }
    // DO NOT USE
    jtag: JtagResources {
        tdo: PY00,
        tdi: PY01,
        tck: PY02,
        tms: PY03,
        trst: PY04,
    },
    // DO NOT USE
    xpi0: Xpi0Resources {
        CS: PA00,
        D1: PA01,
        D2: PA02,
        D0: PA03,
        SCLK: PA04,
        D3: PA05,
    }
}

#[hal::entry]
fn main() -> ! {
    let _p = hal::init(Default::default());

    let mut delay = McycleDelay::new(hal::sysctl::clocks().cpu0.0);

    info!("CPU0 clock: {}Hz", hal::sysctl::clocks().cpu0.0);
    info!("CPU Info");

    let misa = riscv::register::misa::read().unwrap();

    for c in 'A'..='Z' {
        if misa.has_extension(c) {
            info!("  Extension: {}", c);
        }
    }
    //    misa.has_extension(extension)

    let r = register::mmsc_cfg().read();
    info!("mmsc_cfg: {:08x}", r.0);
    info!("  ECC: {}", r.ecc());
    info!("  CodeDense: {}", r.ecd());
    info!("  PowerBrake: {}", r.pft());

    info!("  HW Stack protection: {}", r.hsp());
    // andes custom extension
    info!("  ACE: {}", r.ace());
    // vectored plic
    info!("  VPLIC: {}", r.vplic());
    // Andes V5 performance extension
    info!("  EV5PE: {}", r.ev5pe());
    info!("  PMNDS: {}", r.pmnds());
    info!("  CCTLCSR: {}", r.cctlcsr());
    info!("  EFHW: {}", r.efhw());
    info!("  VCCTL: {}", r.vcctl());
    info!("  EXCSLVL: {}", r.excslvl());
    info!("  NOPMC: {}", r.nopmc());
    info!("  SPE_AFT: {}", r.spe_aft());
    info!("  ESLEEP: {}", r.esleep());
    info!("  PPI: {}", r.ppi());
    info!("  FIO: {}", r.fio());

    info!("  CLIC: {}", r.clic());
    info!("  ECLIC: {}", r.eclic());

    info!("  EDSP: {}", r.edsp());

    info!("  PPMA: {}", r.ppma());

    info!("  MSC_EXT: {}", r.msc_ext());

    let r = register::mmsc_cfg2().read();
    info!("mmsc_cfg2: {:08x}", r.0);
    info!("  BF16CVT: {}", r.bf16cvt());
    info!("  ZFH: {}", r.zfh());
    info!("  FINV: {}", r.finv());

    if r.rvarch() {
        info!("  RVARCH: {}", r.rvarch());

        let r = register::mrvarch_cfg().read();
        info!("mrvarch_cfg: {:08x}", r.0);
    }

    loop {
        defmt::info!("tick");

        delay.delay_ms(2000);
    }
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    defmt::info!("panic");
    loop {}
}
