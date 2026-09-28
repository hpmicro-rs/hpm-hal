#![no_main]
#![no_std]

use andes_riscv::register;
use defmt::info;
use embedded_hal::delay::DelayNs;
use hal::gpio::{Level, Output, Speed};
use riscv::delay::McycleDelay;
use {defmt_rtt as _, hpm_hal as hal};

#[hal::entry]
fn main() -> ! {
    let p = hal::init(Default::default());

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
    info!("  RVARCH: {}", r.rvarch());

    let r = register::mrvarch_cfg().read();
    info!("mrvarch_cfg: {:08x}", r.0);

    // let mut led = Output::new(p.PA10, Level::Low, Speed::default());
    let mut led = Output::new(p.PA23, Level::Low, Speed::default());

    loop {
        defmt::info!("tick");

        led.toggle();
        delay.delay_ms(2000);
    }
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    defmt::info!("panic");
    loop {}
}
