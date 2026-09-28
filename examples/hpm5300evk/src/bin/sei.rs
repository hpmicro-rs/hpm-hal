#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]
#![feature(impl_trait_in_assoc_type)]
#![feature(abi_riscv_interrupt)]

use embassy_time::Timer;
use hal::pac;
use hpm_hal::gpio::{Level, NoPin, Output};
use hpm_hal::interrupt::InterruptExt as _;
use hpm_hal::pac::qei::vals;
use {defmt_rtt as _, hpm_hal as hal};

#[allow(non_snake_case)]
#[no_mangle]
unsafe extern "riscv-interrupt-m" fn QEI1() {
    let r = hal::pac::QEI1;

    let ph0 = r.phase_cnt().read().0;
    let z_cnt = r.count_current().z().read().0;

    let freq_in = hal::sysctl::clocks().ahb.0;
    let cycle0_snap0 = r.cycle0_snap0().read().cycle0_snap0();
    let cycle0_snap1 = r.cycle0_snap1().read().cycle0_snap1();

    if cycle0_snap0 != 0 && cycle0_snap1 != 0 {
        let speed0 = freq_in / cycle0_snap0;
        let speed1 = freq_in / cycle0_snap1;

        defmt::info!(
            "z: {} ph: {} speed0: {} r/s  speed1: {} r/s",
            z_cnt,
            ph0,
            speed0,
            speed1
        );
    }

    r.sr().modify(|w| w.set_pulse0f(true)); // clear interrupt flag. W1C

    hal::interrupt::QEI1.complete();
}

#[embassy_executor::main(entry = "hpm_hal::entry")]
async fn main(_spawner: embassy_executor::Spawner) -> ! {
    let p = hal::init(Default::default());

    defmt::info!("Clock summary:");
    defmt::info!("  CPU0:\t{}Hz", hal::sysctl::clocks().cpu0.0);
    defmt::info!("  AHB:\t{}Hz", hal::sysctl::clocks().ahb.0);

    // SEI0
    // CK: PB05
    // DE: PB04
    // RX: PB15
    // TX: __

    hal::sysctl::clock_add_to_group(pac::resources::MOT0, 0);

    pac::IOC
        .pad(pac::pins::PB05)
        .func_ctl()
        .write(|w| w.set_alt_select(pac::iomux::IOC_PB05_FUNC_CTL_SEI0_CK));
    pac::IOC
        .pad(pac::pins::PB04)
        .func_ctl()
        .write(|w| w.set_alt_select(pac::iomux::IOC_PB04_FUNC_CTL_SEI0_DE));
    pac::IOC
        .pad(pac::pins::PB15)
        .func_ctl()
        .write(|w| w.set_alt_select(pac::iomux::IOC_PB15_FUNC_CTL_SEI0_RX));

    use pac::SEI;

    SEI.ctrl(0).engine_ctrl().modify(|w| w.set_enable(false)); // disable engine

    SEI.ctrl(0).xcvr_baud_cfg().modify(|w| w.set_baud_div(val));

    let mut led = Output::new(p.PA23, Level::High, Default::default());

    loop {
        led.toggle();

        Timer::after_millis(500).await;
    }
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    let mut err = heapless::String::<1024>::new();

    use core::fmt::Write as _;

    write!(err, "panic: {}", info).ok();

    defmt::info!("{}", err.as_str());

    loop {}
}
