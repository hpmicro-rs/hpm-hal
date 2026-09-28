#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]
#![feature(impl_trait_in_assoc_type)]

use embassy_executor::Spawner;
use embassy_time::Timer;
use hal::gpio::Pin as _;
use hal::pac;
use hpm_hal::gpio::{AnyPin, Level, Output};
use {defmt_rtt as _, hpm_hal as hal};

#[embassy_executor::task(pool_size = 3)]
async fn blink(pin: AnyPin, interval_ms: u32) {
    let mut led = Output::new(pin, Level::Low, Default::default());

    loop {
        led.toggle();

        Timer::after_millis(interval_ms as u64).await;
    }
}


fn init_esc_pins() {
    use pac::pins::*;
    use pac::IOC;
    use pac::iomux::*;

    // ESC
    IOC.pad(PA09).func_ctl().modify(|w| w.set_alt_select(IOC_PA09_FUNC_CTL_ESC0_REFCK));
    IOC.pad(PA30).func_ctl().modify(|w| w.set_alt_select(IOC_PA30_FUNC_CTL_ESC0_MDIO));
    IOC.pad(PA31).func_ctl().modify(|w| w.set_alt_select(IOC_PA31_FUNC_CTL_ESC0_MDC));
    IOC.pad(PA12).func_ctl().modify(|w| w.set_alt_select(IOC_PA12_FUNC_CTL_ESC0_SDA));
    IOC.pad(PA13).func_ctl().modify(|w| w.set_alt_select(IOC_PA13_FUNC_CTL_ESC0_SCL));

    // ESC specific functions
    IOC.pad(PA10).func_ctl().modify(|w| w.set_alt_select(IOC_PA10_FUNC_CTL_GPIO_A_10));
}


#[embassy_executor::main(entry = "hpm_hal::entry")]
async fn main(spawner: Spawner) -> ! {
    let p = hal::init(Default::default());

    defmt::info!("Board init!");

    // spawner.spawn(blink(p.PE14.degrade(), 100)).unwrap();
    // spawner.spawn(blink(p.PE15.degrade(), 200)).unwrap();
    spawner.spawn(blink(p.PE04.degrade(), 1000)).unwrap();

    defmt::info!("Tasks init!");

    let r = pac::ESC;

    //  let ecat_type = r.type_().read().type_();

    defmt::info!("addr = 0x{:x}", r.as_ptr() as u32);

    hal::sysctl::clock_add_to_group(pac::resources::ESC0, 0);

    //    defmt::info!("ECAT type: {:02x}", ecat_type);

    let max_sm = r.syncm_num().read().num();

    defmt::info!("max sync managers: {}", max_sm);

    loop {
        defmt::info!("tick");

        Timer::after_millis(1000).await;
    }
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    let mut err = heapless::String::<1024>::new();

    use core::fmt::Write as _;

    write!(err, "panic: {}", _info).ok();

    defmt::info!("{}", err.as_str());
    loop {}
}
