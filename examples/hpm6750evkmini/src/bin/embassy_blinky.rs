#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]
#![feature(impl_trait_in_assoc_type)]

use embassy_executor::Spawner;
use embassy_time::Timer;
use hal::Peri;
use hpm_hal as hal;
use hpm_hal::gpio::{AnyPin, Level, Output};

#[embassy_executor::task(pool_size = 3)]
async fn blink(pin: Peri<'static, AnyPin>, interval_ms: u64) {
    let mut led = Output::new(pin, Level::Low, Default::default());

    loop {
        led.toggle();

        Timer::after_millis(interval_ms).await;
    }
}

#[embassy_executor::main(entry = "hpm_hal::entry")]
async fn main(spawner: Spawner) -> ! {
    let p = hal::init(Default::default());

    // RGB LED: PB19 (R), PB18 (G), PB20 (B)
    spawner.spawn(blink(p.PB19.into(), 500)).unwrap();
    spawner.spawn(blink(p.PB18.into(), 200)).unwrap();
    spawner.spawn(blink(p.PB20.into(), 300)).unwrap();

    loop {
        Timer::after_millis(1000).await;
    }
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {}
}
