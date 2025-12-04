#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]
#![feature(impl_trait_in_assoc_type)]

use defmt::println;
use embassy_executor::Spawner;
use embassy_time::Timer;
use hal::gpio::{AnyPin, Flex};
use hal::pac::MCHTMR;
use hal::Peri;
use {defmt_rtt as _, hpm_hal as hal};

const BOARD_NAME: &str = "HPM6200EVK";

#[embassy_executor::task(pool_size = 3)]
async fn blink(pin: Peri<'static, AnyPin>, interval: u64) {
    let mut led = Flex::new(pin);
    led.set_as_output(Default::default());
    led.set_high();

    loop {
        led.toggle();

        Timer::after_millis(interval).await;
    }
}

#[embassy_executor::main(entry = "hpm_hal::entry")]
async fn main(spawner: Spawner) -> ! {
    let p = hal::init(Default::default());

    println!("Rust SDK: hpm-hal v0.0.1");
    println!("Embassy driver: hpm-hal v0.0.1");
    println!("Author: @andelf");
    println!("==============================");
    println!(" {} clock summary", BOARD_NAME);
    println!("==============================");
    println!("cpu0:\t\t {}Hz", hal::sysctl::clocks().cpu0.0);
    println!("ahb:\t\t {}Hz", hal::sysctl::clocks().ahb.0);
    println!("==============================");

    println!("Hello, world!");

    // RGB LED pins: PA27 (R), PB01 (G), PB19 (B)
    spawner.must_spawn(blink(p.PA27.into(), 500));
    spawner.must_spawn(blink(p.PB01.into(), 300));
    spawner.must_spawn(blink(p.PB19.into(), 200));

    loop {
        Timer::after_millis(1000).await;

        defmt::info!("Main loop tick - mtime: 0x{:016X}", MCHTMR.mtime().read());
    }
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {}
}
