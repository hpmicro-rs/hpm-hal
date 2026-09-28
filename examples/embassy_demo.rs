#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]
#![feature(impl_trait_in_assoc_type)]

use defmt::println;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_time::Timer;
use embedded_io::Write as _; // `writeln!` provider
use hal::gpio::{AnyPin, Flex, Pin};
use hal::mode::Blocking;
use hal::pac;
use hal::pac::MCHTMR;
use hpm_hal as hal;
use hpm_hal::gpio::{Input, Pull};

const BANNER: &str = include_str!("../../../assets/BANNER");

#[embassy_executor::task(pool_size = 4)]
async fn blink(pin: AnyPin) {
    let mut led = Flex::new(pin);
    led.set_as_output(Default::default());
    led.set_high();

    loop {
        led.toggle();

        Timer::after_millis(500).await;
    }
}

#[embassy_executor::task]
async fn button(pin: AnyPin) {
    let mut button = Input::new(pin, Pull::Down); // hpm5300evklite, BOOT1_KEY
    loop {
        button.wait_for_falling_edge().await;
        defmt::info!("PA03 Button pressed! {}", button.is_high());
    }
}

static mut UART: Option<hal::uart::Uart<'static, Blocking>> = None;

#[embassy_executor::main(entry = "hpm_hal::entry")]
async fn main(spawner: Spawner) -> ! {
    let mut config = hal::Config::default();
    {
        use hal::sysctl::*;
        config.sysctl.pll0 = Some(Pll {
            mfi: 40,
            mfn: 0,
            div: (0, 1, 3),
        });
    }
    let p = hal::init(config);
    let uart = hal::uart::Uart::new_blocking(p.UART0, p.PA01, p.PA00, Default::default()).unwrap();
    unsafe {
        UART = Some(uart);
    }
    let uart = unsafe { UART.as_mut().unwrap() };

    writeln!(uart, "{}", BANNER).unwrap();
    writeln!(uart, "Board inited OK!").unwrap();

    writeln!(uart, "Clock summary:").unwrap();
    writeln!(uart, "  CPU0:\t{}Hz", hal::sysctl::clocks().cpu0.0).unwrap();
    writeln!(uart, "  AHB:\t{}Hz", hal::sysctl::clocks().ahb.0).unwrap();
    writeln!(
        uart,
        "  XPI0:\t{}Hz",
        hal::sysctl::clocks().get_clock_freq(pac::clocks::XPI0).0
    )
    .unwrap();
    writeln!(
        uart,
        "  MCHTMR:\t{}Hz",
        hal::sysctl::clocks().get_clock_freq(pac::clocks::MCT0).0
    )
    .unwrap();

    println!("Hello, world!");

    //let mie = riscv::register::mie::read();
    //println!("mie: {:?}", mie);

    spawner.spawn(blink(p.PA10.degrade())).unwrap();

    spawner.spawn(button(p.PA03.degrade())).unwrap();

    loop {
        Timer::after_millis(1000).await;

        // ::info!("tick {}", MCHTMR.mtime().read());
    }
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    //let _ = println!("\n\n\n{}", info);

    loop {}
}
