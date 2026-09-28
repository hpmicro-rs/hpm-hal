#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]
#![feature(impl_trait_in_assoc_type)]

use defmt::println;
use embassy_executor::Spawner;
use embassy_time::{Delay, Timer};
use embedded_hal::delay::DelayNs;
use embedded_io::Write as _;
use hal::gpio::{AnyPin, Flex, Pin};
use hal::pac;
use hpm_hal::mode::Blocking;
use hpm_hal::time::Hertz;
use {defmt_rtt as _, hpm_hal as hal};

const BOARD_NAME: &str = "HPM5300EVK";
const BANNER: &str = include_str!("../../../assets/BANNER");

#[embassy_executor::task]
async fn blink(pin: AnyPin) {
    let mut led = Flex::new(pin);
    led.set_as_output(Default::default());
    led.set_high();

    loop {
        led.toggle();

        Timer::after_millis(500).await;
    }
}

macro_rules! println {
    ($($arg:tt)*) => {
        let _ = writeln!(unsafe {(&mut *(&raw mut UART)).as_mut().unwrap()}, $($arg)*);
    };
}

static mut UART: Option<hal::uart::Uart<'static, Blocking>> = None;

#[embassy_executor::main(entry = "hpm_hal::entry")]
#[link_section = ".fast"]
async fn main(spawner: Spawner) -> ! {
    let mut config = hal::Config::default();
    {
        use hal::sysctl::*;
        config.sysctl.pll0 = Some(Pll {
            div: (0, 1, 3),
            freq_in: Hertz::mhz(960),
        });
        config.sysctl.cpu0 = ClockConfig::new(ClockMux::PLL0CLK0, 2);
        config.sysctl.ahb_div = AHBDiv::DIV3;
    }

    let p = hal::init(config);
    // let button = Input::new(p.PA03, Pull::Down); // hpm5300evklite, BOOT1_KEY
    let uart = hal::uart::Uart::new_blocking(p.UART0, p.PA01, p.PA00, Default::default()).unwrap();
    unsafe {
        UART = Some(uart);
    }

    println!("{}", BANNER);
    println!("{} init OK!", BOARD_NAME);
    println!("DCDC voltage: {}mV", pac::PCFG.dcdc_mode().read().volt());

    println!("Clock summary:");
    println!("  CPU0:\t{}Hz", hal::sysctl::clocks().cpu0.0);
    println!("  AHB:\t{}Hz", hal::sysctl::clocks().ahb.0);
    println!(
        "  XPI0:\t{}Hz",
        hal::sysctl::clocks().get_clock_freq(hal::pac::clocks::XPI0).0
    );
    println!(
        "  MTMR:\t{}Hz",
        hal::sysctl::clocks().get_clock_freq(pac::clocks::MCT0).0
    );

    println!("==============================");

    println!("Hello, world!");

    {
        pac::SYSCTL.monitor(0).control().modify(|w| {
            w.set_accuracy(true); // 1Hz
            w.set_reference(true); // 24M
            w.set_mode(true); // save to min and max
            w.set_selection(pac::sysctl::vals::MonitorSelection::CLK_TOP_CPU0); // pll0 clk0
            w.set_start(true);
        });
        while !pac::SYSCTL.monitor(0).control().read().valid() {}
        println!(
            "Monitor 0 measure: {} min={} max={}!",
            pac::SYSCTL.monitor(0).current().read().frequency(),
            pac::SYSCTL.monitor(0).low_limit().read().frequency(),
            pac::SYSCTL.monitor(0).high_limit().read().frequency()
        );
    }

    //let mie = riscv::register::mie::read();
    //println!("mie: {:?}", mie);

    spawner.spawn(blink(p.PA10.degrade())).unwrap();

    unsafe {
        riscv::register::mcounteren::set_cy();
    }

    let mut tick = riscv::register::mcycle::read64();
    loop {
        // Timer::after_millis(1000).await;
        Delay.delay_ms(1000);

        println!("tick {}", riscv::register::mcycle::read64() - tick);
        tick = riscv::register::mcycle::read64();
    }
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    println!("\n\n\nPANIC:\n{}", info);

    loop {}
}
