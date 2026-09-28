#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]
#![feature(impl_trait_in_assoc_type)]
#![feature(abi_riscv_interrupt)]

use core::fmt::Write as _;

use defmt::println;
use embassy_executor::Spawner;
use embassy_time::{Delay, Instant, Timer};
use embedded_hal::delay::DelayNs;
use embedded_io::Write as _;
use hal::gpio::{AnyPin, Flex, Pin};
use hal::pac;
use hpm_hal::mode::Blocking;
use hpm_hal::time::Hertz;
use hpm_hal::{bind_interrupts, peripherals};
use {defmt_rtt as _, hpm_hal as hal};

bind_interrupts!(struct Irqs {
    UART0 => hal::uart::InterruptHandler<peripherals::UART0>;
});

const BOARD_NAME: &str = "HPM5300EVK";
const BANNER: &str = include_str!("../../../assets/BANNER");

#[embassy_executor::task(pool_size = 2)]
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

    spawner.spawn(blink(p.PA23.degrade())).unwrap();
    spawner.spawn(blink(p.PA10.degrade())).unwrap();

    // let button = Input::new(p.PA03, Pull::Down); // hpm5300evklite, BOOT1_KEY
    let mut uart = hal::uart::UartRx::new(p.UART0, p.PA01, Irqs, p.HDMA_CH1, Default::default()).unwrap();

    let mut buf = [0u8; 256];

    defmt::info!("Loop");

    uart.read(&mut buf[..10]).await.unwrap();

    defmt::info!("Read: {}", buf);

    loop {
        Timer::after_millis(1000).await;

        uart.read(&mut buf[..10]).await.unwrap();

        let s = core::str::from_utf8(&buf[..10]).unwrap();

        defmt::info!("Read: {}", s);
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
