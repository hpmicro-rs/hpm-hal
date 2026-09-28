#![no_main]
#![no_std]

use core::str;

use defmt_rtt as _;
use embedded_io::Write as _; // `writeln!` provider
use hpm_hal as hal;
use hpm_hal::gpio::{Level, Output, Speed};
use hpm_hal::uart::{UartRx, UartTx};
use riscv::delay::McycleDelay;

#[hal::entry]
fn main() -> ! {
    let p = hal::init(Default::default());

    // let mut delay = McycleDelay::new(hal::sysctl::clocks().cpu0.0);

    defmt::info!("Board init!");

    //    let mut tx = UartTx::new_blocking(p.UART0, p.PA00, Default::default()).unwrap();
    let uart_config = hal::uart::Config::default();
    //uart_config.fifo_level = None;
    //let mut rx = UartRx::new_blocking(p.UART1, p.PA06, uart_config).unwrap();
    let mut uart = hal::uart::Uart::new_blocking(p.UART0, p.PA01, p.PA00, uart_config).unwrap();
    //let mut uart = hal::uart::Uart::new_blocking(p.UART2, p.PB09, p.PB08, uart_config).unwrap();
    let mut buf = [0u8; 4];

    //let mut led = Output::new(p.PA23, Level::Low, Speed::default());
    let mut led = Output::new(p.PA10, Level::Low, Speed::default());

    writeln!(uart, "UART init OK!").unwrap();

    loop {
        uart.blocking_read(&mut buf).unwrap();
        for b in &mut buf {
            // convert to upper case
            if *b >= b'a' && *b <= b'z' {
                *b -= (b'a' - b'A');
            } else if *b == b'\r' {
                *b = b'\n';
            }
        }

        unsafe {
            writeln!(uart, "{}", str::from_utf8_unchecked(&buf)).unwrap();
        }

        defmt::info!("tick! {:?}", buf);

        led.toggle();
        //        delay.delay_ms(1000);

        //    led.set_low();
        //      delay.delay_ms(1000);
    }
}

#[panic_handler]
unsafe fn panic(_info: &core::panic::PanicInfo) -> ! {
    defmt::info!("panic!");
    loop {}
}
