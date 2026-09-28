#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]
#![feature(impl_trait_in_assoc_type)]

use assign_resources::assign_resources;
use defmt::println;
use embassy_time::{Delay, Duration, Timer};
use embedded_io::Write as _; // `writeln!` provider
use hal::gpio::{Level, Output, Speed};
use hal::i2c::I2c;
use hal::mode::Blocking;
use hal::peripherals;
use hpm_hal::gpio::Pin;
use {defmt_rtt as _, hpm_hal as hal};

const BOARD_NAME: &str = "HPM5300EVK";
const BANNER: &str = include_str!("../../../assets/BANNER");

assign_resources! {
    leds: Leds {
        // PWM1_P_7
        red: PA23,
    }
    // FT2232 UART, default uart
    uart: Uart0 {
        tx: PA00,
        rx: PA01,
        uart0: UART0,
    }
    i2c: I2c1 {
        sda: PB06,
        scl: PB07,
        i2c1: I2C1,
    }
}

// - MARK: Main

#[embassy_executor::main(entry = "hpm_hal::entry")]
async fn main(spawner: embassy_executor::Spawner) -> ! {
    let config = hal::Config::default();
    let p = hal::init(config);

    let r = split_resources!(p);

    let mut led = Output::new(r.leds.red, Level::Low, Default::default());

    let mut i2c_config = hal::i2c::Config::default();
    {
        use hal::i2c::*;
        i2c_config.mode = I2cMode::Fast;
        i2c_config.timeout = Duration::from_secs(1);
    }
    let i2c = I2c::new_blocking(r.i2c.i2c1, r.i2c.scl, r.i2c.sda, i2c_config);

    let mut bmp085 = edrv_bmp180::BMP180::new_primary(i2c);
    bmp085.init(Default::default()).unwrap();

    loop {
        // let temp = bmp085.read_temperature(&mut Delay).unwrap();
        let measure = bmp085.read_measurement(&mut Delay).unwrap();

        defmt::info!(
            "Temp: {}, Pressure: {}hPa",
            measure.temperature,
            (measure.pressure as f32) / 100.0
        );

        Timer::after_millis(1000).await;

        led.toggle();
    }
}

#[panic_handler]
unsafe fn panic(info: &core::panic::PanicInfo) -> ! {
    defmt::println!("panic!\n {}", defmt::Debug2Format(info));

    loop {}
}
