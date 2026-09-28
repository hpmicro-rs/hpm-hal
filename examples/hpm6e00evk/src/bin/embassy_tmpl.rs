#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]
#![feature(impl_trait_in_assoc_type)]

use assign_resources::assign_resources;
use embassy_executor::Spawner;
use embassy_time::Timer;
use embedded_io::Write as _;
use hal::gpio::{AnyPin, Level, Output, Pin as _};
use hal::mode::Blocking;
use hal::{pac, peripherals};
use {defmt_rtt as _, hpm_hal as hal};

assign_resources! {
    leds: Leds {
        red: PE14,
        green: PE15,
        blue: PE04,
    }
    buttons: Keys {
        keya: PB24,
        keyb: PB25,
    }
    // TJA1042T
    mcan4: Can {
        tx: PZ00,
        rx: PZ01,
        stby: PZ02,
        periph: MCAN4,
    }
    // FT2232 UART, default uart
    uart0: Uart0 {
        tx: PA00,
        rx: PA01,
        periph: UART0,
    }
    // UART0 or PUART
    puat: Puart {
        tx: PY00,
        rx: PY01,
        periph: PUART,
    }
    // RPi header
    spi: SpiRes {
        mosi: PF29,
        miso: PF28,
        sclk: PF26,
        ce0: PF25,
        ce1: PF24,
    },
    // RPi. pin3, pin5
    i2c1: I2c2 {
        sda: PY06,
        scl: PY07,
        periph: I2C1,
    }
    // RPi, pin27, pin28
    i2c0: I2c0 {
        sda: PB07,
        scl: PB06,
        periph: I2C0,
    }
    // WM896OCGEFL
    // LINPUT: HP_MIC, mic 插头
    // RINPUT: 板载电容麦
    // SPK_L, SPK_R: 喇叭插座
    i2s: I2sRes {
        mclk: PB11,
        bclk: PB01,
        fclk: PB10,
        txd: PB00,
        rxd: PB08,
        periph: I2S0,
        scl: PF13,
        sda: PF12,
        // control: I2C1,
    }
    // SPH0641LU4H, stereo microphone
    pdm: Pdm {
        clk: PB02,
        data: PB03,
        // data1: PB03,
        periph: PDM,
    }
    // NSI1306M25, RF调制器和解调器
    sdm: Sdm {
        clk: PF17,
        data: PE16,
        clk_out: PE19,
        periph: SDM0,
    }
    // NS4150B
    dao: Dao {
        rn: PF03,
        rp: PF04,
        periph: DAO,
    }
}

const BANNER: &str = include_str!("../../../assets/BANNER");

#[embassy_executor::task(pool_size = 3)]
async fn blink(pin: AnyPin, interval_ms: u32) {
    // all leds are active low
    let mut led = Output::new(pin, Level::Low, Default::default());
    loop {
        led.toggle();

        Timer::after_millis(interval_ms as u64).await;
    }
}

static mut UART: Option<hal::uart::Uart<'static, Blocking>> = None;

macro_rules! println {
    ($($arg:tt)*) => {
        unsafe {
            if let Some(uart) = UART.as_mut() {
                let _ = writeln!(uart, $($arg)*);
            }
        }
    };
}

#[embassy_executor::main(entry = "hpm_hal::entry")]
async fn main(spawner: Spawner) -> ! {
    let p = hal::init(Default::default());
    let r = split_resources!(p);

    let uart = hal::uart::Uart::new_blocking(r.uart0.periph, r.uart0.rx, r.uart0.tx, Default::default()).unwrap();
    unsafe { UART = Some(uart) }

    defmt::info!("Board init!");

    //let key_a = p.PB24;
    //let key_b = p.PB25;

    //let led_r = p.PE14; // PWM1_P_6
    //let led_g = p.PE15; // PWM1_P_7
    //let led_b = p.PE04; // PWM0_P_4

    spawner.spawn(blink(r.leds.red.degrade(), 1000)).unwrap();
    spawner.spawn(blink(r.leds.green.degrade(), 2000)).unwrap();
    spawner.spawn(blink(r.leds.blue.degrade(), 3000)).unwrap();
    defmt::info!("Tasks init!");

    println!("{}", BANNER);

    println!("Clocks: {:?}", hal::sysctl::clocks());
    println!(
        "XPI0: {}Hz (noinit if running from ram)",
        hal::sysctl::clocks().get_clock_freq(pac::clocks::XPI0).0
    );
    println!("MCT0: {}Hz", hal::sysctl::clocks().get_clock_freq(pac::clocks::MCT0).0);

    println!("Hello, world!");

    loop {
        defmt::info!("tick");

        Timer::after_millis(1000).await;

        println!("Hello, world! xxx");
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
