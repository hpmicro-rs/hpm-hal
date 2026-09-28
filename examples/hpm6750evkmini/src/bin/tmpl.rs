#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]
#![feature(impl_trait_in_assoc_type)]

use assign_resources::assign_resources;
use embassy_executor::Spawner;
use embassy_time::{Instant, Timer};
use embedded_io::Write as _;
use hal::gpio::{AnyPin, Flex, Pin};
use hal::{pac, peripherals};
use hpm_hal as hal;
use hpm_hal::mode::Blocking;

const BOARD_NAME: &str = "HPM6750EVKMINI";
const BANNER: &str = include_str!("../../../assets/BANNER");

static mut UART: Option<hal::uart::Uart<'static, Blocking>> = None;

macro_rules! println {
    ($($arg:tt)*) => {
        {
            if let Some(uart) = unsafe { (&mut *(&raw mut UART)).as_mut() } {
                writeln!(uart, $($arg)*).unwrap();
            }
        }
    }
}

assign_resources! {
    leds: Leds {
        // PWM1_P0
        r: PB19,
        // PWM1_P1
        g: PB18,
        // PWM0_P7
        b: PB20,
    }
    power_leds: PowerLeds {
        pled: PZ04,
        wled: PZ05,
    }
    power: PowerDomain {
        pbutn: PZ02,
        wbutn: PZ03,
        resetn: PZ01,
        boot0: PZ06,
        boot1: PZ07,
    }
    // FT2232 UART, default uart
    uart: Uart0 {
        tx: PY06,
        rx: PY07,
        uart0: UART0,
    }
    uart6: Uart6 {
        tx: PE28,
        rx: PE27,
        uart6: UART6,
    }
    uart13: Uart13 {
        tx: PZ09,
        rx: PZ08,
        uart13: UART13,
    }
    uart14: Uart14 {
        tx: PZ11,
        rx: PZ10,
        uart14: UART14,
    }
    rw007: RW007Resource {
        rst: PE02,
        int: PE01,
        // SPI1
        cs: PE03,
        mosi: PE04,
        miso: PD30,
        sclk: PD31,
        spi1: SPI1,
    }
    // PWM3_P4
    buzzer: Buzzer {
        pin: PE05,
    }
    // IS31AP2005
    // SPH0641LM4H-1 mic, stereo
    dao: Dao {
        inp: PB15,
        inn: PB16,
        dao: DAO,
    }
    mic: Microphone {
        clk: PY10,
        dat: PY11,
        vad: VAD,
    }
    sdcard: SdCard {
        // SDC1
        cdn: PD28,
        // SD_VSelect=0,Vout=1.25V*(R1/R2+1)=3.3V
        // SD_VSelect=1,Vout=1.25V*(R1/R2+1)=1.8V
        vsel: PD29,
        clk: PD22,
        cmd: PD21,
        data0: PD18,
        data1: PD17,
        data2: PD27,
        data3: PD26,
        sdxc0: SDXC0,
    }
    // RGB888, CAM
    i2c0: I2c0 {
        sda: PB10,
        scl: PB11,
        i2c0: I2C0,
    }
    // P1 Header
    i2c3: I2c3 {
        sda: PB13,
        scl: PB14,
        i2c3: I2C3,
    }
    // P1 Header
    spi2: Spi2 {
        sclk: PB21,
        mosi: PB22,
        miso: PB25,
        csn: PB24,
        spi2: SPI2,
    }
    // P1 Header
    can: Can {
        tx: PE31,
        rx: PE30,
        can1: CAN1,
    }
}

#[embassy_executor::main(entry = "hpm_hal::entry")]
async fn main(spawner: Spawner) -> ! {
    // let p = hal::init(Default::default());
    let mut config = hal::Config::default();
    {
        use hal::sysctl::*;
        config.sysctl.cpu0 = ClockConfig::new(ClockMux::PLL0CLK0, 1);

        config.sysctl.ahb = ClockConfig::new(ClockMux::PLL1CLK1, 4); // AHB = 100M
    }
    let p = hal::init(config);

    hpm6750evkmini::init_ext_ram();

    let r = split_resources!(p);

    // 115200 8N1
    let uart = hal::uart::Uart::new_blocking(r.uart.uart0, r.uart.rx, r.uart.tx, Default::default()).unwrap();
    unsafe { UART = Some(uart) };

    println!("{}", BANNER);
    println!("Board: {}", BOARD_NAME);

    println!("Clock summary:");
    println!("  CPU0:\t{}Hz", hal::sysctl::clocks().cpu0.0);
    println!("  CPU1:\t{}Hz", hal::sysctl::clocks().cpu1.0);
    println!("  AHB:\t{}Hz", hal::sysctl::clocks().ahb.0);
    println!(
        "  AXI0:\t{}Hz",
        hal::sysctl::clocks().get_clock_freq(pac::clocks::AXI).0
    );
    // not the same as hpm_sdk, which calls it axi1, axi2
    println!(
        "  CONN:\t{}Hz",
        hal::sysctl::clocks().get_clock_freq(pac::clocks::CONN).0
    );
    println!("  VIS:\t{}Hz", hal::sysctl::clocks().get_clock_freq(pac::clocks::VIS).0);
    println!(
        "  XPI0:\t{}Hz",
        hal::sysctl::clocks().get_clock_freq(pac::clocks::XPI0).0
    );
    println!(
        "  FEMC:\t{}Hz",
        hal::sysctl::clocks().get_clock_freq(pac::clocks::FEMC).0
    );
    // DISP subsystem
    println!(
        "  LCDC:\t{}Hz",
        hal::sysctl::clocks().get_clock_freq(pac::clocks::LCDC).0
    );
    println!(
        "  MTMR:\t{}Hz",
        hal::sysctl::clocks().get_clock_freq(pac::clocks::MCHTMR0).0
    );

    spawner.spawn(blink(r.leds.r.degrade(), 500)).unwrap();
    spawner.spawn(blink(r.leds.g.degrade(), 200)).unwrap();
    spawner.spawn(blink(r.leds.b.degrade(), 300)).unwrap();

    let mut curr = riscv::register::mcycle::read64();
    loop {
        Timer::after_millis(1000).await;
        let elapsed = riscv::register::mcycle::read64() - curr;
        curr = riscv::register::mcycle::read64();

        let now = Instant::now();
        println!("[{:6.3}] cycle: {}", (now.as_millis() as f32) / 1000.0, elapsed);
    }
}

#[embassy_executor::task(pool_size = 3)]
async fn blink(pin: AnyPin, interval_ms: u64) {
    let mut led = Flex::new(pin);
    led.set_as_output(Default::default());
    led.set_high();

    loop {
        led.toggle();

        Timer::after_millis(interval_ms).await;
    }
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    let _ = println!("\n\n\n{}", info);

    loop {}
}
