#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]
#![feature(impl_trait_in_assoc_type)]

use core::fmt::Write as _;

use assign_resources::assign_resources;
use embassy_executor::Spawner;
use embassy_time::{Delay, Instant, Timer};
use embedded_io::Write as _;
use hal::gpio::{AnyPin, Flex, Pin};
use hal::{pac, peripherals};
use hpm_hal as hal;
use hpm_hal::gpio::Output;
use hpm_hal::mode::{Async, Blocking};
use hpm_hal::spi::Spi;

const BOARD_NAME: &str = "HPM6750EVKMINI";

const BANNER: &str = include_str!("../../../assets/BANNER");

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
    // WiFi module
    rw007: RW007 {
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
    i2c3: I2c3 {
        sda: PB13,
        scl: PB14,
        i2c3: I2C3,
    }
    spi2: Spi2 {
        sclk: PB21,
        mosi: PB22,
        miso: PB25,
        csn: PB24,
        spi2: SPI2,
    }
    can: Can {
        tx: PE31,
        rx: PE30,
        can1: CAN1,
    }
}

static mut UART: Option<hal::uart::Uart<'static, Blocking>> = None;

macro_rules! println {
    ($($arg:tt)*) => {
        {
            if let Some(uart) = unsafe { UART.as_mut() } {
                writeln!(uart, $($arg)*).unwrap();
            }
        }
    }
}

pub mod cmds {
    pub const INIT: u8 = 0x00;
    pub const SET_MODE: u8 = 0x01;
    pub const MAC_GET: u8 = 0x02;
    pub const MAC_SET: u8 = 0x03;
    pub const GET_SN: u8 = 0x04;
    pub const GET_VSR: u8 = 0x05;
    pub const SCAN: u8 = 0x06;
    pub const JOIN: u8 = 0x07;
    pub const SOFTAP: u8 = 0x08;
    pub const DISCONNECT: u8 = 0x09;
    pub const AP_STOP: u8 = 0x0A;
    pub const AP_DEAUTH: u8 = 0x0B;
    pub const SCAN_STOP: u8 = 0x0C;
    pub const GET_RSSI: u8 = 0x0D;
    pub const SET_PWR_SAVE: u8 = 0x0E;
    pub const GET_PWR_SAVE: u8 = 0x0F;
    pub const CFG_PROMISC: u8 = 0x10;
    pub const CFG_FILTER: u8 = 0x11;
    pub const SET_CHANNEL: u8 = 0x12;
    pub const GET_CHANNEL: u8 = 0x13;
    pub const SET_COUNTRY: u8 = 0x14;
    pub const GET_COUNTRY: u8 = 0x15;
    pub const AP_MAC_GET: u8 = 0x16;
    pub const AP_MAC_SET: u8 = 0x17;
    pub const CFG_OTA: u8 = 0x18;
    pub const BLE_CFGWIFI: u8 = 0x19;

    // typed
    pub const FLAG_MRDY: u32 = 0x01;
    pub const MAGIC1: u32 = 0x67452301;
    pub const MAGIC2: u32 = 0xEFCDAB89;

    #[repr(C)]
    pub struct CmdRequest {
        pub flag: u32,
        pub m2s_len: u32, // master to slave data len
        pub magic1: u32,
        pub magic2: u32,
    }

    impl CmdRequest {
        pub fn new(m2s_len: u32) -> Self {
            CmdRequest {
                flag: FLAG_MRDY,
                m2s_len,
                magic1: MAGIC1,
                magic2: MAGIC2,
            }
        }

        pub fn as_bytes(&self) -> &[u8] {
            unsafe { core::slice::from_raw_parts(self as *const _ as *const u8, core::mem::size_of::<Self>()) }
        }
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

    let r = split_resources!(p);

    // use IOC for power domain PY pins
    r.uart.tx.set_as_ioc_gpio();
    r.uart.rx.set_as_ioc_gpio();

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

    // - MARK: Main
    let spi_config = hal::spi::Config::default();
    let mut spi: hal::spi::Spi<'_, Async> = Spi::new(
        r.rw007.spi1,
        r.rw007.sclk,
        r.rw007.mosi,
        r.rw007.miso,
        p.HDMA_CH2,
        p.HDMA_CH3,
        spi_config,
    );

    let mut cs = Output::new(r.rw007.cs, hal::gpio::Level::High, hal::gpio::Speed::Fast);
    let mut rst = Output::new(r.rw007.rst, hal::gpio::Level::High, hal::gpio::Speed::Fast);
    let mut int = Output::new(r.rw007.int, hal::gpio::Level::High, hal::gpio::Speed::Fast); // busy

    rst.set_low();
    Timer::after_millis(200).await;
    rst.set_high();

    println!("begin wifi test");

    let mut out = [0u8; 64];
    let cmd = [cmds::GET_SN, 0];
    //    trans_config.transfer_mode = hal::spi::TransMode::READ_WRITE;

    println!("int {}", int.is_set_high());

    let cmd = cmds::CmdRequest::new(1);
    cs.set_low();

    //    spi.blocking_write(cmd.as_bytes());
    spi.blocking_write(cmd.as_bytes());
    cs.set_high();

    println!("int {}", int.is_set_high());

    cs.set_low();
    spi.blocking_read(&mut out);

    cs.set_high();

    println!("SN: {:?}", out);

    let mut curr = riscv::register::mcycle::read64();
    loop {
        cs.set_low();
        spi.blocking_read(&mut out);

        cs.set_high();

        println!("SN: {:02x?}", out);

        Timer::after_millis(1000).await;
        let elapsed = riscv::register::mcycle::read64() - curr;
        curr = riscv::register::mcycle::read64();

        let now = Instant::now();
        println!("[{:6.3}] cycle: {}", (now.as_millis() as f32) / 1000.0, elapsed);
    }
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    let _ = println!("PANIC: {}", info);

    loop {}
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
