#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]
#![feature(impl_trait_in_assoc_type)]

use assign_resources::assign_resources;
use embassy_executor::Spawner;
use embassy_time::{Delay, Instant, Timer};
use embedded_io::Write as _;
use hal::gpio::{AnyPin, Flex, Pin};
use hal::{pac, peripherals};
use hpm_hal::gpio::{Input, Level, Output, Pull};
use hpm_hal::mode::{Async, Blocking};
use {defmt_rtt as _, hpm_hal as hal};

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
        tx_dma: HDMA_CH1,
        rx_dma: HDMA_CH2,
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

pub const FLAG_MRDY: u32 = 0x01;

pub const CMD_MAGIC1: u32 = 0x67452301;
pub const CMD_MAGIC2: u32 = 0xEFCDAB89;

pub const RESP_MAGIC1: u32 = 0x98BADCFE;
pub const RESP_MAGIC2: u32 = 0x10325476;

#[derive(Debug, Clone, Copy)]
pub enum PhaseType {
    MasterCmd = 0x01,
    MasterData = 0x02,
    SlaveCmd = 0x03,
    SlaveData = 0x04,
}

#[derive(Debug, Clone, Copy)]
pub enum TransferState {
    DataSuccess = 0,
    DataError = 1,
    DataContinue = 2,
}

#[derive(Debug, Clone, Copy)]
#[repr(C)]
pub struct MasterRequest {
    /*
     uint8_t reserve1;
    uint8_t flag: 4;
    uint8_t type: 4;
    uint16_t reserve2;
     */
    pub flag: u32,
    pub seq: u16,
    pub len: u16, // master to slave data len
    pub magic1: u32,
    pub magic2: u32,
}
#[derive(Debug, Clone, Copy)]
#[repr(C)]
pub struct SlaveResponse {
    /*
    uint8_t reserve1;
    uint8_t flag: 4;
    uint8_t type: 4;
    uint8_t slave_rx_buf: 4;
    uint8_t slave_tx_buf: 4;
    uint8_t reserve2;
     */
    pub flag: u32,
    pub seq: u16,
    pub len: u16, // slave to master data len
    pub magic1: u32,
    pub magic2: u32,
}

#[derive(Debug, Clone, Copy)]
#[repr(C)]
pub struct ReqResp {
    pub flag: u32,
    pub seq: u16,
    pub len: u16, // master to slave data len
    pub magic1: u32,
    pub magic2: u32,
}

impl ReqResp {
    pub fn new(phase: PhaseType, m2s_len: u16) -> Self {
        let mut this = ReqResp {
            flag: FLAG_MRDY,
            seq: 0,
            len: m2s_len,
            magic1: CMD_MAGIC1,
            magic2: CMD_MAGIC2,
        };
        this.flag = this.flag | ((phase as u32) << (8 + 4));

        this
    }

    pub fn phase(&self) -> PhaseType {
        match (self.flag >> (8 + 4)) & 0x0F {
            0x01 => PhaseType::MasterCmd,
            0x02 => PhaseType::MasterData,
            0x03 => PhaseType::SlaveCmd,
            0x04 => PhaseType::SlaveData,
            _ => unreachable!(),
        }
    }

    pub fn as_bytes(&self) -> &[u8] {
        unsafe { core::slice::from_raw_parts(self as *const _ as *const u8, core::mem::size_of::<Self>()) }
    }

    pub fn from_bytes(bytes: &[u8]) -> &Self {
        unsafe { &*(bytes.as_ptr() as *const Self) }
    }

    pub fn from_bytes_mut(bytes: &mut [u8]) -> &mut Self {
        unsafe { &mut *(bytes.as_mut_ptr() as *mut Self) }
    }
}

pub struct RW007 {
    spi: hal::spi::Spi<'static, Async>,
    cs: Output<'static>,
    busy: Input<'static>,
    inited: bool,
}

impl RW007 {
    pub fn new(spi: hal::spi::Spi<'static, Async>, cs: Output<'static>, busy: Input<'static>) -> Self {
        RW007 {
            spi,
            cs,
            busy,
            inited: false,
        }
    }

    pub fn is_busy(&self) -> bool {
        self.busy.is_low()
    }

    pub async fn send_data(&mut self, buf: &[u8]) {}

    pub async fn write(&mut self, data: &[u8]) {
        self.cs.set_low();
        self.spi.write(data).await.unwrap();
        self.cs.set_high();
    }

    pub async fn read(&mut self, data: &mut [u8]) {
        self.cs.set_low();
        self.spi.read(data).await.unwrap();
        self.cs.set_high();
    }
}

#[embassy_executor::task]
async fn rw007_task(r: RW007Resource) {
    let spi = hal::spi::Spi::new(r.spi1, r.sclk, r.mosi, r.miso, r.tx_dma, r.rx_dma, Default::default());
    // let spi_bus = embedded_hal_bus::util::AtomicCell::new(spi);
    let cs = Output::new(r.cs, Level::High, Default::default());
    let busy = Input::new(r.int, Pull::None);
    let mut rst = Output::new(r.rst, Level::High, Default::default());

    let mut rw007 = RW007::new(spi, cs, busy);
    //  let spi_dev = embedded_hal_bus::spi::AtomicDevice::new(&spi_bus, cs, Delay).unwrap();

    rst.set_low();
    Timer::after_millis(200).await;
    rst.set_high();

    let mut buf = [0u8; 64];
    let mut out = [0u8; 64];

    while rw007.is_busy() {
        println!("busy wait");
    }

    println!("busy out");

    // Stage 1: Send command to rw007
    let mut r = ReqResp::new(PhaseType::MasterCmd, 16);
    r.flag = r.flag & (!0x1);

    let mut buf = [0u8; 16];
    buf.copy_from_slice(r.as_bytes());
    rw007.blocking_write(&buf[..16]);

    loop {
        Timer::after_millis(1000).await;
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
    // spawner.spawn(blink(r.leds.b.degrade(), 300)).unwrap();
    spawner.spawn(blink(r.power_leds.pled.degrade(), 300)).unwrap();

    spawner.must_spawn(rw007_task(r.rw007));

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
