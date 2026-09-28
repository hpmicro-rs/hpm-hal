#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]
#![feature(impl_trait_in_assoc_type)]

use core::fmt::Write as _;

use assign_resources::assign_resources;
use cmds::{CmdHeader, DataPacket, DataType, ReqResp};
use embassy_executor::Spawner;
use embassy_time::{Delay, Instant, Timer};
use embedded_hal::spi::SpiBus;
use embedded_io::Write as _;
use hal::gpio::{AnyPin, Flex, Pin};
use hal::{pac, peripherals};
use hpm_hal as hal;
use hpm_hal::gpio::{Input, Output, Pull};
use hpm_hal::mode::{Async, Blocking};
use hpm_hal::spi::{Spi, TransferConfig};

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
            if let Some(uart) = unsafe { (&mut *(&raw mut UART)).as_mut() } {
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

    /*
    struct spi_master_request
    {
        uint8_t reserve1;
        uint8_t flag: 4;
        uint8_t type: 4;
        uint16_t reserve2;
        uint16_t seq;
        uint16_t M2S_len; // master to slave data len.
        uint32_t magic1;
        uint32_t magic2;
    };
    struct spi_slave_response
    {
        uint8_t reserve1;
        uint8_t flag: 4;
        uint8_t type: 4;
        uint8_t slave_rx_buf: 4;
        uint8_t slave_tx_buf: 4;
        uint8_t reserve2;
        uint16_t seq;
        uint16_t S2M_len; // slave to master data len.
        uint32_t magic1;
        uint32_t magic2;
    };
    */
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

        pub fn set_phase(&mut self, phase: PhaseType) {
            self.flag = self.flag & !(0x0F << (8 + 4));
            self.flag = self.flag | ((phase as u32) << (8 + 4));
        }

        pub fn set_seq(&mut self, seq: u16) {
            self.seq = seq;
        }

        pub fn is_response(&self) -> bool {
            self.magic1 == RESP_MAGIC1 && self.magic2 == RESP_MAGIC2
        }

        pub fn is_ready(&self) -> bool {
            self.flag & FLAG_MRDY != 0
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

    #[derive(Debug, Clone, Copy)]
    #[repr(C)]
    pub struct DataPacket {
        pub data_len: u32,
        pub data_type: DataType,
    }
    impl DataPacket {
        pub fn as_bytes(&self) -> &[u8] {
            unsafe { core::slice::from_raw_parts(self as *const _ as *const u8, core::mem::size_of::<Self>()) }
        }

        pub fn from_bytes(bytes: &[u8]) -> &Self {
            unsafe { &*(bytes.as_ptr() as *const Self) }
        }
    }

    #[derive(Debug, Clone, Copy)]
    #[repr(C)]
    pub struct CmdHeader {
        pub cmd: u32,
        pub len: u32,
    }

    impl CmdHeader {
        pub fn as_bytes(&self) -> &[u8] {
            unsafe { core::slice::from_raw_parts(self as *const _ as *const u8, core::mem::size_of::<Self>()) }
        }

        pub fn from_bytes(bytes: &[u8]) -> &Self {
            unsafe { &*(bytes.as_ptr() as *const Self) }
        }
    }

    pub const SHARED_ENABLED: u32 = 0x00008000;
    pub const WPA_SECURITY: u32 = 0x00200000;
    pub const WPA2_SECURITY: u32 = 0x00400000;
    pub const WPS_ENABLED: u32 = 0x10000000;
    pub const WEP_ENABLED: u32 = 0x0001;
    pub const TKIP_ENABLED: u32 = 0x0002;
    pub const AES_ENABLED: u32 = 0x0004;
    pub const WSEC_SWFLAG: u32 = 0x0008;

    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    #[repr(u32)]
    pub enum Security {
        Open = 0,
        WepPsk = WEP_ENABLED,
        WepShared = WEP_ENABLED | SHARED_ENABLED,
        WpaTkipPsk = WPA_SECURITY | TKIP_ENABLED,
        WpaAesPsk = WPA_SECURITY | AES_ENABLED,
        Wpa2AesPsk = WPA2_SECURITY | AES_ENABLED,
        Wpa2TkipPsk = WPA2_SECURITY | TKIP_ENABLED,
        Wpa2MixedPsk = WPA2_SECURITY | AES_ENABLED | TKIP_ENABLED,
        WpsOpen = WPS_ENABLED,
        WpsSecure = WPS_ENABLED | AES_ENABLED,
        Unknown = 0xFFFFFFFF, // Representing -1 as u32
        Force32Bit = 0x7FFFFFFF,
    }

    // cmd

    #[derive(Debug, Clone, Copy)]
    #[repr(u32)]
    pub enum DataType {
        StaEthData = 1,
        ApEthData = 2,
        PromiscData = 3,
        Cmd = 4,
        Resp = 5,
        CallBack = 6,
        BLE = 7,
    }

    #[derive(Debug, Clone, Copy)]
    pub struct CmdPacket(pub u32);

    // traits
    pub trait Response {}

    pub trait Cmd: Sized {
        type Response: Response;
        const CMD: u8;

        fn as_bytes(&self) -> &[u8] {
            unsafe { core::slice::from_raw_parts(self as *const _ as *const u8, core::mem::size_of::<Self>()) }
        }

        fn from_bytes(bytes: &[u8]) -> &Self {
            unsafe { &*(bytes.as_ptr() as *const Self) }
        }
    }

    impl Response for () {}

    /// rw009_cmd_init
    #[derive(Debug, Clone, Copy)]
    #[repr(C)]
    pub struct Init {}
    impl Cmd for Init {
        type Response = ();
        const CMD: u8 = 0x00;
    }
}

pub struct RW007 {
    spi: hal::spi::Spi<'static, Async>,
    cs: Output<'static>,
    busy: Input<'static>,
}
impl RW007 {
    pub fn new(spi: hal::spi::Spi<'static, Async>, cs: Output<'static>, busy: Input<'static>) -> Self {
        RW007 { spi, cs, busy }
    }

    pub fn blocking_write(&mut self, data: &[u8]) {
        self.cs.set_low();
        self.spi.blocking_write(data);
        self.cs.set_high();
    }

    pub fn blocking_read(&mut self, data: &mut [u8]) {
        self.cs.set_low();
        self.spi.blocking_read(data);
        self.cs.set_high();
    }

    pub fn blocking_transfer_inplace(&mut self, data: &mut [u8]) {
        let mut cfg = TransferConfig::default();
        cfg.transfer_mode = hal::spi::TransMode::WRITE_READ_TOGETHER;
        self.cs.set_low();
        self.spi.blocking_transfer_inplace(data, &cfg).unwrap();
        self.cs.set_high();
    }

    pub fn send_cmd_request(&mut self, len: u16) {
        let r = cmds::ReqResp::new(cmds::PhaseType::MasterCmd, len);

        let mut buf = [0u8; 16];
        buf.copy_from_slice(r.as_bytes());
        self.blocking_write(&buf);

        while self.busy.is_low() {}
    }

    pub fn wait_response_rdy(&mut self) -> bool {
        let mut buf = [0u8; 16];
        self.blocking_read(&mut buf);
        let h = cmds::ReqResp::from_bytes(&buf);
        println!("header: {:#x?} ready = {}", h, h.is_ready());

        h.is_ready()
    }

    pub fn is_busy(&self) -> bool {
        self.busy.is_low()
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

    // hpm6750evkmini::init_ext_ram();

    let r = split_resources!(p);

    let uart = hal::uart::Uart::new_blocking(r.uart.uart0, r.uart.rx, r.uart.tx, Default::default()).unwrap();
    unsafe { UART = Some(uart) };

    println!("init ok");

    {
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
    }

    spawner.spawn(blink(r.leds.r.degrade(), 1000)).unwrap();
    //    spawner.spawn(blink(r.leds.g.degrade(), 200)).unwrap();
    //  spawner.spawn(blink(r.leds.b.degrade(), 300)).unwrap();

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
    let mut int = Input::new(r.rw007.int, Pull::None); // busy

    rst.set_low();
    Timer::after_millis(200).await;
    rst.set_high();

    let mut rw007 = RW007::new(spi, cs, int);

    while rw007.is_busy() {}

    let buf = [0u8; 16];

    println!("begin wifi test");

    let mut buf = [0u8; 64];
    let mut out = [0u8; 64];
    //let cmd = [cmds::GET_SN, 0];
    //    trans_config.transfer_mode = hal::spi::TransMode::READ_WRITE;

    println!("BB busy {}", rw007.busy.is_high());
    while rw007.busy.is_low() {}

    Timer::after_millis(2000).await;

    // Stage 1: Send command to rw007
    let mut r = cmds::ReqResp::new(cmds::PhaseType::MasterCmd, 16);
    r.flag = r.flag & (!0x1);

    let mut buf = [0u8; 16];
    buf.copy_from_slice(r.as_bytes());
    rw007.blocking_write(&buf[..16]);

    while rw007.is_busy() {}

    println!("written");

    let mut buf = [0u8; 16];
    rw007.blocking_read(&mut buf);
    println!("BB busy {}", rw007.is_busy());
    let h = cmds::ReqResp::from_bytes(&buf);
    println!("resp: {}", h.is_response());
    println!("header: {:#x?} ready = {}", h, h.is_ready());
    println!("phase: {:?}", h.phase());

    while rw007.is_busy() {} // rising edge

    let mut data_pkt = DataPacket {
        data_len: 8,
        data_type: DataType::Cmd,
    };
    let init_cmd = CmdHeader {
        cmd: 0x00, // RW00x_CMD_INIT
        len: 0,
    };

    // ///
    //
    //     let mut r = cmds::ReqResp::new(cmds::PhaseType::MasterCmd, 16);

    let mut buf = [0u8; 16];

    let r = cmds::ReqResp::new(cmds::PhaseType::MasterData, 16);
    buf.copy_from_slice(r.as_bytes());

    println!("buf {:02x?}", &buf);

    rw007.blocking_transfer_inplace(&mut buf[..16]);

    println!("buf {:02x?}", &buf);
    // let mut buf = [0u8; 16];
    //   rw007.blocking_read(&mut buf);
    println!("BB busy {}", rw007.is_busy());
    let h = cmds::ReqResp::from_bytes(&buf);
    println!("resp: {}", h.is_response());
    println!("header: {:#x?} ready = {}", h, h.is_ready());
    println!("phase: {:?}", h.phase());

    loop {}

    rw007.send_cmd_request(16);

    let mut buf = [0u8; 16];
    rw007.blocking_read(&mut buf);
    let h = cmds::ReqResp::from_bytes(&buf);
    println!("header: {:#x?} ready = {} rep = {}", h, h.is_ready(), h.is_response());

    loop {
        Timer::after_millis(1000).await;
    }

    /*
    let d = cmds::DataPacket {
        cmd: DataType::Cmd as u32,
        len: 8,
    };
    buf[0..8].copy_from_slice(d.as_bytes());
    let pkt = cmds::CmdHeader { cmd: 0x00, len: 0 };
    buf[8..16].copy_from_slice(pkt.as_bytes());
    let init = cmds::Init {};
    buf[16..16].copy_from_slice(init.as_bytes());

    loop {
        println!("int {}", rw007.busy.is_high());
        let rdy = rw007.wait_response_rdy();

        println!("ready {}", rdy);

        Timer::after_millis(1000).await;
    }
    */

    use crate::cmds::Cmd as _;

    /*

    let header = cmds::Header::new(32 - 16); // 28 - 16 = 12
    buf[0..16].copy_from_slice(header.as_bytes());
    let d = cmds::DataPacket {
        cmd: DataType::Cmd as u32,
        len: 0,
    };
    buf[16..24].copy_from_slice(d.as_bytes());
    let pkt = cmds::CmdHeader { cmd: 0x00, len: 0 };
    buf[24..32].copy_from_slice(pkt.as_bytes());
    let init = cmds::Init {};
    buf[32..32].copy_from_slice(init.as_bytes());

    let d = cmds::DataPacket {
        cmd: DataType::Cmd as u32,
        len: 8,
    };
    buf[0..8].copy_from_slice(d.as_bytes());
    let pkt = cmds::CmdHeader { cmd: 0x00, len: 0 };
    buf[8..16].copy_from_slice(pkt.as_bytes());
    let init = cmds::Init {};
    buf[16..16].copy_from_slice(init.as_bytes());

    //    spi.blocking_write(cmd.as_bytes());

    println!("int {}", rw007.busy.is_high());

    println!("SN: {:?}", buf);
    while rw007.busy.is_high() {}
        */

    loop {
        rw007.blocking_write(&mut buf[..16]);

        println!("ready {}", rw007.busy.is_high());
        println!("write: {:02x?}", &buf[0..32]);
        rw007.blocking_read(&mut out);
        println!("read:  {:02x?}", &out[..32]);

        let h = cmds::ReqResp::from_bytes(&out);

        println!("header: {:#x?} ready = {}", h, h.is_ready());
        let payload = &out[16..16 + (h.len as usize)];

        println!("payload: {:?}", payload);
        Timer::after_millis(1000).await;
        //     let elapsed = riscv::register::mcycle::read64() - curr;
        //       curr = riscv::register::mcycle::read64();

        //        let now = Instant::now();
        //      println!("[{:6.3}] cycle: {}", (now.as_millis() as f32) / 1000.0, elapsed);
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
