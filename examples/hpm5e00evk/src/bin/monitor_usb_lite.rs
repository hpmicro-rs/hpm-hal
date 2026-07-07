#![no_main]
#![no_std]
#![feature(abi_riscv_interrupt)]
#![feature(impl_trait_in_assoc_type)]

use core::ptr::{addr_of, addr_of_mut};
use core::sync::atomic::{AtomicU32, Ordering};

use embassy_executor::Spawner;
use embassy_usb::Builder;
use embassy_usb::class::cdc_acm::{CdcAcmClass, Receiver, Sender, State};
use embassy_usb::driver::EndpointError;
use futures_util::future::join;
use hal::usb::{EndpointState, Instance, UsbDriver};
use hpm_hal as hal;
use hpm_hal::{bind_interrupts, peripherals};
use panic_halt as _;
use static_cell::StaticCell;

const MONITOR_START_CMD: u16 = 0x4850;
const MONITOR_MAGIC_CMD: u16 = 0x4d44;
const MONITOR_PROFILE_VERSION: u16 = 2;
const MONITOR_END_CMD: u16 = 0xa55a;
const MONITOR_CMD_HOST_MASK: u16 = 0x8000;
const MONITOR_RESULT_ACK_MASK: u16 = 0x8000;

const CMD_CONNECT: u16 = 0x0001;
const CMD_GETINFO: u16 = 0x0002;
const CMD_GETVALUE: u16 = 0x00a0;

const TYPE_UINT32: u8 = 0x05;

const RESULT_HEAD_ERR: u16 = 0x0002;
const RESULT_CRC_ERR: u16 = 0x0004;
const RESULT_PAYLOAD_ERR: u16 = 0x0005;
const RESULT_GET_ERR: u16 = 0x0006;
const RESULT_DATA_TYPE_ERR: u16 = 0x0009;
const RESULT_CMD_ERR: u16 = 0x000b;
const RESULT_SUCCESS: u16 = 0x00ff;

const PROFILE_MIN_SIZE: usize = 34;
const HEADER_CRC_OFFSET: usize = 14;
const CRC_OFFSET: usize = 10;
const INDEX_OFFSET: usize = 14;
const CMD_OFFSET: usize = 16;
const RESULT_OFFSET: usize = 18;
const TIMING_OFFSET: usize = 20;
const EXTEND2_OFFSET: usize = 24;
const LENGTH_OFFSET: usize = 28;
const PAYLOAD_OFFSET: usize = 32;

const MONITOR_INFO_SIZE: usize = 24;
const MAX_PACKET_SIZE: usize = 512;
const MAX_VALUE_COUNT: u16 = 32;
const USB_MAX_PACKET_SIZE: usize = 64;

#[unsafe(no_mangle)]
static mut MON_LOOP_COUNT: u32 = 0;
#[unsafe(no_mangle)]
static mut MON_LED_STATE: u32 = 0;
#[unsafe(no_mangle)]
static mut MON_LAST_ERROR: u32 = 0;
#[unsafe(no_mangle)]
static mut MON_RX_PACKETS: u32 = 0;

#[unsafe(no_mangle)]
static EXCEPTION_MCAUSE: AtomicU32 = AtomicU32::new(0xffff_ffff);
#[unsafe(no_mangle)]
static EXCEPTION_MEPC: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static EXCEPTION_MTVAL: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static EXCEPTION_TRAP_FRAME: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static EXCEPTION_MSTATUS: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static EXCEPTION_MIE: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static EXCEPTION_MIP: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static EXCEPTION_MDCAUSE: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static EXCEPTION_MXSTATUS: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static EXCEPTION_MCACHE_CTL: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static EXCEPTION_MMISC_CTL: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static EXCEPTION_PMPCFG0: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static EXCEPTION_PMPADDR0: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static EXCEPTION_PMPADDR1: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static EXCEPTION_PMACFG0: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static EXCEPTION_PMAADDR0: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static EXCEPTION_PMAADDR1: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static EXCEPTION_RA: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static EXCEPTION_T0: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static EXCEPTION_T1: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static EXCEPTION_T2: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static EXCEPTION_A0: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static EXCEPTION_A1: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static EXCEPTION_A2: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static EXCEPTION_A3: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static EXCEPTION_A4: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static EXCEPTION_A5: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static EXCEPTION_A6: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static EXCEPTION_A7: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static EXCEPTION_GP: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static EXCEPTION_TP: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static EXCEPTION_S0: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static EXCEPTION_S1: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static EXCEPTION_S2: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static EXCEPTION_S3: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static EXCEPTION_S4: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static EXCEPTION_S5: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static EXCEPTION_S6: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static EXCEPTION_S7: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static EXCEPTION_S8: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static EXCEPTION_S9: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static EXCEPTION_S10: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static EXCEPTION_S11: AtomicU32 = AtomicU32::new(0);

#[unsafe(no_mangle)]
static MON_UPTIME_US: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static MON_CSR_MCAUSE: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static MON_CSR_MEPC: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static MON_CSR_MTVAL: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static MON_CSR_MSTATUS: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static MON_CSR_MIE: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static MON_CSR_MIP: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static MON_CSR_MDCAUSE: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static MON_CSR_MXSTATUS: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static MON_CSR_MCACHE_CTL: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static MON_CSR_MMISC_CTL: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static MON_CSR_PMPCFG0: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static MON_CSR_PMPADDR0: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static MON_CSR_PMPADDR1: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static MON_CSR_PMACFG0: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static MON_CSR_PMAADDR0: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static MON_CSR_PMAADDR1: AtomicU32 = AtomicU32::new(0);

#[unsafe(no_mangle)]
static MON_USB_STAGE: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static MON_USB_USBSTS: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static MON_USB_USBINTR: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static MON_USB_PORTSC1: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static MON_USB_OTGSC: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static MON_USB_PHY_CTRL0: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static MON_USB_USBCMD: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static MON_USB_USBMODE: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static MON_USB_IRQ_COUNT: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static MON_USB_QTD_ADDR: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static MON_USB_QTD_DATA_PTR: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static MON_USB_QTD_LEN: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static MON_USB_QTD_SYS_ADDR: AtomicU32 = AtomicU32::new(0);

bind_interrupts!(struct Irqs {
    USB0 => hal::usb::InterruptHandler<peripherals::USB0>;
});

#[unsafe(link_section = ".noncacheable")]
static EP_STATE: EndpointState = EndpointState::new();

#[unsafe(link_section = ".noncacheable")]
static CONFIG_DESC: StaticCell<[u8; 256]> = StaticCell::new();
#[unsafe(link_section = ".noncacheable")]
static BOS_DESC: StaticCell<[u8; 256]> = StaticCell::new();
#[unsafe(link_section = ".noncacheable")]
static CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();
#[unsafe(link_section = ".noncacheable")]
static CDC_STATE: StaticCell<State> = StaticCell::new();
#[unsafe(link_section = ".noncacheable")]
static MONITOR_STATE: StaticCell<MonitorUsbLite> = StaticCell::new();
#[unsafe(link_section = ".noncacheable")]
static USB_READ_BUF: StaticCell<[u8; USB_MAX_PACKET_SIZE]> = StaticCell::new();

struct MonitorPacket {
    index: u16,
    cmd: u16,
    payload_len: usize,
}

#[repr(C)]
struct TrapFrame {
    ra: usize,
    t0: usize,
    t1: usize,
    t2: usize,
    t3: usize,
    t4: usize,
    t5: usize,
    t6: usize,
    a0: usize,
    a1: usize,
    a2: usize,
    a3: usize,
    a4: usize,
    a5: usize,
    a6: usize,
    a7: usize,
    gp: usize,
    tp: usize,
    s0: usize,
    s1: usize,
    s2: usize,
    s3: usize,
    s4: usize,
    s5: usize,
    s6: usize,
    s7: usize,
    s8: usize,
    s9: usize,
    s10: usize,
    s11: usize,
}

struct MonitorUsbLite {
    rx: [u8; MAX_PACKET_SIZE],
    rx_len: usize,
    tx: [u8; MAX_PACKET_SIZE],
}

impl MonitorUsbLite {
    const fn new() -> Self {
        Self {
            rx: [0; MAX_PACKET_SIZE],
            rx_len: 0,
            tx: [0; MAX_PACKET_SIZE],
        }
    }

    fn push_rx(&mut self, data: &[u8]) {
        for &byte in data {
            if self.rx_len == self.rx.len() {
                self.rx_len = 0;
                write_mon_u32(addr_of_mut!(MON_LAST_ERROR), RESULT_HEAD_ERR as u32);
            }

            self.rx[self.rx_len] = byte;
            self.rx_len += 1;
        }
    }

    fn process_next(&mut self, micros: u32) -> Option<usize> {
        let frame_len = self.next_frame_len()?;
        let frame = &self.rx[..frame_len];
        let parsed = parse_packet(frame);
        let tx_len = match parsed {
            Ok(packet) => self.handle_packet(packet, micros),
            Err(result) => {
                self.tx.fill(0);
                let index = read_u16(frame, INDEX_OFFSET).unwrap_or(0);
                let cmd =
                    read_u16(frame, CMD_OFFSET).unwrap_or(CMD_CONNECT | MONITOR_CMD_HOST_MASK) & !MONITOR_CMD_HOST_MASK;
                write_mon_u32(addr_of_mut!(MON_LAST_ERROR), result as u32);
                self.write_response(index, cmd, result, micros, 0)
            }
        };
        self.consume(frame_len);
        Some(tx_len)
    }

    fn next_frame_len(&mut self) -> Option<usize> {
        let start = self
            .rx
            .windows(4)
            .take(self.rx_len.saturating_sub(3))
            .position(|window| {
                u16::from_le_bytes([window[0], window[1]]) == MONITOR_START_CMD
                    && u16::from_le_bytes([window[2], window[3]]) == MONITOR_MAGIC_CMD
            })?;

        if start > 0 {
            self.consume(start);
        }

        if self.rx_len < PROFILE_MIN_SIZE {
            return None;
        }

        let length = read_u32(&self.rx, LENGTH_OFFSET)? as usize;
        let frame_len = PROFILE_MIN_SIZE.checked_add(length)?;
        if frame_len > MAX_PACKET_SIZE {
            self.consume(PROFILE_MIN_SIZE);
            write_mon_u32(addr_of_mut!(MON_LAST_ERROR), RESULT_PAYLOAD_ERR as u32);
            return None;
        }

        if self.rx_len >= frame_len {
            Some(frame_len)
        } else {
            None
        }
    }

    fn consume(&mut self, count: usize) {
        self.rx.copy_within(count..self.rx_len, 0);
        self.rx_len -= count;
    }

    fn handle_packet(&mut self, packet: MonitorPacket, micros: u32) -> usize {
        self.tx.fill(0);

        write_mon_u32(addr_of_mut!(MON_RX_PACKETS), read_mon_u32(addr_of!(MON_RX_PACKETS)) + 1);

        let led_state = read_mon_u32(addr_of!(MON_LED_STATE)) ^ 1;
        write_mon_u32(addr_of_mut!(MON_LED_STATE), led_state);

        match packet.cmd {
            CMD_CONNECT => self.write_response(packet.index, CMD_CONNECT, RESULT_SUCCESS, micros, 0),
            CMD_GETINFO => {
                let payload_len = self.write_monitor_info();
                self.write_response(packet.index, CMD_GETINFO, RESULT_SUCCESS, micros, payload_len)
            }
            CMD_GETVALUE => {
                let result = self.process_getvalue(packet.payload_len);
                self.write_response(packet.index, CMD_GETVALUE, result, micros, packet.payload_len)
            }
            _ => self.write_response(packet.index, packet.cmd, RESULT_CMD_ERR, micros, 0),
        }
    }

    fn write_response(&mut self, index: u16, cmd: u16, result: u16, micros: u32, payload_len: usize) -> usize {
        write_u16(&mut self.tx, 0, MONITOR_START_CMD);
        write_u16(&mut self.tx, 2, MONITOR_MAGIC_CMD);
        write_u16(&mut self.tx, 4, MONITOR_PROFILE_VERSION);
        write_u32(&mut self.tx, 6, 0);
        write_u16(&mut self.tx, INDEX_OFFSET, index);
        write_u16(&mut self.tx, CMD_OFFSET, cmd);
        write_u16(&mut self.tx, RESULT_OFFSET, result | MONITOR_RESULT_ACK_MASK);
        write_u32(&mut self.tx, TIMING_OFFSET, micros);
        write_u32(&mut self.tx, EXTEND2_OFFSET, 0);
        write_u32(&mut self.tx, LENGTH_OFFSET, payload_len as u32);

        let total_len = PROFILE_MIN_SIZE + payload_len;
        write_u16(&mut self.tx, total_len - 2, MONITOR_END_CMD);
        let crc = crc32(&self.tx[HEADER_CRC_OFFSET..total_len]);
        write_u32(&mut self.tx, CRC_OFFSET, crc);
        total_len
    }

    fn write_monitor_info(&mut self) -> usize {
        let payload = &mut self.tx[PAYLOAD_OFFSET..PAYLOAD_OFFSET + MONITOR_INFO_SIZE];
        payload.fill(0);
        payload[0..2].copy_from_slice(&0x1209u16.to_le_bytes());
        payload[2..4].copy_from_slice(&0x5e31u16.to_le_bytes());
        payload[4..12].copy_from_slice(&202607050002u64.to_le_bytes());
        payload[12..16].copy_from_slice(&(MAX_PACKET_SIZE as u32).to_le_bytes());
        payload[16..20].copy_from_slice(&0u32.to_le_bytes());
        payload[20..22].copy_from_slice(&MAX_VALUE_COUNT.to_le_bytes());
        payload[22..24].copy_from_slice(&0u16.to_le_bytes());
        MONITOR_INFO_SIZE
    }

    fn process_getvalue(&mut self, payload_len: usize) -> u16 {
        let payload = &self.rx[PAYLOAD_OFFSET..PAYLOAD_OFFSET + payload_len];
        if payload.len() < 20 {
            return RESULT_PAYLOAD_ERR;
        }
        self.tx[PAYLOAD_OFFSET..PAYLOAD_OFFSET + payload_len].copy_from_slice(payload);

        let pkt_count = match read_u16(payload, 0) {
            Some(v) => v as usize,
            None => return RESULT_PAYLOAD_ERR,
        };
        if pkt_count > MAX_VALUE_COUNT as usize {
            return RESULT_PAYLOAD_ERR;
        }

        let mut offset = 12;
        for _ in 0..pkt_count {
            if offset + 12 > payload.len() {
                return RESULT_PAYLOAD_ERR;
            }

            let ty = payload[offset];
            let addr = match read_u32(payload, offset + 4) {
                Some(v) => v,
                None => return RESULT_PAYLOAD_ERR,
            };
            if ty != TYPE_UINT32 {
                return RESULT_DATA_TYPE_ERR;
            }

            let value = match read_allowed_u32(addr) {
                Some(value) => value,
                None => return RESULT_GET_ERR,
            };
            self.tx[PAYLOAD_OFFSET + offset + 8..PAYLOAD_OFFSET + offset + 12].copy_from_slice(&value.to_le_bytes());
            offset += 12;
        }

        RESULT_SUCCESS
    }
}

fn init_usb0_pins() {
    use hal::pac::{IOC, iomux};

    IOC.pad(179)
        .func_ctl()
        .write(|w| w.set_alt_select(iomux::IOC_PF19_FUNC_CTL_USB0_PWR));
    IOC.pad(182)
        .func_ctl()
        .write(|w| w.set_alt_select(iomux::IOC_PF22_FUNC_CTL_USB0_ID));
    IOC.pad(183)
        .func_ctl()
        .write(|w| w.set_alt_select(iomux::IOC_PF23_FUNC_CTL_USB0_OC));
}

#[embassy_executor::main(entry = "hpm_hal::entry")]
async fn main(_spawner: Spawner) -> ! {
    write_mon_u32(addr_of_mut!(MON_LAST_ERROR), 0x200);

    let p = hal::init(Default::default());
    write_mon_u32(addr_of_mut!(MON_LAST_ERROR), 0x210);

    init_usb0_pins();

    let mut usb_config = hal::usb::Config::default();
    usb_config.use_internal_vbus = true;
    let usb_driver = hal::usb::UsbDriver::new(p.USB0, Irqs, usb_config, &EP_STATE);
    write_mon_u32(addr_of_mut!(MON_LAST_ERROR), 0x220);

    let mut config = embassy_usb::Config::new(0x1209, 0x5e31);
    config.manufacturer = Some("hpm-hal");
    config.product = Some("HPM5E Monitor");
    config.serial_number = Some("HPM5E31");
    config.device_class = 0xEF;
    config.device_sub_class = 0x02;
    config.device_protocol = 0x01;
    config.composite_with_iads = true;
    write_mon_u32(addr_of_mut!(MON_LAST_ERROR), 0x230);

    write_mon_u32(addr_of_mut!(MON_LAST_ERROR), 0x240);
    let mut builder = Builder::new(
        usb_driver,
        config,
        CONFIG_DESC.init([0; 256]),
        BOS_DESC.init([0; 256]),
        &mut [],
        CONTROL_BUF.init([0; 64]),
    );
    write_mon_u32(addr_of_mut!(MON_LAST_ERROR), 0x250);
    let class = CdcAcmClass::new(&mut builder, CDC_STATE.init(State::new()), USB_MAX_PACKET_SIZE as u16);
    write_mon_u32(addr_of_mut!(MON_LAST_ERROR), 0x260);
    let mut usb = builder.build();
    write_mon_u32(addr_of_mut!(MON_LAST_ERROR), 0x270);

    let usb_fut = async {
        write_mon_u32(addr_of_mut!(MON_LAST_ERROR), 0x280);
        usb.run().await;
    };
    let monitor_fut = async {
        write_mon_u32(addr_of_mut!(MON_LAST_ERROR), 0x290);
        let (mut sender, mut reader) = class.split();
        let monitor = MONITOR_STATE.init(MonitorUsbLite::new());
        let usb_read_buf = USB_READ_BUF.init([0; USB_MAX_PACKET_SIZE]);
        loop {
            write_mon_u32(addr_of_mut!(MON_LAST_ERROR), 0x2a0);
            sender.wait_connection().await;
            write_mon_u32(addr_of_mut!(MON_LAST_ERROR), 0x2b0);
            reader.wait_connection().await;
            write_mon_u32(addr_of_mut!(MON_LAST_ERROR), 0x2c0);
            let _ = run_monitor(&mut reader, &mut sender, monitor, usb_read_buf).await;
            write_mon_u32(addr_of_mut!(MON_LAST_ERROR), 0x2ff);
        }
    };

    join(usb_fut, monitor_fut).await;

    loop {
        embassy_time::Timer::after_millis(500).await;
    }
}

struct Disconnected {}

impl From<EndpointError> for Disconnected {
    fn from(val: EndpointError) -> Self {
        match val {
            EndpointError::BufferOverflow => panic!("USB endpoint buffer overflow"),
            EndpointError::Disabled => Disconnected {},
        }
    }
}

async fn run_monitor<'d, T: Instance + 'd>(
    reader: &mut Receiver<'d, UsbDriver<'d, T>>,
    sender: &mut Sender<'d, UsbDriver<'d, T>>,
    monitor: &mut MonitorUsbLite,
    buf: &mut [u8; USB_MAX_PACKET_SIZE],
) -> Result<(), Disconnected> {
    monitor.rx_len = 0;

    loop {
        let n = reader.read_packet(&mut buf[..]).await?;
        write_mon_u32(
            addr_of_mut!(MON_LOOP_COUNT),
            read_mon_u32(addr_of!(MON_LOOP_COUNT)).wrapping_add(1),
        );

        monitor.push_rx(&buf[..n]);
        let micros = embassy_time::Instant::now().as_micros() as u32;
        while let Some(tx_len) = monitor.process_next(micros) {
            for chunk in monitor.tx[..tx_len].chunks(USB_MAX_PACKET_SIZE) {
                sender.write_packet(chunk).await?;
            }
        }
    }
}

fn parse_packet(frame: &[u8]) -> Result<MonitorPacket, u16> {
    if frame.len() < PROFILE_MIN_SIZE {
        return Err(RESULT_HEAD_ERR);
    }
    if read_u16(frame, 0) != Some(MONITOR_START_CMD)
        || read_u16(frame, 2) != Some(MONITOR_MAGIC_CMD)
        || read_u16(frame, 4) != Some(MONITOR_PROFILE_VERSION)
    {
        return Err(RESULT_HEAD_ERR);
    }

    let payload_len = read_u32(frame, LENGTH_OFFSET).ok_or(RESULT_HEAD_ERR)? as usize;
    let total_len = PROFILE_MIN_SIZE + payload_len;
    if frame.len() < total_len || read_u16(frame, total_len - 2) != Some(MONITOR_END_CMD) {
        return Err(RESULT_HEAD_ERR);
    }

    let crc = read_u32(frame, CRC_OFFSET).ok_or(RESULT_HEAD_ERR)?;
    if crc != crc32(&frame[HEADER_CRC_OFFSET..total_len]) {
        return Err(RESULT_CRC_ERR);
    }

    let cmd = read_u16(frame, CMD_OFFSET).ok_or(RESULT_HEAD_ERR)?;
    if cmd & MONITOR_CMD_HOST_MASK == 0 {
        return Err(RESULT_CMD_ERR);
    }

    Ok(MonitorPacket {
        index: read_u16(frame, INDEX_OFFSET).ok_or(RESULT_HEAD_ERR)?,
        cmd: cmd & !MONITOR_CMD_HOST_MASK,
        payload_len,
    })
}

fn read_allowed_u32(addr: u32) -> Option<u32> {
    match addr {
        v if v == addr_of!(MON_LOOP_COUNT) as u32 => Some(read_mon_u32(addr_of!(MON_LOOP_COUNT))),
        v if v == addr_of!(MON_LED_STATE) as u32 => Some(read_mon_u32(addr_of!(MON_LED_STATE))),
        v if v == addr_of!(MON_LAST_ERROR) as u32 => Some(read_mon_u32(addr_of!(MON_LAST_ERROR))),
        v if v == addr_of!(MON_RX_PACKETS) as u32 => Some(read_mon_u32(addr_of!(MON_RX_PACKETS))),

        v if v == atomic_addr(&EXCEPTION_MCAUSE) => Some(EXCEPTION_MCAUSE.load(Ordering::Relaxed)),
        v if v == atomic_addr(&EXCEPTION_MEPC) => Some(EXCEPTION_MEPC.load(Ordering::Relaxed)),
        v if v == atomic_addr(&EXCEPTION_MTVAL) => Some(EXCEPTION_MTVAL.load(Ordering::Relaxed)),
        v if v == atomic_addr(&EXCEPTION_TRAP_FRAME) => Some(EXCEPTION_TRAP_FRAME.load(Ordering::Relaxed)),
        v if v == atomic_addr(&EXCEPTION_MSTATUS) => Some(EXCEPTION_MSTATUS.load(Ordering::Relaxed)),
        v if v == atomic_addr(&EXCEPTION_MIE) => Some(EXCEPTION_MIE.load(Ordering::Relaxed)),
        v if v == atomic_addr(&EXCEPTION_MIP) => Some(EXCEPTION_MIP.load(Ordering::Relaxed)),
        v if v == atomic_addr(&EXCEPTION_MDCAUSE) => Some(EXCEPTION_MDCAUSE.load(Ordering::Relaxed)),
        v if v == atomic_addr(&EXCEPTION_MXSTATUS) => Some(EXCEPTION_MXSTATUS.load(Ordering::Relaxed)),
        v if v == atomic_addr(&EXCEPTION_MCACHE_CTL) => Some(EXCEPTION_MCACHE_CTL.load(Ordering::Relaxed)),
        v if v == atomic_addr(&EXCEPTION_MMISC_CTL) => Some(EXCEPTION_MMISC_CTL.load(Ordering::Relaxed)),
        v if v == atomic_addr(&EXCEPTION_PMPCFG0) => Some(EXCEPTION_PMPCFG0.load(Ordering::Relaxed)),
        v if v == atomic_addr(&EXCEPTION_PMPADDR0) => Some(EXCEPTION_PMPADDR0.load(Ordering::Relaxed)),
        v if v == atomic_addr(&EXCEPTION_PMPADDR1) => Some(EXCEPTION_PMPADDR1.load(Ordering::Relaxed)),
        v if v == atomic_addr(&EXCEPTION_PMACFG0) => Some(EXCEPTION_PMACFG0.load(Ordering::Relaxed)),
        v if v == atomic_addr(&EXCEPTION_PMAADDR0) => Some(EXCEPTION_PMAADDR0.load(Ordering::Relaxed)),
        v if v == atomic_addr(&EXCEPTION_PMAADDR1) => Some(EXCEPTION_PMAADDR1.load(Ordering::Relaxed)),

        v if v == atomic_addr(&MON_UPTIME_US) => Some(embassy_time::Instant::now().as_micros() as u32),
        v if v == atomic_addr(&MON_CSR_MCAUSE) => Some(read_csr("mcause")),
        v if v == atomic_addr(&MON_CSR_MEPC) => Some(read_csr("mepc")),
        v if v == atomic_addr(&MON_CSR_MTVAL) => Some(read_csr("mtval")),
        v if v == atomic_addr(&MON_CSR_MSTATUS) => Some(read_csr("mstatus")),
        v if v == atomic_addr(&MON_CSR_MIE) => Some(read_csr("mie")),
        v if v == atomic_addr(&MON_CSR_MIP) => Some(read_csr("mip")),
        v if v == atomic_addr(&MON_CSR_MDCAUSE) => Some(read_csr("mdcause")),
        v if v == atomic_addr(&MON_CSR_MXSTATUS) => Some(read_csr("mxstatus")),
        v if v == atomic_addr(&MON_CSR_MCACHE_CTL) => Some(read_csr("mcache_ctl")),
        v if v == atomic_addr(&MON_CSR_MMISC_CTL) => Some(read_csr("mmisc_ctl")),
        v if v == atomic_addr(&MON_CSR_PMPCFG0) => Some(read_csr("pmpcfg0")),
        v if v == atomic_addr(&MON_CSR_PMPADDR0) => Some(read_csr("pmpaddr0")),
        v if v == atomic_addr(&MON_CSR_PMPADDR1) => Some(read_csr("pmpaddr1")),
        v if v == atomic_addr(&MON_CSR_PMACFG0) => Some(read_csr("pmacfg0")),
        v if v == atomic_addr(&MON_CSR_PMAADDR0) => Some(read_csr("pmaaddr0")),
        v if v == atomic_addr(&MON_CSR_PMAADDR1) => Some(read_csr("pmaaddr1")),

        v if v == atomic_addr(&MON_USB_STAGE) => Some(hal::usb::USB_DEBUG_STAGE.load(Ordering::Relaxed)),
        v if v == atomic_addr(&MON_USB_USBSTS) => Some(hal::usb::USB_DEBUG_USBSTS.load(Ordering::Relaxed)),
        v if v == atomic_addr(&MON_USB_USBINTR) => Some(hal::usb::USB_DEBUG_USBINTR.load(Ordering::Relaxed)),
        v if v == atomic_addr(&MON_USB_PORTSC1) => Some(hal::usb::USB_DEBUG_PORTSC1.load(Ordering::Relaxed)),
        v if v == atomic_addr(&MON_USB_OTGSC) => Some(hal::usb::USB_DEBUG_OTGSC.load(Ordering::Relaxed)),
        v if v == atomic_addr(&MON_USB_PHY_CTRL0) => Some(hal::usb::USB_DEBUG_PHY_CTRL0.load(Ordering::Relaxed)),
        v if v == atomic_addr(&MON_USB_USBCMD) => Some(hal::usb::USB_DEBUG_USBCMD.load(Ordering::Relaxed)),
        v if v == atomic_addr(&MON_USB_USBMODE) => Some(hal::usb::USB_DEBUG_USBMODE.load(Ordering::Relaxed)),
        v if v == atomic_addr(&MON_USB_IRQ_COUNT) => Some(hal::usb::USB_DEBUG_IRQ_COUNT.load(Ordering::Relaxed)),
        v if v == atomic_addr(&MON_USB_QTD_ADDR) => Some(hal::usb::USB_DEBUG_QTD_ADDR.load(Ordering::Relaxed)),
        v if v == atomic_addr(&MON_USB_QTD_DATA_PTR) => Some(hal::usb::USB_DEBUG_QTD_DATA_PTR.load(Ordering::Relaxed)),
        v if v == atomic_addr(&MON_USB_QTD_LEN) => Some(hal::usb::USB_DEBUG_QTD_LEN.load(Ordering::Relaxed)),
        v if v == atomic_addr(&MON_USB_QTD_SYS_ADDR) => Some(hal::usb::USB_DEBUG_QTD_SYS_ADDR.load(Ordering::Relaxed)),
        _ => None,
    }
}

fn atomic_addr(value: &'static AtomicU32) -> u32 {
    value as *const AtomicU32 as u32
}

fn read_mon_u32(ptr: *const u32) -> u32 {
    unsafe { core::ptr::read_volatile(ptr) }
}

fn write_mon_u32(ptr: *mut u32, value: u32) {
    unsafe { core::ptr::write_volatile(ptr, value) }
}

fn read_u16(buf: &[u8], offset: usize) -> Option<u16> {
    let bytes = buf.get(offset..offset + 2)?;
    Some(u16::from_le_bytes([bytes[0], bytes[1]]))
}

fn read_u32(buf: &[u8], offset: usize) -> Option<u32> {
    let bytes = buf.get(offset..offset + 4)?;
    Some(u32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]))
}

fn write_u16(buf: &mut [u8], offset: usize, value: u16) {
    buf[offset..offset + 2].copy_from_slice(&value.to_le_bytes());
}

fn write_u32(buf: &mut [u8], offset: usize, value: u32) {
    buf[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
}

fn crc32(data: &[u8]) -> u32 {
    const POLYNOMIAL: u32 = 0xbeaf5aa5;
    let mut crc = 0xffff_ffffu32;
    for byte in data {
        crc ^= *byte as u32;
        for _ in 0..8 {
            let mask = 0u32.wrapping_sub(crc & 1);
            crc = (crc >> 1) ^ (POLYNOMIAL & mask);
        }
    }
    crc
}

macro_rules! exception_handler {
    ($($name:ident),* $(,)?) => {
        $(
            #[unsafe(no_mangle)]
            #[allow(non_snake_case)]
            unsafe extern "C" fn $name(trap_frame: *const TrapFrame) -> ! {
                record_exception(trap_frame);
            }
        )*
    };
}

exception_handler!(
    InstructionMisaligned,
    InstructionFault,
    IllegalInstruction,
    Breakpoint,
    LoadMisaligned,
    LoadFault,
    StoreMisaligned,
    StoreFault,
    UserEnvCall,
    SupervisorEnvCall,
    MachineEnvCall,
    InstructionPageFault,
    LoadPageFault,
    StorePageFault,
    ExceptionHandler,
);

fn record_exception(trap_frame: *const TrapFrame) -> ! {
    let mcause = read_csr("mcause");
    let mepc = read_csr("mepc");
    let mtval = read_csr("mtval");
    let mstatus = read_csr("mstatus");
    let mie = read_csr("mie");
    let mip = read_csr("mip");
    let mdcause = read_csr("mdcause");
    let mxstatus = read_csr("mxstatus");
    let mcache_ctl = read_csr("mcache_ctl");
    let mmisc_ctl = read_csr("mmisc_ctl");
    let pmpcfg0 = read_csr("pmpcfg0");
    let pmpaddr0 = read_csr("pmpaddr0");
    let pmpaddr1 = read_csr("pmpaddr1");
    let pmacfg0 = read_csr("pmacfg0");
    let pmaaddr0 = read_csr("pmaaddr0");
    let pmaaddr1 = read_csr("pmaaddr1");

    EXCEPTION_MCAUSE.store(mcause, Ordering::Relaxed);
    EXCEPTION_MEPC.store(mepc, Ordering::Relaxed);
    EXCEPTION_MTVAL.store(mtval, Ordering::Relaxed);
    EXCEPTION_TRAP_FRAME.store(trap_frame as u32, Ordering::Relaxed);
    EXCEPTION_MSTATUS.store(mstatus, Ordering::Relaxed);
    EXCEPTION_MIE.store(mie, Ordering::Relaxed);
    EXCEPTION_MIP.store(mip, Ordering::Relaxed);
    EXCEPTION_MDCAUSE.store(mdcause, Ordering::Relaxed);
    EXCEPTION_MXSTATUS.store(mxstatus, Ordering::Relaxed);
    EXCEPTION_MCACHE_CTL.store(mcache_ctl, Ordering::Relaxed);
    EXCEPTION_MMISC_CTL.store(mmisc_ctl, Ordering::Relaxed);
    EXCEPTION_PMPCFG0.store(pmpcfg0, Ordering::Relaxed);
    EXCEPTION_PMPADDR0.store(pmpaddr0, Ordering::Relaxed);
    EXCEPTION_PMPADDR1.store(pmpaddr1, Ordering::Relaxed);
    EXCEPTION_PMACFG0.store(pmacfg0, Ordering::Relaxed);
    EXCEPTION_PMAADDR0.store(pmaaddr0, Ordering::Relaxed);
    EXCEPTION_PMAADDR1.store(pmaaddr1, Ordering::Relaxed);

    if let Some(trap_frame) = unsafe { trap_frame.as_ref() } {
        EXCEPTION_RA.store(trap_frame.ra as u32, Ordering::Relaxed);
        EXCEPTION_T0.store(trap_frame.t0 as u32, Ordering::Relaxed);
        EXCEPTION_T1.store(trap_frame.t1 as u32, Ordering::Relaxed);
        EXCEPTION_T2.store(trap_frame.t2 as u32, Ordering::Relaxed);
        EXCEPTION_A0.store(trap_frame.a0 as u32, Ordering::Relaxed);
        EXCEPTION_A1.store(trap_frame.a1 as u32, Ordering::Relaxed);
        EXCEPTION_A2.store(trap_frame.a2 as u32, Ordering::Relaxed);
        EXCEPTION_A3.store(trap_frame.a3 as u32, Ordering::Relaxed);
        EXCEPTION_A4.store(trap_frame.a4 as u32, Ordering::Relaxed);
        EXCEPTION_A5.store(trap_frame.a5 as u32, Ordering::Relaxed);
        EXCEPTION_A6.store(trap_frame.a6 as u32, Ordering::Relaxed);
        EXCEPTION_A7.store(trap_frame.a7 as u32, Ordering::Relaxed);
        EXCEPTION_GP.store(trap_frame.gp as u32, Ordering::Relaxed);
        EXCEPTION_TP.store(trap_frame.tp as u32, Ordering::Relaxed);
        EXCEPTION_S0.store(trap_frame.s0 as u32, Ordering::Relaxed);
        EXCEPTION_S1.store(trap_frame.s1 as u32, Ordering::Relaxed);
        EXCEPTION_S2.store(trap_frame.s2 as u32, Ordering::Relaxed);
        EXCEPTION_S3.store(trap_frame.s3 as u32, Ordering::Relaxed);
        EXCEPTION_S4.store(trap_frame.s4 as u32, Ordering::Relaxed);
        EXCEPTION_S5.store(trap_frame.s5 as u32, Ordering::Relaxed);
        EXCEPTION_S6.store(trap_frame.s6 as u32, Ordering::Relaxed);
        EXCEPTION_S7.store(trap_frame.s7 as u32, Ordering::Relaxed);
        EXCEPTION_S8.store(trap_frame.s8 as u32, Ordering::Relaxed);
        EXCEPTION_S9.store(trap_frame.s9 as u32, Ordering::Relaxed);
        EXCEPTION_S10.store(trap_frame.s10 as u32, Ordering::Relaxed);
        EXCEPTION_S11.store(trap_frame.s11 as u32, Ordering::Relaxed);
    }

    loop {
        core::hint::spin_loop();
    }
}

#[inline(always)]
fn read_csr(csr: &str) -> u32 {
    let value: usize;
    unsafe {
        match csr {
            "mcause" => core::arch::asm!("csrr {0}, mcause", out(reg) value),
            "mepc" => core::arch::asm!("csrr {0}, mepc", out(reg) value),
            "mtval" => core::arch::asm!("csrr {0}, mtval", out(reg) value),
            "mstatus" => core::arch::asm!("csrr {0}, mstatus", out(reg) value),
            "mie" => core::arch::asm!("csrr {0}, mie", out(reg) value),
            "mip" => core::arch::asm!("csrr {0}, mip", out(reg) value),
            "mdcause" => core::arch::asm!("csrr {0}, 0x7C9", out(reg) value),
            "mxstatus" => core::arch::asm!("csrr {0}, 0x7C4", out(reg) value),
            "mcache_ctl" => core::arch::asm!("csrr {0}, 0x7CA", out(reg) value),
            "mmisc_ctl" => core::arch::asm!("csrr {0}, 0x7D0", out(reg) value),
            "pmpcfg0" => core::arch::asm!("csrr {0}, pmpcfg0", out(reg) value),
            "pmpaddr0" => core::arch::asm!("csrr {0}, pmpaddr0", out(reg) value),
            "pmpaddr1" => core::arch::asm!("csrr {0}, pmpaddr1", out(reg) value),
            "pmacfg0" => core::arch::asm!("csrr {0}, 0xBC0", out(reg) value),
            "pmaaddr0" => core::arch::asm!("csrr {0}, 0xBD0", out(reg) value),
            "pmaaddr1" => core::arch::asm!("csrr {0}, 0xBD1", out(reg) value),
            _ => core::hint::unreachable_unchecked(),
        }
    }
    value as u32
}
