#![no_main]
#![no_std]
#![feature(abi_riscv_interrupt)]
#![feature(impl_trait_in_assoc_type)]

use core::cell::RefCell;
use core::panic::PanicInfo;
use core::ptr::{addr_of, addr_of_mut};
use core::sync::atomic::{AtomicU32, Ordering};

use bosch_bme680::{Bme680, BmeError, Configuration, DeviceAddress};
use defmt_rtt as _;
use embassy_embedded_hal::shared_bus::blocking::i2c::I2cDevice;
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::{Delay, Duration, Timer};
use embassy_usb::Builder;
use embassy_usb::driver::{Driver, Endpoint, EndpointError, EndpointIn, EndpointOut};
use embassy_usb::msos::{CompatibleIdFeatureDescriptor, windows_version};
use embedded_hal::i2c::I2c as _;
use embedded_hal_nb::serial::Read as _;
use futures_util::future::join;
use hal::usb::EndpointState;
use hpm_hal as hal;
use hpm_hal::i2c::{I2c, I2cMode};
use hpm_hal::mode::Blocking;
use hpm_hal::uart::UartRx;
use hpm_hal::{bind_interrupts, peripherals};
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
const BULK_MAX_PACKET_SIZE: usize = 64;

const BH1750_ADDR: u8 = 0x23;
const APDS9930_ADDR: u8 = 0x39;
const BH1750_POWER_ON: u8 = 0x01;
const BH1750_RESET: u8 = 0x07;
const BH1750_CONTINUOUS_HIGH_RESOLUTION: u8 = 0x10;
const APDS9930_CMD_BYTE: u8 = 0x80;
const APDS9930_CMD_AUTO_INCREMENT: u8 = 0xa0;
const APDS9930_ENABLE: u8 = 0x00;
const APDS9930_ATIME: u8 = 0x01;
const APDS9930_PTIME: u8 = 0x02;
const APDS9930_WTIME: u8 = 0x03;
const APDS9930_PPULSE: u8 = 0x0e;
const APDS9930_CONTROL: u8 = 0x0f;
const APDS9930_ID: u8 = 0x12;
const APDS9930_STATUS: u8 = 0x13;
const APDS9930_CH0DATAL: u8 = 0x14;
const PMS7003_PREFIX: [u8; 4] = [0x42, 0x4d, 0x00, 0x1c];

#[unsafe(no_mangle)]
static mut MON_LOOP_COUNT: u32 = 0;
#[unsafe(no_mangle)]
static mut MON_LED_STATE: u32 = 0;
#[unsafe(no_mangle)]
static mut MON_LAST_ERROR: u32 = 0;
#[unsafe(no_mangle)]
static mut MON_RX_PACKETS: u32 = 0;
#[unsafe(no_mangle)]
static MON_PANIC_LINE: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static MON_PANIC_COLUMN: AtomicU32 = AtomicU32::new(0);

#[unsafe(no_mangle)]
static MON_SENSOR_CYCLE_COUNT: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static MON_SENSOR_READY_MASK: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static MON_SENSOR_LAST_ERROR: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static MON_BH1750_SAMPLE_COUNT: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static MON_BH1750_RAW: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static MON_BH1750_LUX_X100: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static MON_APDS9930_SAMPLE_COUNT: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static MON_APDS9930_ID: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static MON_APDS9930_STATUS: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static MON_APDS9930_CLEAR_RAW: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static MON_APDS9930_IR_RAW: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static MON_APDS9930_PROXIMITY_RAW: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static MON_BME680_SAMPLE_COUNT: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static MON_BME680_TEMP_C_X100: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static MON_BME680_HUMIDITY_RH_X100: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static MON_BME680_PRESSURE_PA: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static MON_BME680_GAS_OHM: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static MON_BME680_GAS_VALID: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static MON_PMS7003_STATUS: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static MON_PMS7003_VALID_FRAMES: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static MON_PMS7003_INVALID_FRAMES: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static MON_PMS7003_UART_ERRORS: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static MON_PMS7003_PM1_0_UG_M3: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static MON_PMS7003_PM2_5_UG_M3: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static MON_PMS7003_PM10_UG_M3: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static MON_PMS7003_VERSION: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static MON_PMS7003_SENSOR_ERROR: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static MON_PMS7003_LAST_UPDATE_MS: AtomicU32 = AtomicU32::new(0);

#[unsafe(no_mangle)]
static EXCEPTION_MCAUSE: AtomicU32 = AtomicU32::new(0);
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

#[repr(align(4))]
struct AlignedBuf<const N: usize>([u8; N]);

#[unsafe(link_section = ".noncacheable")]
static EP_STATE: EndpointState = EndpointState::new();

#[unsafe(link_section = ".noncacheable")]
static CONFIG_DESC: StaticCell<AlignedBuf<512>> = StaticCell::new();
#[unsafe(link_section = ".noncacheable")]
static BOS_DESC: StaticCell<AlignedBuf<512>> = StaticCell::new();
#[unsafe(link_section = ".noncacheable")]
static MSOS_DESC: StaticCell<AlignedBuf<256>> = StaticCell::new();
#[unsafe(link_section = ".noncacheable")]
static CONTROL_BUF: StaticCell<AlignedBuf<64>> = StaticCell::new();
static MONITOR_BULK_STATE: StaticCell<MonitorUsbLite> = StaticCell::new();
#[unsafe(link_section = ".noncacheable")]
static BULK_READ_BUF: StaticCell<AlignedBuf<BULK_MAX_PACKET_SIZE>> = StaticCell::new();
#[unsafe(link_section = ".noncacheable")]
static BULK_WRITE_BUF: StaticCell<AlignedBuf<BULK_MAX_PACKET_SIZE>> = StaticCell::new();

struct MonitorPacket {
    index: u16,
    cmd: u16,
    payload_len: usize,
}

struct MonitorBulkClass<'d, D: Driver<'d>> {
    read_ep: D::EndpointOut,
    write_ep: D::EndpointIn,
}

impl<'d, D: Driver<'d>> MonitorBulkClass<'d, D> {
    fn new(builder: &mut Builder<'d, D>, max_packet_size: u16) -> Self {
        write_mon_u32(addr_of_mut!(MON_LAST_ERROR), 0x252);
        let mut function = builder.function(0xff, 0x00, 0x00);
        write_mon_u32(addr_of_mut!(MON_LAST_ERROR), 0x253);
        function.msos_feature(CompatibleIdFeatureDescriptor::new("WINUSB", ""));
        write_mon_u32(addr_of_mut!(MON_LAST_ERROR), 0x254);

        let mut interface = function.interface();
        write_mon_u32(addr_of_mut!(MON_LAST_ERROR), 0x255);
        let mut alt = interface.alt_setting(0xff, 0x00, 0x00, None);
        write_mon_u32(addr_of_mut!(MON_LAST_ERROR), 0x256);
        let read_ep = alt.endpoint_bulk_out(None, max_packet_size);
        write_mon_u32(addr_of_mut!(MON_LAST_ERROR), 0x257);
        let write_ep = alt.endpoint_bulk_in(None, max_packet_size);
        write_mon_u32(addr_of_mut!(MON_LAST_ERROR), 0x258);

        Self { read_ep, write_ep }
    }
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
        let remaining = self.rx_len - count;
        for i in 0..remaining {
            unsafe {
                let value = core::ptr::read_volatile(self.rx.as_ptr().add(count + i));
                core::ptr::write_volatile(self.rx.as_mut_ptr().add(i), value);
            }
        }
        self.rx_len -= count;
    }

    fn handle_packet(&mut self, packet: MonitorPacket, micros: u32) -> usize {
        write_mon_u32(addr_of_mut!(MON_LAST_ERROR), 0x300 | packet.cmd as u32);
        fill_bytes(&mut self.tx, 0);

        write_mon_u32(addr_of_mut!(MON_RX_PACKETS), read_mon_u32(addr_of!(MON_RX_PACKETS)) + 1);

        let led_state = read_mon_u32(addr_of!(MON_LED_STATE)) ^ 1;
        write_mon_u32(addr_of_mut!(MON_LED_STATE), led_state);

        if packet.payload_len > MAX_PACKET_SIZE - PROFILE_MIN_SIZE {
            return self.write_response(packet.index, packet.cmd, RESULT_PAYLOAD_ERR, micros, 0);
        }

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
        write_mon_u32(addr_of_mut!(MON_LAST_ERROR), 0x310);
        write_u16(&mut self.tx, PAYLOAD_OFFSET, 0x1209);
        write_u16(&mut self.tx, PAYLOAD_OFFSET + 2, 0x5e31);
        write_u64(&mut self.tx, PAYLOAD_OFFSET + 4, 202607050002);
        write_u32(&mut self.tx, PAYLOAD_OFFSET + 12, MAX_PACKET_SIZE as u32);
        write_u32(&mut self.tx, PAYLOAD_OFFSET + 16, 0);
        write_u16(&mut self.tx, PAYLOAD_OFFSET + 20, MAX_VALUE_COUNT);
        write_u16(&mut self.tx, PAYLOAD_OFFSET + 22, 0);
        MONITOR_INFO_SIZE
    }

    fn process_getvalue(&mut self, payload_len: usize) -> u16 {
        if payload_len > MAX_PACKET_SIZE - PROFILE_MIN_SIZE {
            return RESULT_PAYLOAD_ERR;
        }
        let payload = &self.rx[PAYLOAD_OFFSET..PAYLOAD_OFFSET + payload_len];
        if payload.len() < 20 {
            return RESULT_PAYLOAD_ERR;
        }
        copy_bytes(&mut self.tx[PAYLOAD_OFFSET..PAYLOAD_OFFSET + payload_len], payload);

        let pkt_count = match read_u16(payload, 0) {
            Some(v) => v as usize,
            None => return RESULT_PAYLOAD_ERR,
        };
        if pkt_count > MAX_VALUE_COUNT as usize {
            return RESULT_PAYLOAD_ERR;
        }
        if 12usize
            .checked_add(pkt_count.saturating_mul(12))
            .filter(|expected| *expected == payload_len)
            .is_none()
        {
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
            write_u32(&mut self.tx, PAYLOAD_OFFSET + offset + 8, value);
            offset += 12;
        }

        RESULT_SUCCESS
    }
}

fn init_usb0_pins() {
    const IOC_BASE: usize = 0xf404_0000;
    const FUNC_CTL_STRIDE: usize = 8;
    const USB0_ALT: u32 = 0x18;

    fn set_pad_alt(pad: usize, alt: u32) {
        let func_ctl = (IOC_BASE + pad * FUNC_CTL_STRIDE) as *mut u32;
        unsafe {
            let value = core::ptr::read_volatile(func_ctl);
            core::ptr::write_volatile(func_ctl, (value & !0x1f) | (alt & 0x1f));
        }
    }

    write_mon_u32(addr_of_mut!(MON_LAST_ERROR), 0x211);
    set_pad_alt(179, USB0_ALT);
    write_mon_u32(addr_of_mut!(MON_LAST_ERROR), 0x212);
    set_pad_alt(182, USB0_ALT);
    write_mon_u32(addr_of_mut!(MON_LAST_ERROR), 0x213);
    set_pad_alt(183, USB0_ALT);
    write_mon_u32(addr_of_mut!(MON_LAST_ERROR), 0x214);
}

#[embassy_executor::main(entry = "hpm_hal::entry")]
async fn main(spawner: Spawner) -> ! {
    write_mon_u32(addr_of_mut!(MON_LAST_ERROR), 0x200);
    defmt::info!("hpm usb bulk sensor monitor start");

    let p = hal::init(Default::default());
    write_mon_u32(addr_of_mut!(MON_LAST_ERROR), 0x210);
    defmt::info!("hal init complete");

    let mut i2c_config = hal::i2c::Config::default();
    i2c_config.mode = I2cMode::Standard;
    i2c_config.timeout = Duration::from_millis(100);
    let i2c = I2c::new_blocking(p.I2C0, p.PC08, p.PC09, i2c_config);
    match i2c_sensors_task(i2c) {
        Ok(task) => spawner.spawn(task),
        Err(_) => MON_SENSOR_LAST_ERROR.store(0x0000_00f1, Ordering::Relaxed),
    }

    let mut uart_config = hal::uart::Config::default();
    uart_config.baudrate = 9_600;
    uart_config.detect_previous_overrun = false;
    match UartRx::new_blocking(p.UART3, p.PC14, uart_config) {
        Ok(rx) => match pms7003_task(rx) {
            Ok(task) => spawner.spawn(task),
            Err(_) => MON_PMS7003_STATUS.store(0x0000_00f1, Ordering::Relaxed),
        },
        Err(_) => MON_PMS7003_STATUS.store(0x0000_00f2, Ordering::Relaxed),
    }

    init_usb0_pins();

    let mut usb_config = hal::usb::Config::default();
    usb_config.use_internal_vbus = true;
    usb_config.force_full_speed = true;
    let usb_driver = hal::usb::UsbDriver::new(p.USB0, Irqs, usb_config, &EP_STATE);
    write_mon_u32(addr_of_mut!(MON_LAST_ERROR), 0x220);

    let mut config = embassy_usb::Config::new(0x1209, 0x5e31);
    config.manufacturer = Some("hpm-hal");
    config.product = Some("HPM5E Sensor Monitor");
    config.serial_number = Some("HPM5E31");
    config.device_class = 0x00;
    config.device_sub_class = 0x00;
    config.device_protocol = 0x00;
    config.composite_with_iads = false;
    write_mon_u32(addr_of_mut!(MON_LAST_ERROR), 0x230);

    write_mon_u32(addr_of_mut!(MON_LAST_ERROR), 0x240);
    let mut builder = Builder::new(
        usb_driver,
        config,
        &mut CONFIG_DESC.init(AlignedBuf([0; 512])).0,
        &mut BOS_DESC.init(AlignedBuf([0; 512])).0,
        &mut MSOS_DESC.init(AlignedBuf([0; 256])).0,
        &mut CONTROL_BUF.init(AlignedBuf([0; 64])).0,
    );
    write_mon_u32(addr_of_mut!(MON_LAST_ERROR), 0x250);
    builder.msos_descriptor(windows_version::WIN10, 0x21);
    write_mon_u32(addr_of_mut!(MON_LAST_ERROR), 0x251);
    let bulk_class = MonitorBulkClass::new(&mut builder, BULK_MAX_PACKET_SIZE as u16);
    write_mon_u32(addr_of_mut!(MON_LAST_ERROR), 0x260);
    let mut usb = builder.build();
    write_mon_u32(addr_of_mut!(MON_LAST_ERROR), 0x270);

    let usb_fut = async {
        write_mon_u32(addr_of_mut!(MON_LAST_ERROR), 0x280);
        defmt::info!("usb device run");
        usb.run().await;
    };
    let bulk_monitor_fut = async {
        write_mon_u32(addr_of_mut!(MON_LAST_ERROR), 0x290);
        let mut bulk_class = bulk_class;
        let monitor = MONITOR_BULK_STATE.init(MonitorUsbLite::new());
        let bulk_read_buf = &mut BULK_READ_BUF.init(AlignedBuf([0; BULK_MAX_PACKET_SIZE])).0;
        let bulk_write_buf = &mut BULK_WRITE_BUF.init(AlignedBuf([0; BULK_MAX_PACKET_SIZE])).0;
        loop {
            write_mon_u32(addr_of_mut!(MON_LAST_ERROR), 0x2a0);
            let _ = run_bulk_monitor(&mut bulk_class, monitor, bulk_read_buf, bulk_write_buf).await;
            write_mon_u32(addr_of_mut!(MON_LAST_ERROR), 0x2ff);
        }
    };
    join(usb_fut, bulk_monitor_fut).await;

    loop {
        embassy_time::Timer::after_millis(500).await;
    }
}

#[embassy_executor::task]
async fn i2c_sensors_task(i2c: I2c<'static, Blocking>) {
    let bus = Mutex::<CriticalSectionRawMutex, _>::new(RefCell::new(i2c));
    let mut bh1750 = I2cDevice::new(&bus);
    let mut apds9930 = I2cDevice::new(&bus);
    let bme680_i2c = I2cDevice::new(&bus);

    let bh1750_ready = bh1750.write(BH1750_ADDR, &[BH1750_POWER_ON]).is_ok()
        && bh1750.write(BH1750_ADDR, &[BH1750_RESET]).is_ok()
        && bh1750.write(BH1750_ADDR, &[BH1750_CONTINUOUS_HIGH_RESOLUTION]).is_ok();
    if bh1750_ready {
        MON_SENSOR_READY_MASK.fetch_or(1 << 0, Ordering::Relaxed);
    } else {
        MON_SENSOR_LAST_ERROR.store(0x0000_0101, Ordering::Relaxed);
    }

    let apds9930_ready = init_apds9930(&mut apds9930);
    if apds9930_ready {
        let mut id = [0u8; 1];
        if apds9930
            .write_read(APDS9930_ADDR, &[APDS9930_CMD_BYTE | APDS9930_ID], &mut id)
            .is_ok()
        {
            MON_APDS9930_ID.store(id[0] as u32, Ordering::Relaxed);
            MON_SENSOR_READY_MASK.fetch_or(1 << 1, Ordering::Relaxed);
        } else {
            MON_SENSOR_LAST_ERROR.store(0x0000_0202, Ordering::Relaxed);
        }
    } else {
        MON_SENSOR_LAST_ERROR.store(0x0000_0201, Ordering::Relaxed);
    }

    let bme_config = Configuration::default();
    let mut bme680 = match Bme680::new(bme680_i2c, DeviceAddress::Secondary, Delay, &bme_config, 20) {
        Ok(sensor) => {
            MON_SENSOR_READY_MASK.fetch_or(1 << 2, Ordering::Relaxed);
            Some(sensor)
        }
        Err(error) => {
            MON_SENSOR_LAST_ERROR.store(0x0003_0000 | bme_error_code(error), Ordering::Relaxed);
            None
        }
    };

    defmt::info!(
        "sensor monitor I2C ready mask=0x{:x}",
        MON_SENSOR_READY_MASK.load(Ordering::Relaxed)
    );
    Timer::after_millis(180).await;

    loop {
        MON_SENSOR_CYCLE_COUNT.fetch_add(1, Ordering::Relaxed);

        if bh1750_ready {
            let mut data = [0u8; 2];
            match bh1750.read(BH1750_ADDR, &mut data) {
                Ok(()) => {
                    let raw = u16::from_be_bytes(data) as u32;
                    MON_BH1750_RAW.store(raw, Ordering::Relaxed);
                    MON_BH1750_LUX_X100.store(raw * 250 / 3, Ordering::Relaxed);
                    MON_BH1750_SAMPLE_COUNT.fetch_add(1, Ordering::Relaxed);
                }
                Err(_) => MON_SENSOR_LAST_ERROR.store(0x0000_0102, Ordering::Relaxed),
            }
        }

        if apds9930_ready {
            let mut status = [0u8; 1];
            let mut data = [0u8; 6];
            if apds9930
                .write_read(APDS9930_ADDR, &[APDS9930_CMD_BYTE | APDS9930_STATUS], &mut status)
                .is_ok()
                && apds9930
                    .write_read(
                        APDS9930_ADDR,
                        &[APDS9930_CMD_AUTO_INCREMENT | APDS9930_CH0DATAL],
                        &mut data,
                    )
                    .is_ok()
            {
                MON_APDS9930_STATUS.store(status[0] as u32, Ordering::Relaxed);
                MON_APDS9930_CLEAR_RAW.store(u16::from_le_bytes([data[0], data[1]]) as u32, Ordering::Relaxed);
                MON_APDS9930_IR_RAW.store(u16::from_le_bytes([data[2], data[3]]) as u32, Ordering::Relaxed);
                MON_APDS9930_PROXIMITY_RAW.store(u16::from_le_bytes([data[4], data[5]]) as u32, Ordering::Relaxed);
                MON_APDS9930_SAMPLE_COUNT.fetch_add(1, Ordering::Relaxed);
            } else {
                MON_SENSOR_LAST_ERROR.store(0x0000_0203, Ordering::Relaxed);
            }
        }

        if let Some(sensor) = bme680.as_mut() {
            match sensor.measure() {
                Ok(data) => {
                    MON_BME680_TEMP_C_X100.store(scale_f32_to_u32(data.temperature, 100.0), Ordering::Relaxed);
                    MON_BME680_HUMIDITY_RH_X100.store(scale_f32_to_u32(data.humidity, 100.0), Ordering::Relaxed);
                    MON_BME680_PRESSURE_PA.store(scale_f32_to_u32(data.pressure, 1.0), Ordering::Relaxed);
                    if let Some(gas) = data.gas_resistance {
                        MON_BME680_GAS_OHM.store(scale_f32_to_u32(gas, 1.0), Ordering::Relaxed);
                        MON_BME680_GAS_VALID.store(1, Ordering::Relaxed);
                    } else {
                        MON_BME680_GAS_VALID.store(0, Ordering::Relaxed);
                    }
                    MON_BME680_SAMPLE_COUNT.fetch_add(1, Ordering::Relaxed);
                }
                Err(error) => MON_SENSOR_LAST_ERROR.store(0x0003_1000 | bme_error_code(error), Ordering::Relaxed),
            }
        }

        Timer::after_secs(1).await;
    }
}

fn init_apds9930<I: embedded_hal::i2c::I2c>(i2c: &mut I) -> bool {
    let setup = [
        (APDS9930_ENABLE, 0x00),
        (APDS9930_ATIME, 0xdb),
        (APDS9930_PTIME, 0xff),
        (APDS9930_WTIME, 0xff),
        (APDS9930_PPULSE, 8),
        (APDS9930_CONTROL, 0x20),
        (APDS9930_ENABLE, 0x0f),
    ];

    setup
        .into_iter()
        .all(|(register, value)| i2c.write(APDS9930_ADDR, &[APDS9930_CMD_BYTE | register, value]).is_ok())
}

fn bme_error_code<I>(error: BmeError<I>) -> u32
where
    I: embedded_hal::i2c::ErrorType,
{
    match error {
        BmeError::WriteError(_) => 1,
        BmeError::WriteReadError(_) => 2,
        BmeError::UnexpectedChipId(id) => 0x100 | id as u32,
        BmeError::MeasuringTimeOut => 3,
    }
}

#[embassy_executor::task]
async fn pms7003_task(mut rx: UartRx<'static, Blocking>) {
    let mut parser = Pms7003Parser::new();
    MON_PMS7003_STATUS.store(1, Ordering::Relaxed);
    defmt::info!("sensor monitor PMS7003 UART3 RX PC14 9600 8N1");

    loop {
        let mut consumed = 0usize;
        while consumed < 64 {
            match rx.read() {
                Ok(byte) => {
                    consumed += 1;
                    if parser.push(byte) {
                        if parser.checksum_valid() {
                            MON_PMS7003_PM1_0_UG_M3.store(parser.word(10) as u32, Ordering::Relaxed);
                            MON_PMS7003_PM2_5_UG_M3.store(parser.word(12) as u32, Ordering::Relaxed);
                            MON_PMS7003_PM10_UG_M3.store(parser.word(14) as u32, Ordering::Relaxed);
                            MON_PMS7003_VERSION.store(parser.frame[28] as u32, Ordering::Relaxed);
                            MON_PMS7003_SENSOR_ERROR.store(parser.frame[29] as u32, Ordering::Relaxed);
                            MON_PMS7003_LAST_UPDATE_MS
                                .store(embassy_time::Instant::now().as_millis() as u32, Ordering::Relaxed);
                            MON_PMS7003_VALID_FRAMES.fetch_add(1, Ordering::Relaxed);
                            MON_PMS7003_STATUS.store(2, Ordering::Relaxed);
                        } else {
                            MON_PMS7003_INVALID_FRAMES.fetch_add(1, Ordering::Relaxed);
                            MON_PMS7003_STATUS.store(0x0000_00e1, Ordering::Relaxed);
                        }
                    }
                }
                Err(nb::Error::WouldBlock) => break,
                Err(nb::Error::Other(_)) => {
                    MON_PMS7003_UART_ERRORS.fetch_add(1, Ordering::Relaxed);
                    MON_PMS7003_STATUS.store(0x0000_00e2, Ordering::Relaxed);
                    parser.reset();
                    break;
                }
            }
        }
        Timer::after_millis(1).await;
    }
}

struct Pms7003Parser {
    frame: [u8; 32],
    position: usize,
}

impl Pms7003Parser {
    const fn new() -> Self {
        Self {
            frame: [0; 32],
            position: 0,
        }
    }

    fn reset(&mut self) {
        self.position = 0;
    }

    fn push(&mut self, byte: u8) -> bool {
        if self.position < PMS7003_PREFIX.len() {
            if byte == PMS7003_PREFIX[self.position] {
                self.frame[self.position] = byte;
                self.position += 1;
            } else if byte == PMS7003_PREFIX[0] {
                self.frame[0] = byte;
                self.position = 1;
            } else {
                self.position = 0;
            }
            return false;
        }

        self.frame[self.position] = byte;
        self.position += 1;
        if self.position == self.frame.len() {
            self.position = 0;
            true
        } else {
            false
        }
    }

    fn checksum_valid(&self) -> bool {
        let sum = self.frame[..30]
            .iter()
            .fold(0u16, |sum, byte| sum.wrapping_add(*byte as u16));
        sum == self.word(30)
    }

    fn word(&self, offset: usize) -> u16 {
        u16::from_be_bytes([self.frame[offset], self.frame[offset + 1]])
    }
}

async fn run_bulk_monitor<'d, D: Driver<'d>>(
    class: &mut MonitorBulkClass<'d, D>,
    monitor: &mut MonitorUsbLite,
    read_buf: &mut [u8; BULK_MAX_PACKET_SIZE],
    write_buf: &mut [u8; BULK_MAX_PACKET_SIZE],
) -> Result<(), Disconnected> {
    monitor.rx_len = 0;
    class.read_ep.wait_enabled().await;
    class.write_ep.wait_enabled().await;

    loop {
        // HPM5E can expose a retired qTD before the short-packet remaining
        // count has crossed into memory. Clear the packet buffer before each
        // receive so a transient full-size length contains zero padding rather
        // than bytes from an earlier monitor request.
        fill_bytes(read_buf, 0);
        let n = class.read_ep.read(&mut read_buf[..]).await?;
        write_mon_u32(
            addr_of_mut!(MON_LOOP_COUNT),
            read_mon_u32(addr_of!(MON_LOOP_COUNT)).wrapping_add(1),
        );

        monitor.push_rx(&read_buf[..n]);
        let micros = embassy_time::Instant::now().as_micros() as u32;
        while let Some(tx_len) = monitor.process_next(micros) {
            for chunk in monitor.tx[..tx_len].chunks(BULK_MAX_PACKET_SIZE) {
                write_mon_u32(addr_of_mut!(MON_LAST_ERROR), 0x340);
                copy_bytes(&mut write_buf[..chunk.len()], chunk);
                class.write_ep.write(&write_buf[..chunk.len()]).await?;
                write_mon_u32(addr_of_mut!(MON_LAST_ERROR), 0x350);
            }
        }
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
        v if v == atomic_addr(&MON_PANIC_LINE) => Some(MON_PANIC_LINE.load(Ordering::Relaxed)),
        v if v == atomic_addr(&MON_PANIC_COLUMN) => Some(MON_PANIC_COLUMN.load(Ordering::Relaxed)),

        v if v == atomic_addr(&MON_SENSOR_CYCLE_COUNT) => Some(MON_SENSOR_CYCLE_COUNT.load(Ordering::Relaxed)),
        v if v == atomic_addr(&MON_SENSOR_READY_MASK) => Some(MON_SENSOR_READY_MASK.load(Ordering::Relaxed)),
        v if v == atomic_addr(&MON_SENSOR_LAST_ERROR) => Some(MON_SENSOR_LAST_ERROR.load(Ordering::Relaxed)),
        v if v == atomic_addr(&MON_BH1750_SAMPLE_COUNT) => Some(MON_BH1750_SAMPLE_COUNT.load(Ordering::Relaxed)),
        v if v == atomic_addr(&MON_BH1750_RAW) => Some(MON_BH1750_RAW.load(Ordering::Relaxed)),
        v if v == atomic_addr(&MON_BH1750_LUX_X100) => Some(MON_BH1750_LUX_X100.load(Ordering::Relaxed)),
        v if v == atomic_addr(&MON_APDS9930_SAMPLE_COUNT) => Some(MON_APDS9930_SAMPLE_COUNT.load(Ordering::Relaxed)),
        v if v == atomic_addr(&MON_APDS9930_ID) => Some(MON_APDS9930_ID.load(Ordering::Relaxed)),
        v if v == atomic_addr(&MON_APDS9930_STATUS) => Some(MON_APDS9930_STATUS.load(Ordering::Relaxed)),
        v if v == atomic_addr(&MON_APDS9930_CLEAR_RAW) => Some(MON_APDS9930_CLEAR_RAW.load(Ordering::Relaxed)),
        v if v == atomic_addr(&MON_APDS9930_IR_RAW) => Some(MON_APDS9930_IR_RAW.load(Ordering::Relaxed)),
        v if v == atomic_addr(&MON_APDS9930_PROXIMITY_RAW) => Some(MON_APDS9930_PROXIMITY_RAW.load(Ordering::Relaxed)),
        v if v == atomic_addr(&MON_BME680_SAMPLE_COUNT) => Some(MON_BME680_SAMPLE_COUNT.load(Ordering::Relaxed)),
        v if v == atomic_addr(&MON_BME680_TEMP_C_X100) => Some(MON_BME680_TEMP_C_X100.load(Ordering::Relaxed)),
        v if v == atomic_addr(&MON_BME680_HUMIDITY_RH_X100) => {
            Some(MON_BME680_HUMIDITY_RH_X100.load(Ordering::Relaxed))
        }
        v if v == atomic_addr(&MON_BME680_PRESSURE_PA) => Some(MON_BME680_PRESSURE_PA.load(Ordering::Relaxed)),
        v if v == atomic_addr(&MON_BME680_GAS_OHM) => Some(MON_BME680_GAS_OHM.load(Ordering::Relaxed)),
        v if v == atomic_addr(&MON_BME680_GAS_VALID) => Some(MON_BME680_GAS_VALID.load(Ordering::Relaxed)),
        v if v == atomic_addr(&MON_PMS7003_STATUS) => Some(MON_PMS7003_STATUS.load(Ordering::Relaxed)),
        v if v == atomic_addr(&MON_PMS7003_VALID_FRAMES) => Some(MON_PMS7003_VALID_FRAMES.load(Ordering::Relaxed)),
        v if v == atomic_addr(&MON_PMS7003_INVALID_FRAMES) => Some(MON_PMS7003_INVALID_FRAMES.load(Ordering::Relaxed)),
        v if v == atomic_addr(&MON_PMS7003_UART_ERRORS) => Some(MON_PMS7003_UART_ERRORS.load(Ordering::Relaxed)),
        v if v == atomic_addr(&MON_PMS7003_PM1_0_UG_M3) => Some(MON_PMS7003_PM1_0_UG_M3.load(Ordering::Relaxed)),
        v if v == atomic_addr(&MON_PMS7003_PM2_5_UG_M3) => Some(MON_PMS7003_PM2_5_UG_M3.load(Ordering::Relaxed)),
        v if v == atomic_addr(&MON_PMS7003_PM10_UG_M3) => Some(MON_PMS7003_PM10_UG_M3.load(Ordering::Relaxed)),
        v if v == atomic_addr(&MON_PMS7003_VERSION) => Some(MON_PMS7003_VERSION.load(Ordering::Relaxed)),
        v if v == atomic_addr(&MON_PMS7003_SENSOR_ERROR) => Some(MON_PMS7003_SENSOR_ERROR.load(Ordering::Relaxed)),
        v if v == atomic_addr(&MON_PMS7003_LAST_UPDATE_MS) => Some(MON_PMS7003_LAST_UPDATE_MS.load(Ordering::Relaxed)),

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

        v if v == atomic_addr(&MON_USB_STAGE) => Some(hal::usb::diagnostics::STAGE.load(Ordering::Relaxed)),
        v if v == atomic_addr(&MON_USB_USBSTS) => Some(hal::usb::diagnostics::USBSTS.load(Ordering::Relaxed)),
        v if v == atomic_addr(&MON_USB_USBINTR) => Some(hal::usb::diagnostics::USBINTR.load(Ordering::Relaxed)),
        v if v == atomic_addr(&MON_USB_PORTSC1) => Some(hal::usb::diagnostics::PORTSC1.load(Ordering::Relaxed)),
        v if v == atomic_addr(&MON_USB_OTGSC) => Some(hal::usb::diagnostics::OTGSC.load(Ordering::Relaxed)),
        v if v == atomic_addr(&MON_USB_PHY_CTRL0) => Some(hal::usb::diagnostics::PHY_CTRL0.load(Ordering::Relaxed)),
        v if v == atomic_addr(&MON_USB_USBCMD) => Some(hal::usb::diagnostics::USBCMD.load(Ordering::Relaxed)),
        v if v == atomic_addr(&MON_USB_USBMODE) => Some(hal::usb::diagnostics::USBMODE.load(Ordering::Relaxed)),
        v if v == atomic_addr(&MON_USB_IRQ_COUNT) => Some(hal::usb::diagnostics::IRQ_COUNT.load(Ordering::Relaxed)),
        v if v == atomic_addr(&MON_USB_QTD_ADDR) => Some(hal::usb::diagnostics::QTD_ADDR.load(Ordering::Relaxed)),
        v if v == atomic_addr(&MON_USB_QTD_DATA_PTR) => {
            Some(hal::usb::diagnostics::QTD_DATA_PTR.load(Ordering::Relaxed))
        }
        v if v == atomic_addr(&MON_USB_QTD_LEN) => Some(hal::usb::diagnostics::QTD_LEN.load(Ordering::Relaxed)),
        v if v == atomic_addr(&MON_USB_QTD_SYS_ADDR) => {
            Some(hal::usb::diagnostics::QTD_SYS_ADDR.load(Ordering::Relaxed))
        }
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

fn scale_f32_to_u32(value: f32, scale: f32) -> u32 {
    let scaled = value * scale;
    if scaled.is_finite() && scaled > 0.0 {
        scaled.min(u32::MAX as f32) as u32
    } else {
        0
    }
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
    let bytes = value.to_le_bytes();
    buf[offset] = bytes[0];
    buf[offset + 1] = bytes[1];
}

fn write_u32(buf: &mut [u8], offset: usize, value: u32) {
    let bytes = value.to_le_bytes();
    buf[offset] = bytes[0];
    buf[offset + 1] = bytes[1];
    buf[offset + 2] = bytes[2];
    buf[offset + 3] = bytes[3];
}

fn write_u64(buf: &mut [u8], offset: usize, value: u64) {
    let bytes = value.to_le_bytes();
    for (i, byte) in bytes.into_iter().enumerate() {
        buf[offset + i] = byte;
    }
}

fn fill_bytes(buf: &mut [u8], value: u8) {
    for i in 0..buf.len() {
        unsafe { core::ptr::write_volatile(buf.as_mut_ptr().add(i), value) };
    }
}

fn copy_bytes(dst: &mut [u8], src: &[u8]) {
    debug_assert_eq!(dst.len(), src.len());
    for i in 0..dst.len() {
        unsafe {
            let value = core::ptr::read_volatile(src.as_ptr().add(i));
            core::ptr::write_volatile(dst.as_mut_ptr().add(i), value);
        }
    }
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
    EXCEPTION_MCAUSE.store(mcause, Ordering::Relaxed);
    let mepc = read_csr("mepc");
    EXCEPTION_MEPC.store(mepc, Ordering::Relaxed);
    let mtval = read_csr("mtval");
    EXCEPTION_MTVAL.store(mtval, Ordering::Relaxed);
    EXCEPTION_TRAP_FRAME.store(trap_frame as u32, Ordering::Relaxed);
    let mstatus = read_csr("mstatus");
    EXCEPTION_MSTATUS.store(mstatus, Ordering::Relaxed);
    let mie = read_csr("mie");
    EXCEPTION_MIE.store(mie, Ordering::Relaxed);
    let mip = read_csr("mip");
    EXCEPTION_MIP.store(mip, Ordering::Relaxed);

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

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    write_mon_u32(addr_of_mut!(MON_LAST_ERROR), 0xffff_0001);
    if let Some(location) = info.location() {
        MON_PANIC_LINE.store(location.line(), Ordering::Relaxed);
        MON_PANIC_COLUMN.store(location.column(), Ordering::Relaxed);
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
