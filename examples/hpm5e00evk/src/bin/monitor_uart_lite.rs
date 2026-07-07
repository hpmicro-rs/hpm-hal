#![no_main]
#![no_std]

use core::ptr::{addr_of, addr_of_mut};

use defmt_rtt as _;
use embedded_hal::delay::DelayNs;
use embedded_hal_nb::serial::Read as _;
use hpm_hal as hal;
use hpm_hal::gpio::{Level, Output};
use hpm_hal::uart::Uart;
use nb::Error as NbError;
use panic_halt as _;
use riscv::delay::McycleDelay;

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
const MAX_PACKET_SIZE: usize = 160;

#[unsafe(no_mangle)]
static mut MON_LOOP_COUNT: u32 = 0;
#[unsafe(no_mangle)]
static mut MON_LED_STATE: u32 = 0;
#[unsafe(no_mangle)]
static mut MON_LAST_ERROR: u32 = 0;
#[unsafe(no_mangle)]
static mut MON_RX_PACKETS: u32 = 0;

struct MonitorPacket {
    index: u16,
    cmd: u16,
    payload_len: usize,
}

struct MonitorUartLite {
    rx: [u8; MAX_PACKET_SIZE],
    rx_len: usize,
    tx: [u8; MAX_PACKET_SIZE],
}

impl MonitorUartLite {
    const fn new() -> Self {
        Self {
            rx: [0; MAX_PACKET_SIZE],
            rx_len: 0,
            tx: [0; MAX_PACKET_SIZE],
        }
    }

    fn poll(&mut self, uart: &mut Uart<'_, hal::mode::Blocking>, micros: u32) {
        loop {
            match uart.read() {
                Ok(byte) => {
                    if self.rx_len == self.rx.len() {
                        self.rx_len = 0;
                        write_mon_u32(addr_of_mut!(MON_LAST_ERROR), RESULT_HEAD_ERR as u32);
                    }
                    self.rx[self.rx_len] = byte;
                    self.rx_len += 1;
                }
                Err(NbError::WouldBlock) => break,
                Err(NbError::Other(_)) => {
                    self.rx_len = 0;
                    write_mon_u32(addr_of_mut!(MON_LAST_ERROR), RESULT_HEAD_ERR as u32);
                    break;
                }
            }
        }

        while let Some(frame_len) = self.next_frame_len() {
            let frame = &self.rx[..frame_len];
            let parsed = parse_packet(frame);
            let tx_len = match parsed {
                Ok(packet) => self.handle_packet(packet, micros),
                Err(result) => {
                    let index = read_u16(frame, INDEX_OFFSET).unwrap_or(0);
                    let cmd = read_u16(frame, CMD_OFFSET).unwrap_or(CMD_CONNECT | MONITOR_CMD_HOST_MASK)
                        & !MONITOR_CMD_HOST_MASK;
                    write_mon_u32(addr_of_mut!(MON_LAST_ERROR), result as u32);
                    self.write_response(index, cmd, result, micros, 0)
                }
            };
            let _ = uart.blocking_write(&self.tx[..tx_len]);
            self.consume(frame_len);
        }
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
        write_mon_u32(addr_of_mut!(MON_RX_PACKETS), read_mon_u32(addr_of!(MON_RX_PACKETS)) + 1);

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
        self.tx.fill(0);
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
        payload[4..12].copy_from_slice(&202607050001u64.to_le_bytes());
        payload[12..16].copy_from_slice(&(MAX_PACKET_SIZE as u32).to_le_bytes());
        payload[16..20].copy_from_slice(&0u32.to_le_bytes());
        payload[20..22].copy_from_slice(&8u16.to_le_bytes());
        payload[22..24].copy_from_slice(&0u16.to_le_bytes());
        MONITOR_INFO_SIZE
    }

    fn process_getvalue(&mut self, payload_len: usize) -> u16 {
        let payload = &self.rx[PAYLOAD_OFFSET..PAYLOAD_OFFSET + payload_len];
        if payload.len() < 20 {
            return RESULT_PAYLOAD_ERR;
        }

        let pkt_count = match read_u16(payload, 0) {
            Some(v) => v as usize,
            None => return RESULT_PAYLOAD_ERR,
        };

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
    let loop_count = addr_of!(MON_LOOP_COUNT) as u32;
    let led_state = addr_of!(MON_LED_STATE) as u32;
    let last_error = addr_of!(MON_LAST_ERROR) as u32;
    let rx_packets = addr_of!(MON_RX_PACKETS) as u32;

    match addr {
        v if v == loop_count => Some(read_mon_u32(addr_of!(MON_LOOP_COUNT))),
        v if v == led_state => Some(read_mon_u32(addr_of!(MON_LED_STATE))),
        v if v == last_error => Some(read_mon_u32(addr_of!(MON_LAST_ERROR))),
        v if v == rx_packets => Some(read_mon_u32(addr_of!(MON_RX_PACKETS))),
        _ => None,
    }
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

#[hal::entry]
fn main() -> ! {
    write_mon_u32(addr_of_mut!(MON_LAST_ERROR), 0x100);
    let p = hal::init(Default::default());
    write_mon_u32(addr_of_mut!(MON_LAST_ERROR), 0x110);
    let mut delay = McycleDelay::new(hal::sysctl::clocks().cpu0.0);

    let mut led = Output::new(p.PC28, Level::Low, Default::default());
    write_mon_u32(addr_of_mut!(MON_LAST_ERROR), 0x120);

    let mut uart = match Uart::new_blocking(p.UART0, p.PA01, p.PA00, Default::default()) {
        Ok(uart) => {
            write_mon_u32(addr_of_mut!(MON_LAST_ERROR), 0x131);
            uart
        }
        Err(_) => {
            write_mon_u32(addr_of_mut!(MON_LAST_ERROR), 0xe001);
            loop {
                write_mon_u32(
                    addr_of_mut!(MON_LOOP_COUNT),
                    read_mon_u32(addr_of!(MON_LOOP_COUNT)).wrapping_add(1),
                );
                led.toggle();
                delay.delay_ms(200);
            }
        }
    };
    let mut monitor = MonitorUartLite::new();

    let mut led_on = false;
    let mut micros = 0u32;

    loop {
        write_mon_u32(
            addr_of_mut!(MON_LOOP_COUNT),
            read_mon_u32(addr_of!(MON_LOOP_COUNT)).wrapping_add(1),
        );

        monitor.poll(&mut uart, micros);

        if read_mon_u32(addr_of!(MON_LOOP_COUNT)) % 20_000 == 0 {
            led.toggle();
            led_on = !led_on;
            write_mon_u32(addr_of_mut!(MON_LED_STATE), led_on as u32);
        }

        delay.delay_us(50);
        micros = micros.wrapping_add(50);
    }
}
