#![no_main]
#![no_std]

use defmt_rtt as _;
use embassy_time::{Duration, Instant};
use hpm_hal as hal;
use hpm_hal::uart::{Error, UartRx};
use panic_halt as _;

const PREFIX: [u8; 4] = [0x42, 0x4d, 0x00, 0x1c];
const REFRESH_PERIOD: Duration = Duration::from_secs(2);

#[hal::entry]
fn main() -> ! {
    let p = hal::init(Default::default());

    let mut config = hal::uart::Config::default();
    config.baudrate = 9_600;
    config.detect_previous_overrun = false;

    let mut rx = UartRx::new_blocking(p.UART3, p.PC14, config).unwrap();
    let mut frame = [0u8; 32];
    let mut next_refresh = Instant::now();
    let mut valid_frames = 0u32;
    let mut invalid_frames = 0u32;

    defmt::info!("PMS7003 UART3 RX: PC14, 9600 8N1, refresh=2s");

    loop {
        match read_frame(&mut rx, &mut frame) {
            Ok(()) => match Measurement::new(&frame) {
                Some(m) => {
                    valid_frames = valid_frames.wrapping_add(1);
                    let now = Instant::now();
                    if now >= next_refresh {
                        report(&m, valid_frames, invalid_frames);
                        next_refresh = now + REFRESH_PERIOD;
                    }
                }
                None => {
                    invalid_frames = invalid_frames.wrapping_add(1);
                    defmt::warn!("PMS7003 invalid frame count={}", invalid_frames);
                }
            },
            Err(_) => defmt::warn!("PMS7003 UART read error"),
        }
    }
}

fn report(m: &Measurement<'_>, valid_frames: u32, invalid_frames: u32) {
    defmt::info!(
        "PMS7003 atmospheric: PM1.0={} PM2.5={} PM10={} ug/m3",
        m.pm1_0_atm(),
        m.pm2_5_atm(),
        m.pm10_atm()
    );
    defmt::info!(
        "PMS7003 CF=1: PM1.0={} PM2.5={} PM10={} ug/m3",
        m.pm1_0_cf1(),
        m.pm2_5_cf1(),
        m.pm10_cf1()
    );
    defmt::info!(
        "PMS7003 particles/0.1L: >0.3um={} >0.5um={} >1.0um={} >2.5um={} >5.0um={} >10um={}",
        m.gt0_3um(),
        m.gt0_5um(),
        m.gt1_0um(),
        m.gt2_5um(),
        m.gt5_0um(),
        m.gt10um()
    );
    defmt::info!(
        "PMS7003 status: version={} error={} ready={} valid={} invalid={}",
        m.version(),
        m.error_code(),
        m.is_ready(),
        valid_frames,
        invalid_frames
    );
}

fn read_frame(rx: &mut UartRx<'_, hal::mode::Blocking>, frame: &mut [u8; 32]) -> Result<(), Error> {
    let mut prefix_pos = 0usize;
    let mut byte = [0u8; 1];

    while prefix_pos < PREFIX.len() {
        rx.blocking_read(&mut byte)?;

        if byte[0] == PREFIX[prefix_pos] {
            frame[prefix_pos] = byte[0];
            prefix_pos += 1;
        } else if byte[0] == PREFIX[0] {
            frame[0] = byte[0];
            prefix_pos = 1;
        } else {
            prefix_pos = 0;
        }
    }

    rx.blocking_read(&mut frame[PREFIX.len()..])
}

struct Measurement<'a>(&'a [u8; 32]);

impl<'a> Measurement<'a> {
    fn new(data: &'a [u8; 32]) -> Option<Self> {
        let measurement = Self(data);
        measurement.checksum().then_some(measurement)
    }

    fn checksum(&self) -> bool {
        let sum = self.0[..30]
            .iter()
            .fold(0u16, |sum, byte| sum.wrapping_add(*byte as u16));
        sum == u16::from_be_bytes([self.0[30], self.0[31]])
    }

    fn pm1_0_cf1(&self) -> u16 {
        word(self.0, 4)
    }

    fn pm2_5_cf1(&self) -> u16 {
        word(self.0, 6)
    }

    fn pm10_cf1(&self) -> u16 {
        word(self.0, 8)
    }

    fn pm1_0_atm(&self) -> u16 {
        word(self.0, 10)
    }

    fn pm2_5_atm(&self) -> u16 {
        word(self.0, 12)
    }

    fn pm10_atm(&self) -> u16 {
        word(self.0, 14)
    }

    fn gt0_3um(&self) -> u16 {
        word(self.0, 16)
    }

    fn gt0_5um(&self) -> u16 {
        word(self.0, 18)
    }

    fn gt1_0um(&self) -> u16 {
        word(self.0, 20)
    }

    fn gt2_5um(&self) -> u16 {
        word(self.0, 22)
    }

    fn gt5_0um(&self) -> u16 {
        word(self.0, 24)
    }

    fn gt10um(&self) -> u16 {
        word(self.0, 26)
    }

    fn version(&self) -> u8 {
        self.0[28]
    }

    fn error_code(&self) -> u8 {
        self.0[29]
    }

    fn is_ready(&self) -> bool {
        self.gt0_3um() != 0 || self.gt0_5um() != 0 || self.gt1_0um() != 0
    }
}

fn word(frame: &[u8; 32], offset: usize) -> u16 {
    u16::from_be_bytes([frame[offset], frame[offset + 1]])
}
