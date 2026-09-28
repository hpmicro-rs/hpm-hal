#![no_main]
#![no_std]
#![feature(impl_trait_in_assoc_type)]

use core::sync::atomic::{AtomicU32, Ordering};

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_time::Timer;
use hpm_hal as hal;
use hpm_hal::mode::Async;
use hpm_hal::spi::{Config, Spi, TransMode, TransferConfig};
use hpm_hal::time::Hertz;
use panic_halt as _;

const ITERATIONS: u32 = 100;
const TEST_PATTERN: [u8; 32] = [
    0x00, 0xff, 0x5a, 0xa5, 0x12, 0x34, 0x56, 0x78, 0x87, 0x65, 0x43, 0x21, 0x0f, 0xf0, 0x33, 0xcc, 0x69, 0x96, 0x3c,
    0xc3, 0x55, 0xaa, 0x7e, 0xe7, 0x81, 0x18, 0x24, 0x42, 0xbd, 0xdb, 0x01, 0x80,
];

#[unsafe(no_mangle)]
pub static HPM5E_SPI_VALIDATION_STATUS: AtomicU32 = AtomicU32::new(0);

#[embassy_executor::main(entry = "hpm_hal::entry")]
async fn main(_spawner: Spawner) -> ! {
    let p = hal::init(Default::default());
    let config = Config {
        frequency: Hertz(1_000_000),
        ..Default::default()
    };
    let mut spi: Spi<'_, Async> = Spi::new(p.SPI1, p.PC10, p.PC13, p.PC12, p.HDMA_CH1, p.HDMA_CH2, config);

    defmt::info!("HPM5E SPI1 loopback start: PC13 MOSI -> PC12 MISO, SCLK=PC10, 1MHz");
    let transfer_config = TransferConfig {
        transfer_mode: TransMode::WRITE_READ_TOGETHER,
        ..Default::default()
    };

    for iteration in 0..ITERATIONS {
        let mut data = TEST_PATTERN;
        for byte in &mut data {
            *byte ^= iteration as u8;
        }
        let expected = data;

        if spi.transfer_in_place(&mut data, &transfer_config).await.is_err() {
            HPM5E_SPI_VALIDATION_STATUS.store(0xe100 | iteration, Ordering::Release);
            defmt::error!("SPI loopback transfer error at iteration={}", iteration);
            wait_forever().await;
        }
        if data != expected {
            HPM5E_SPI_VALIDATION_STATUS.store(0xe200 | iteration, Ordering::Release);
            defmt::error!(
                "SPI loopback mismatch iteration={} expected0={=u8:#04x} actual0={=u8:#04x}",
                iteration,
                expected[0],
                data[0]
            );
            wait_forever().await;
        }
    }

    HPM5E_SPI_VALIDATION_STATUS.store(ITERATIONS, Ordering::Release);
    defmt::info!("HPM5E SPI1 async DMA loopback PASS, iterations={}", ITERATIONS);
    wait_forever().await
}

async fn wait_forever() -> ! {
    loop {
        Timer::after_secs(60).await;
    }
}
