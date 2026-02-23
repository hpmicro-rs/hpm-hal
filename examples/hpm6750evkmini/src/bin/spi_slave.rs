//! SPI Slave Blocking Example
//!
//! Demonstrates SPI slave mode using data-only mode.
//! The slave waits for a master to initiate transfers.
//!
//! SPI1 Slave pins:
//!   SCLK = PD31 (input from master)
//!   MOSI = PE04 (input from master)
//!   MISO = PD30 (output to master)
//!   CS   = PE03 (input from master)
//!
//! Connect these pins to an SPI master device. Each CS-low transaction must
//! clock exactly 16 bytes. The slave transmits a counter pattern and logs the
//! received bytes.

#![no_main]
#![no_std]

use defmt::*;
use defmt_rtt as _;
use hpm_hal as hal;
use hpm_hal::spi::slave::{Config, SpiSlave};
use panic_halt as _;

#[hal::entry]
fn main() -> ! {
    let p = hal::init(Default::default());

    info!("SPI Slave Example");
    info!("Pins: SCLK=PD31, MOSI=PE04, MISO=PD30, CS=PE03");

    let mut spi_slave = SpiSlave::new(p.SPI1, p.PD31, p.PE04, p.PD30, p.PE03, Config::default());

    info!("SPI Slave initialized, waiting for master...");

    let mut transfer_count: u32 = 0;

    loop {
        let mut tx = [0u8; 16];
        let mut rx = [0u8; 16];

        // Fill TX data: pattern based on transfer count
        for (i, b) in tx.iter_mut().enumerate() {
            *b = ((transfer_count as u8) << 4) | (i as u8);
        }

        info!("Slave ready, TX: {:02x}", &tx[..]);

        match spi_slave.blocking_transfer(&mut rx, &tx) {
            Ok(()) => {
                transfer_count += 1;
                info!("Transfer #{}: RX: {:02x}", transfer_count, &rx[..]);

                let status = spi_slave.status();
                info!("  Status: rx_cnt={}, tx_cnt={}", status.rx_count, status.tx_count);
            }
            Err(e) => {
                error!("Transfer error: {:?}", e);
            }
        }
    }
}
