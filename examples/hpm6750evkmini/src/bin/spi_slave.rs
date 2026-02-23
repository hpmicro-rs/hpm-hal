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
//! Connect these pins to an SPI master device.
//! The slave echoes back received data with each byte incremented by 1.

#![no_main]
#![no_std]

use defmt::*;
use hpm_hal::mode::Blocking;
use hpm_hal::spi::slave::SpiSlave;
use hpm_hal::spi::SlaveConfig;
use {defmt_rtt as _, hpm_hal as hal, panic_halt as _};

#[hal::entry]
fn main() -> ! {
    let p = hal::init(Default::default());

    info!("SPI Slave Example");
    info!("Pins: SCLK=PD31, MOSI=PE04, MISO=PD30, CS=PE03");

    let slave_config = SlaveConfig {
        data_only: true,
        ..Default::default()
    };

    let mut spi_slave: SpiSlave<'_, Blocking> =
        SpiSlave::new_blocking(p.SPI1, p.PD31, p.PE04, p.PD30, p.PE03, slave_config);

    info!("SPI Slave initialized, waiting for master...");

    let mut transfer_count: u32 = 0;

    loop {
        // Buffer for bidirectional transfer
        // Pre-fill with response data; after transfer, it contains received data
        let mut buf: [u8; 16] = [0xA0; 16];

        // Fill TX data: pattern based on transfer count
        for (i, b) in buf.iter_mut().enumerate() {
            *b = ((transfer_count as u8) << 4) | (i as u8);
        }

        info!("Slave ready, TX: {:02x}", &buf[..]);

        match spi_slave.blocking_transfer_in_place(&mut buf) {
            Ok(()) => {
                transfer_count += 1;
                info!("Transfer #{}: RX: {:02x}", transfer_count, &buf[..]);

                let status = spi_slave.slave_status();
                info!(
                    "  Status: ready={}, rx_cnt={}, tx_cnt={}",
                    status.ready, status.rx_count, status.tx_count
                );
            }
            Err(e) => {
                error!("Transfer error: {:?}", e);
            }
        }
    }
}
