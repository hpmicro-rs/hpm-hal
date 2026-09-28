#![no_main]
#![no_std]
#![feature(impl_trait_in_assoc_type)]

use core::array;
use core::sync::atomic::{AtomicU32, Ordering};

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_time::Timer;
use hpm_hal as hal;
use hpm_hal::dma::{Transfer, TransferOptions};
use panic_halt as _;
use static_cell::StaticCell;

const WORDS: usize = 256;

#[unsafe(link_section = ".noncacheable")]
static SRC: StaticCell<[u32; WORDS]> = StaticCell::new();
#[unsafe(link_section = ".noncacheable")]
static HDMA_DST: StaticCell<[u32; WORDS]> = StaticCell::new();
#[unsafe(link_section = ".noncacheable")]
static XDMA_DST: StaticCell<[u32; WORDS]> = StaticCell::new();

#[unsafe(no_mangle)]
pub static HPM5E_DMA_VALIDATION_STATUS: AtomicU32 = AtomicU32::new(0);

#[embassy_executor::main(entry = "hpm_hal::entry")]
async fn main(_spawner: Spawner) -> ! {
    let p = hal::init(Default::default());
    let src = SRC.init(array::from_fn(|i| {
        0x5e31_0000u32.wrapping_add(i as u32).rotate_left((i & 31) as u32)
    }));
    let hdma_dst = HDMA_DST.init([0; WORDS]);
    let xdma_dst = XDMA_DST.init([0; WORDS]);

    defmt::info!("HPM5E DMA memcpy validation start, words={}", WORDS);

    Transfer::new_memcpy(p.HDMA_CH0, &src[..], &mut hdma_dst[..], TransferOptions::default()).await;
    if src[..] != hdma_dst[..] {
        HPM5E_DMA_VALIDATION_STATUS.store(0xe001, Ordering::Release);
        defmt::error!("HDMA memcpy mismatch");
        wait_forever().await;
    }
    HPM5E_DMA_VALIDATION_STATUS.store(1, Ordering::Release);
    defmt::info!("HDMA memcpy PASS checksum={=u32:#010x}", checksum(hdma_dst));

    Transfer::new_memcpy(p.XDMA_CH0, &src[..], &mut xdma_dst[..], TransferOptions::default()).await;
    if src[..] != xdma_dst[..] {
        HPM5E_DMA_VALIDATION_STATUS.store(0xe002, Ordering::Release);
        defmt::error!("XDMA memcpy mismatch");
        wait_forever().await;
    }
    HPM5E_DMA_VALIDATION_STATUS.store(2, Ordering::Release);
    defmt::info!("XDMA memcpy PASS checksum={=u32:#010x}", checksum(xdma_dst));
    defmt::info!("HPM5E DMA memcpy validation PASS");

    wait_forever().await
}

fn checksum(words: &[u32]) -> u32 {
    words.iter().fold(0u32, |sum, word| sum.wrapping_add(*word))
}

async fn wait_forever() -> ! {
    loop {
        Timer::after_secs(60).await;
    }
}
