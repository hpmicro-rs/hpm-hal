#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]
#![feature(impl_trait_in_assoc_type)]

use core::arch::asm;

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_time::Timer;
use hpm_hal as hal;
use hpm_hal::gpio::{Input, Level, Output, Pull};

#[embassy_executor::main(entry = "hpm_hal::entry")]
async fn main(_spawner: Spawner) -> ! {
    let p = hal::init(Default::default());

    let mut button = Input::new(p.PA03, Pull::Down); // hpm5300evklite, BOOT1_KEY
    let mut led = Output::new(p.PA10, Level::Low, Default::default());

    defmt::info!("Board inited");

    let mut a = 0_u32;

    loop {
        //        button.wait_for_falling_edge().await;
        //      defmt::info!("PA03 Button pressed! current={}", button.is_high());
        led.toggle();

        // rdinstret

        unsafe {
            asm!("fence.i");
            asm!("rdcycle {cycle}",
            cycle = out(reg) a,
            );
        }
        defmt::info!("cycle: {}", a);

        let a = 1;
        let mut b = 0_u32;
        unsafe {
            asm!("bclri {res}, {input}, 0",
                input = in(reg) a,
                res = out(reg) b,
            )
        }
        defmt::info!("bclri: {}", b);

        // read csr 0xfc2

        // mmsc_cfg
        // 0b1010_0000_0000_0111_1111_0000_0000_1000
        /*
        bit: 3, ECD, CodeDense
        bit: 12, VPLIC, vectored PLIC mode
        bit: 13, EV5PE, Andes V5 performance extension
        bit: 14, LMSLVP, local memory slave port
        bit: 15, PMNDS, Andes-enhanced performance monitoring feature
        bit: 16, CCTLCSR,
        bit: 17, EFHW, FLHW and FSHW instructions. Half-Precision Load/Store Instructions:
        bit: 18, VCCTL, version number of CCTL command
        bit: 29, EDSP, DSP extension
        bit: 31, MSC_EXT, mmsc_cfg2 CSR
         */

        // mmsc_cfg2 = 0x100000
        /*
        bit: 20, RVARCH, mrvarch_cfg CSR is present
         */

        // mrvarch_cfg = 0xf
        /*
        Zba,Zbb,Zbc,Zbs
         */
        let mut c = 0_u32;
        unsafe {
            asm!("csrr {csr}, 0xfc2",
                csr = out(reg) c,
            )
        }
        defmt::info!("csr 0xfc2: {}", c);

        Timer::after_millis(500).await;
    }
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    //let _ = println!("\n\n\n{}", info);

    loop {}
}
