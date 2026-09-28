#![no_main]
#![no_std]
#![feature(f16)]

use core::arch::asm;

use embedded_hal::delay::DelayNs;
use hal::gpio::{Level, Output, Speed};
use hal::pac;
use riscv::delay::McycleDelay;
use {defmt_rtt as _, hpm_hal as hal};

#[hal::entry]
fn main() -> ! {
    let a = 100.0_f16;
    let b  = 200.5_f16;
    let c = a + b;
    defmt::println!("a = {:?}", c.to_le_bytes());

    unsafe {
        asm!("nop");
        asm!("nop");
        asm!("nop");

        asm!("
            bbs a0, 5, -1
        ");
    }

    // let p = hal::init(Default::default());
    let x: u32 = 0xf1f2f3f4;
    let y: u32 = 0xf5f6f7f8;

    let c = x.overflowing_add(y).0;
    let mut tmp: u32 = 0;

    unsafe {
        asm!(
            r#"
                .insn r 0x7f, 0, 0b0100100, {out}, {in1}, {in2}
            "#,
            in1 = in(reg) x,
            in2 = in(reg) y,
            out = out(reg) tmp,
        )
    };

    defmt::println!("x + y = 0x{:08x}", tmp);
    defmt::println!("x + y = 0x{:08x}", c);

    loop {
        unsafe {
            asm!("wfi");
        }
    }
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    defmt::info!("panic");
    loop {}
}
