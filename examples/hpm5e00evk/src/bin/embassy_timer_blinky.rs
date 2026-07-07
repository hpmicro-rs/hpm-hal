#![no_main]
#![no_std]
#![feature(impl_trait_in_assoc_type)]

use core::sync::atomic::{AtomicU32, Ordering};

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use hpm_hal as hal;
use hpm_hal::gpio::{Level, Output};
use panic_halt as _;

#[unsafe(no_mangle)]
static EMBASSY_TIMER_TICKS: AtomicU32 = AtomicU32::new(0);

#[unsafe(no_mangle)]
static EXCEPTION_MCAUSE: AtomicU32 = AtomicU32::new(0xffff_ffff);
#[unsafe(no_mangle)]
static EXCEPTION_MEPC: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static EXCEPTION_MTVAL: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static EXCEPTION_TICKS: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static EXCEPTION_TRAP_FRAME: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static EXCEPTION_MSTATUS: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static EXCEPTION_MIE: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static EXCEPTION_MIP: AtomicU32 = AtomicU32::new(0);
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
}

#[embassy_executor::main(entry = "hpm_hal::entry")]
async fn main(_spawner: Spawner) -> ! {
    let p = hal::init(Default::default());

    let mut led0 = Output::new(p.PD06, Level::Low, Default::default());
    let mut led1 = Output::new(p.PD07, Level::High, Default::default());

    let period = Duration::from_millis(500);

    loop {
        Timer::after(period).await;

        EMBASSY_TIMER_TICKS.fetch_add(1, Ordering::Relaxed);
        led0.toggle();
        led1.toggle();
    }
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

    EXCEPTION_MCAUSE.store(mcause, Ordering::Relaxed);
    EXCEPTION_MEPC.store(mepc, Ordering::Relaxed);
    EXCEPTION_MTVAL.store(mtval, Ordering::Relaxed);
    EXCEPTION_TICKS.store(EMBASSY_TIMER_TICKS.load(Ordering::Relaxed), Ordering::Relaxed);
    EXCEPTION_TRAP_FRAME.store(trap_frame as u32, Ordering::Relaxed);
    EXCEPTION_MSTATUS.store(mstatus, Ordering::Relaxed);
    EXCEPTION_MIE.store(mie, Ordering::Relaxed);
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
            _ => core::hint::unreachable_unchecked(),
        }
    }
    value as u32
}
