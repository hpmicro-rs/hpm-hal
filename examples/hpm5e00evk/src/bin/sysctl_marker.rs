#![no_main]
#![no_std]

use hpm_hal as hal;
use hpm_hal::pac;
use hpm_hal::sysctl::ClockMux;
use panic_halt as _;

const SYSCTL_CPU0_CLOCK: *const u32 = 0xf400_1800 as *const u32;
const MAGIC: u32 = 0x5359_5343;

#[unsafe(no_mangle)]
#[unsafe(link_section = ".uninit.sysctl_marker")]
static mut SYSCTL_MARKER: [u32; 8] = [0; 8];

#[hal::entry]
fn main() -> ! {
    record(0, MAGIC);
    record(1, 0x10);

    let raw = unsafe { SYSCTL_CPU0_CLOCK.read_volatile() };
    record(2, raw);
    record(1, 0x20);

    let raw_mux = (raw >> 8) & 0x07;
    let raw_div = raw & 0xff;
    record(3, raw_mux);
    record(4, raw_div);
    record(1, 0x30);

    let clock = pac::SYSCTL.clock(pac::clocks::CPU0).read();
    record(5, clock.mux().to_bits() as u32);
    record(1, 0x40);

    let decoded = decode_clock(clock.mux()) / (clock.div() as u32 + 1);
    record(6, decoded);
    record(1, 0x50);

    let frequency = hal::sysctl::clocks().get_clock_freq(pac::clocks::CPU0).0;
    record(7, frequency);
    record(1, 0x60);

    loop {
        riscv::asm::nop();
    }
}

#[inline(never)]
fn decode_clock(mux: ClockMux) -> u32 {
    match mux {
        ClockMux::CLK_24M => 24_000_000,
        ClockMux::PLL0CLK0 => 600_000_000,
        ClockMux::PLL0CLK1 => 500_000_000,
        ClockMux::PLL1CLK0 => 400_000_000,
        ClockMux::PLL1CLK1 => 333_333_333,
        ClockMux::PLL1CLK2 => 250_000_000,
        ClockMux::PLL2CLK0 => 516_096_000,
        ClockMux::PLL2CLK1 => 451_584_000,
    }
}

#[inline(always)]
fn record(index: usize, value: u32) {
    unsafe {
        let base = core::ptr::addr_of_mut!(SYSCTL_MARKER).cast::<u32>();
        base.add(index).write_volatile(value);
    }
}
