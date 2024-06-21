//! System control, clocks, group links.

use core::mem::MaybeUninit;

use crate::pac::sysctl::vals;
pub use crate::pac::sysctl::vals::ClockMux;
use crate::pac::SYSCTL;
use crate::time::Hertz;

pub const CLK_24M: Hertz = Hertz(24_000_000);

pub const PLL0CLK0: Hertz = Hertz(720_000_000);
pub const PLL0CLK1: Hertz = Hertz(600_000_000);
pub const PLL0CLK2: Hertz = Hertz(400_000_000);

pub const PLL1CLK0: Hertz = Hertz(800_000_000);
pub const PLL1CLK1: Hertz = Hertz(666_000_000);
pub const PLL1CLK2: Hertz = Hertz(500_000_000);
pub const PLL1CLK3: Hertz = Hertz(266_000_000);

/// The default system clock configuration
static mut CLOCKS: MaybeUninit<Clocks> = MaybeUninit::uninit();

#[derive(Clone, Copy, Debug)]
pub struct Clocks {
    /// CPU0
    pub hart0: Hertz,
    /// AHB clock: HDMA, HRAM, MOT, ACMP, GPIO, ADC/DAC
    pub ahb: Hertz,

    // System clock source
    pub clk_24m: Hertz,
    pub pll0clk0: Hertz,
    pub pll0clk1: Hertz,
    pub pll0clk2: Hertz,
    pub pll1clk0: Hertz,
    pub pll1clk1: Hertz,
    pub pll1clk2: Hertz,
    pub pll1clk3: Hertz,
}

impl Clocks {
    pub fn of(&self, src: ClockMux) -> Hertz {
        match src {
            vals::ClockMux::CLK_24M => self.clk_24m,
            vals::ClockMux::PLL0CLK0 => self.pll0clk0,
            vals::ClockMux::PLL0CLK1 => self.pll0clk1,
            vals::ClockMux::PLL0CLK2 => self.pll0clk2,
            vals::ClockMux::PLL1CLK0 => self.pll1clk0,
            vals::ClockMux::PLL1CLK1 => self.pll1clk1,
            vals::ClockMux::PLL1CLK2 => self.pll1clk2,
            vals::ClockMux::PLL1CLK3 => self.pll1clk3,
        }
    }

    pub fn get_freq(&self, cfg: &ClockCfg) -> Hertz {
        let clock_in = self.of(cfg.src);
        clock_in / (cfg.raw_div as u32 + 1)
    }
}

pub struct Config {
    pub hart0: ClockCfg,
    /// SUB0_DIV, 4bit, 1 to 16
    pub raw_ahb_div: u8,
}

impl Default for Config {
    fn default() -> Self {
        Config {
            hart0: ClockCfg {
                src: vals::ClockMux::PLL0CLK0,
                raw_div: 1, // div 2
            },
            raw_ahb_div: 1, // div 2
        }
    }
}

#[derive(Clone, Copy)]
pub struct ClockCfg {
    pub src: vals::ClockMux,
    /// raw div, 0 to 255, mapping to 1 to div 256
    pub raw_div: u8,
}

impl ClockCfg {
    pub(crate) fn to_frequency(&self) -> Hertz {
        let src = match self.src {
            vals::ClockMux::CLK_24M => CLK_24M.0,
            vals::ClockMux::PLL0CLK0 => PLL0CLK0.0,
            vals::ClockMux::PLL0CLK1 => PLL0CLK1.0,
            vals::ClockMux::PLL0CLK2 => PLL0CLK2.0,
            vals::ClockMux::PLL1CLK0 => PLL1CLK0.0,
            vals::ClockMux::PLL1CLK1 => PLL1CLK1.0,
            vals::ClockMux::PLL1CLK2 => PLL1CLK2.0,
            vals::ClockMux::PLL1CLK3 => PLL1CLK3.0,
        };
        Hertz(src / (self.raw_div as u32 + 1))
    }
}

pub(crate) unsafe fn init(config: Config) {
    SYSCTL.group0(0).value().write(|w| w.0 = 0xFFFFFFFF);
    SYSCTL.group0(1).value().write(|w| w.0 = 0xFFFFFFFF);

    SYSCTL.affiliate(0).set().write(|w| w.set_link(1));

    SYSCTL.clock_cpu(0).modify(|w| {
        w.set_mux(config.hart0.src);
        w.set_div(config.hart0.raw_div);
        w.set_sub0_div(config.raw_ahb_div);
    });

    while SYSCTL.clock_cpu(0).read().glb_busy() {}

    let hart0_clk = config.hart0.to_frequency();

    let ahb_clk = hart0_clk / (config.raw_ahb_div as u32 + 1);

    let freqs = Clocks {
        hart0: hart0_clk,
        ahb: ahb_clk,

        clk_24m: CLK_24M,
        pll0clk0: PLL0CLK0,
        pll0clk1: PLL0CLK1,
        pll0clk2: PLL0CLK2,
        pll1clk0: PLL1CLK0,
        pll1clk1: PLL1CLK1,
        pll1clk2: PLL1CLK2,
        pll1clk3: PLL1CLK3,
    };
    CLOCKS = MaybeUninit::new(freqs);

    let _ = config;
}

pub fn clocks() -> &'static Clocks {
    unsafe { CLOCKS.assume_init_ref() }
}
