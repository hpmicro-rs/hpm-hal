use core::ops;

use super::clock_add_to_group;
use crate::pac;
use crate::pac::SYSCTL;
pub use crate::pac::sysctl::vals::{ClockMux, SubDiv};
use crate::time::Hertz;

pub const CLK_32K: Hertz = Hertz(32_768);
pub const CLK_24M: Hertz = Hertz(24_000_000);

// default clock sources
const PLL0CLK0: Hertz = Hertz(400_000_000);
const PLL0CLK1: Hertz = Hertz(333_333_333);
const PLL0CLK2: Hertz = Hertz(250_000_000);

const PLL1CLK0: Hertz = Hertz(480_000_000);
const PLL1CLK1: Hertz = Hertz(320_000_000);

const PLL2CLK0: Hertz = Hertz(516_096_000);
const PLL2CLK1: Hertz = Hertz(451_584_000);

// Chip default state after power-on (before HAL init)
// Assumes CPU runs from PLL0CLK0 with some default divider
// These values will be updated by init() to reflect actual configuration
const CLK_CPU0: Hertz = Hertz(400_000_000); // PLL0CLK0, assume div=1
const CLK_AXI: Hertz = Hertz(400_000_000 / 3); // CLK_CPU0 / 3
const CLK_AHB: Hertz = Hertz(400_000_000 / 3); // CLK_CPU0 / 3

// const F_REF: Hertz = CLK_24M;

/// System clock state
///
/// Initial values assume chip's power-on default state (before HAL init).
/// The `cpu0`, `axi`, `ahb` fields are updated by `init()` to reflect
/// the actual configured frequencies.
///
/// PLL frequencies are chip characteristics and remain constant.
pub(crate) static mut CLOCKS: Clocks = Clocks {
    cpu0: CLK_CPU0,
    axi: CLK_AXI,
    ahb: CLK_AHB,
    pll0clk0: PLL0CLK0,
    pll0clk1: PLL0CLK1,
    pll0clk2: PLL0CLK2,
    pll1clk0: PLL1CLK0,
    pll1clk1: PLL1CLK1,
    pll2clk0: PLL2CLK0,
    pll2clk1: PLL2CLK1,
};

#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Clocks {
    pub cpu0: Hertz,
    pub axi: Hertz,
    pub ahb: Hertz,
    pub pll0clk0: Hertz,
    pub pll0clk1: Hertz,
    pub pll0clk2: Hertz,
    pub pll1clk0: Hertz,
    pub pll1clk1: Hertz,
    pub pll2clk0: Hertz,
    pub pll2clk1: Hertz,
}

impl Clocks {
    pub fn of(&self, src: ClockMux) -> Hertz {
        match src {
            ClockMux::CLK_24M => CLK_24M,
            ClockMux::PLL0CLK0 => self.pll0clk0,
            ClockMux::PLL0CLK1 => self.pll0clk1,
            ClockMux::PLL0CLK2 => self.pll0clk2,
            ClockMux::PLL1CLK0 => self.pll1clk0,
            ClockMux::PLL1CLK1 => self.pll1clk1,
            ClockMux::PLL2CLK0 => self.pll2clk0,
            ClockMux::PLL2CLK1 => self.pll2clk1,
            _ => unreachable!(),
        }
    }

    pub fn get_freq(&self, cfg: &ClockConfig) -> Hertz {
        let clock_in = self.of(cfg.src);
        clock_in / (cfg.raw_div as u32 + 1)
    }

    /// use `pac::clocks::` values as clock index
    pub fn get_clock_freq(&self, clock: usize) -> Hertz {
        let r = SYSCTL.clock(clock).read();
        let clock_in = self.of(r.mux());
        clock_in / (r.div() + 1)
    }
}

pub struct Config {
    pub cpu0: ClockConfig,
    pub axi_div: SubDiv,
    pub ahb_div: SubDiv,
}

impl Default for Config {
    /// HAL startup configuration (applied during init)
    ///
    /// This is NOT the chip's power-on default state, but the configuration
    /// that HAL will apply when `init()` is called.
    ///
    /// C SDK configures:
    /// - CPU = 648MHz (PLL1_CLK0/1, with PLL1 reconfigured to 648MHz)
    /// - AXI = 162MHz (CPU/4)
    /// - AHB = 162MHz (CPU/4)
    ///
    /// Since PLLCTLV2 configuration is not yet implemented, we use:
    /// - CPU = 480MHz (PLL1_CLK0/1, default PLL1 frequency)
    /// - AXI = 160MHz (CPU/3, within 166MHz bus limit)
    /// - AHB = 160MHz (CPU/3, within 166MHz bus limit)
    fn default() -> Self {
        Self {
            cpu0: ClockConfig::new(ClockMux::PLL1CLK0, 1), // 480MHz
            axi_div: SubDiv::DIV3, // 160MHz
            ahb_div: SubDiv::DIV3, // 160MHz
        }
    }
}

impl Config {
    /// Conservative configuration for lower power consumption
    ///
    /// - CPU = 200MHz (PLL0_CLK0/2)
    /// - AXI = 200MHz (CPU/1)
    /// - AHB = 66MHz (CPU/3)
    pub const fn low_power() -> Self {
        Self {
            cpu0: ClockConfig::new(ClockMux::PLL0CLK0, 2), // 200MHz
            axi_div: SubDiv::DIV1,
            ahb_div: SubDiv::DIV3,
        }
    }

    /// High-performance configuration (default)
    ///
    /// - CPU = 480MHz (PLL1_CLK0/1)
    /// - AXI = 160MHz (CPU/3)
    /// - AHB = 160MHz (CPU/3)
    pub const fn high_performance() -> Self {
        Self {
            cpu0: ClockConfig::new(ClockMux::PLL1CLK0, 1), // 480MHz
            axi_div: SubDiv::DIV3,
            ahb_div: SubDiv::DIV3,
        }
    }
}

#[derive(Clone, Copy)]
pub struct ClockConfig {
    pub src: ClockMux,
    /// raw div, 0 to 255, mapping to div 1 to 256
    pub raw_div: u8,
}

impl ClockConfig {
    pub const fn new(src: ClockMux, div: u16) -> Self {
        assert!(div <= 256 && div > 0, "div must be in range 1 to 256");
        ClockConfig {
            src,
            raw_div: div as u8 - 1,
        }
    }
}

pub(crate) unsafe fn init(config: Config) {
    // Bump up DCDC voltage to 1275mV for stable operation at higher frequencies
    // (C SDK uses 1275mV, lower voltage may cause instability)
    pac::PCFG.dcdc_mode().modify(|w| w.set_volt(1275));

    if SYSCTL.clock_cpu(0).read().mux() == ClockMux::CLK_24M {
        // TODO, enable XTAL
        // SYSCTL.global00().modify(|w| w.set_mux(0b11));
    }

    // Core resources
    clock_add_to_group(pac::resources::CPU0, 0);
    clock_add_to_group(pac::resources::AHBP, 0);
    clock_add_to_group(pac::resources::AXIC, 0);
    clock_add_to_group(pac::resources::AXIS, 0);

    // Memory and bus resources
    clock_add_to_group(pac::resources::MCT0, 0);
    clock_add_to_group(pac::resources::FEMC, 0);
    clock_add_to_group(pac::resources::XPI0, 0);
    clock_add_to_group(pac::resources::XPI1, 0);

    clock_add_to_group(pac::resources::TMR0, 0);
    clock_add_to_group(pac::resources::WDG0, 0);
    clock_add_to_group(pac::resources::LMM0, 0);
    clock_add_to_group(pac::resources::RAM0, 0);

    // DMA controllers - required for ENET DMA and other peripherals
    clock_add_to_group(pac::resources::DMA0, 0); // HDMA
    clock_add_to_group(pac::resources::DMA1, 0); // XDMA

    // Ethernet controller - required for ENET0 to function
    clock_add_to_group(pac::resources::ETH0, 0);

    // Motor control - required for PWM peripherals (C SDK: clock_mot0, clock_mot1)
    clock_add_to_group(pac::resources::MOT0, 0);
    clock_add_to_group(pac::resources::MOT1, 0);

    // Synchronization timer and PTP clock - required for ENET PTP
    clock_add_to_group(pac::resources::SYNT, 0);
    clock_add_to_group(pac::resources::PTPC, 0);

    clock_add_to_group(pac::resources::GPIO, 0);

    clock_add_to_group(pac::resources::MBX0, 0);

    // Connect Group0 to CPU0
    SYSCTL.affiliate(0).set().write(|w| w.set_link(1 << 0));

    // clock settings
    SYSCTL.clock_cpu(0).modify(|w| {
        w.set_mux(config.cpu0.src);
        w.set_div(config.cpu0.raw_div);
        // axi
        w.set_sub0_div(config.axi_div);
        // ahb
        w.set_sub1_div(config.ahb_div);
    });

    while SYSCTL.clock_cpu(0).read().glb_busy() {}

    let cpu0_clk = CLOCKS.get_freq(&config.cpu0);
    let ahb_clk = cpu0_clk / config.ahb_div;
    let axi_clk = cpu0_clk / config.axi_div;

    unsafe {
        CLOCKS.cpu0 = cpu0_clk;
        CLOCKS.axi = axi_clk;
        CLOCKS.ahb = ahb_clk;
    }
}

impl ops::Div<SubDiv> for Hertz {
    type Output = Hertz;

    /// raw bits 0 to 15 mapping to div 1 to div 16
    fn div(self, rhs: SubDiv) -> Hertz {
        Hertz(self.0 / (rhs as u32 + 1))
    }
}
