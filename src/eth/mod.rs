//! Ethernet (ENET) driver for HPMicro MCUs
//!
//! This driver supports the ENET peripheral found on HPM6300/6700/6800 series MCUs.
//! It implements the `embassy_net_driver::Driver` trait for integration with embassy-net.
//!
//! # Features
//! - RMII and MII interface support
//! - Full/Half duplex operation
//! - 10/100 Mbps speed
//! - SMI (Station Management Interface) for PHY access
//! - PTP timestamp support (optional)
//!
//! # Example
//! ```no_run
//! use hpm_hal::eth::{Ethernet, Config, GenericSmi};
//!
//! // Create the ethernet driver
//! let eth = Ethernet::new(
//!     p.ENET0,
//!     // RMII pins...
//!     config,
//! );
//! ```

mod descriptors;
pub mod generic_smi;

pub use descriptors::{PacketQueue, RX_BUFFER_SIZE, TX_BUFFER_SIZE};
pub use generic_smi::{GenericPhy, GenericSmi, SmiClockDivider, SmiError};

use core::marker::PhantomData;
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering, fence};

use embassy_hal_internal::{Peri, PeripheralType};
use embassy_net_driver::{Capabilities, HardwareAddress, LinkState};
use embassy_sync::waitqueue::AtomicWaker;

use crate::interrupt::typelevel::Interrupt as _;
use crate::sysctl::{ClockConfig, SealedClockPeripheral};
use crate::{interrupt, pac};
#[cfg(hpm63)]
use crate::pac::sysctl::vals::ClockMux;

/// Ethernet error
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// SMI/MDIO communication error
    Smi,
    /// No link
    NoLink,
    /// Buffer too small
    BufferTooSmall,
    /// Transmit error
    TxError,
    /// Receive error
    RxError,
}

// ============================================================================
// D-Cache Management for Andes cores (HPM6300/6700/6800)
//
// CRITICAL: Even with PMA marking memory as noncacheable, the Andes D-Cache
// requires explicit management for DMA coherency:
// - CPU write -> DMA read: flush_dcache (L1D_VA_WB)
// - DMA write -> CPU read: invalidate_dcache (L1D_VA_INVAL)
// ============================================================================

const CACHELINE_SIZE: u32 = 64;
const L1D_VA_WB: u32 = 1;     // Writeback by virtual address
const L1D_VA_INVAL: u32 = 0;  // Invalidate by virtual address (NOT 3!)

/// Flush D-cache for a memory region (CPU write -> DMA read)
#[inline]
fn flush_dcache(addr: u32, size: u32) {
    let mut current = addr & !(CACHELINE_SIZE - 1);
    let end = addr + size;
    while current < end {
        unsafe {
            core::arch::asm!("csrw 0x7CB, {0}", in(reg) current); // MCCTLBEGINADDR
            core::arch::asm!("csrw 0x7CC, {0}", in(reg) L1D_VA_WB); // MCCTLCOMMAND
        }
        current += CACHELINE_SIZE;
    }
    core::sync::atomic::fence(core::sync::atomic::Ordering::SeqCst);
}

/// Invalidate D-cache for a memory region (DMA write -> CPU read)
#[inline]
fn invalidate_dcache(addr: u32, size: u32) {
    let mut current = addr & !(CACHELINE_SIZE - 1);
    let end = addr + size;
    while current < end {
        unsafe {
            core::arch::asm!("csrw 0x7CB, {0}", in(reg) current); // MCCTLBEGINADDR
            core::arch::asm!("csrw 0x7CC, {0}", in(reg) L1D_VA_INVAL); // MCCTLCOMMAND
        }
        current += CACHELINE_SIZE;
    }
    core::sync::atomic::fence(core::sync::atomic::Ordering::SeqCst);
}

/// Ethernet interface mode
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Mode {
    /// RMII (Reduced Media Independent Interface) - 7 data pins
    Rmii,
    /// MII (Media Independent Interface) - 16 data pins (not supported on all chips)
    Mii,
}

/// Ethernet speed
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Speed {
    /// 10 Mbps
    Speed10M,
    /// 100 Mbps
    Speed100M,
}

impl From<Speed> for bool {
    fn from(speed: Speed) -> bool {
        match speed {
            Speed::Speed10M => false,
            Speed::Speed100M => true,
        }
    }
}

/// Ethernet duplex mode
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Duplex {
    /// Half duplex
    HalfDuplex,
    /// Full duplex
    FullDuplex,
}

impl From<Duplex> for bool {
    fn from(duplex: Duplex) -> bool {
        match duplex {
            Duplex::HalfDuplex => false,
            Duplex::FullDuplex => true,
        }
    }
}

/// Ethernet configuration
#[derive(Clone)]
pub struct Config {
    /// MAC address
    pub mac_addr: [u8; 6],
    /// Interface mode (RMII or MII)
    pub mode: Mode,
    /// DMA Programmable Burst Length (1, 2, 4, 8, 16, 32)
    pub dma_pbl: u8,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            mac_addr: [0x02, 0x00, 0x00, 0x00, 0x00, 0x01],
            mode: Mode::Rmii,
            dma_pbl: 16,
        }
    }
}

/// Ethernet driver state
pub struct State {
    rx_waker: AtomicWaker,
    tx_waker: AtomicWaker,
    link_up: AtomicBool,
    speed: AtomicU32,
}

impl State {
    /// Create a new state
    pub const fn new() -> Self {
        Self {
            rx_waker: AtomicWaker::new(),
            tx_waker: AtomicWaker::new(),
            link_up: AtomicBool::new(false),
            speed: AtomicU32::new(0),
        }
    }
}

/// Internal peripheral info
struct Info {
    regs: pac::enet::Enet,
}

/// Interrupt handler
pub struct InterruptHandler<T: Instance> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> interrupt::typelevel::Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        on_interrupt(T::info().regs, T::state());

        // PLIC ack is handled by typelevel Handler
    }
}

unsafe fn on_interrupt(regs: pac::enet::Enet, state: &'static State) {
    let status = regs.dma_status().read();

    // Normal interrupt summary
    if status.nis() {
        // Receive interrupt
        if status.ri() {
            // Clear the interrupt
            regs.dma_status().write(|w| w.set_ri(true));
            state.rx_waker.wake();
        }

        // Transmit interrupt
        if status.ti() {
            // Clear the interrupt
            regs.dma_status().write(|w| w.set_ti(true));
            state.tx_waker.wake();
        }

        // Clear NIS
        regs.dma_status().write(|w| w.set_nis(true));
    }

    // Abnormal interrupt summary
    if status.ais() {
        // Clear AIS and any abnormal status bits
        regs.dma_status().write(|w| {
            w.set_ais(true);
            w.set_fbi(true);  // Fatal bus error
            w.set_eri(true);  // Early receive
            w.set_eti(true);  // Early transmit
            w.set_rwt(true);  // Receive watchdog timeout
            w.set_rps(true);  // Receive process stopped
            w.set_ru(true);   // Receive buffer unavailable
            w.set_unf(true);  // Transmit underflow
            w.set_ovf(true);  // Receive overflow
            w.set_tjt(true);  // Transmit jabber timeout
            w.set_tps(true);  // Transmit process stopped
        });
    }
}

/// Ethernet driver
pub struct Ethernet<'d, T: Instance> {
    _peri: Peri<'d, T>,
    tx: TxRing<'d>,
    rx: RxRing<'d>,
    mac_addr: [u8; 6],
}

struct TxRing<'d> {
    descriptors: &'d mut [descriptors::TDes],
    buffers: &'d mut [[u8; TX_BUFFER_SIZE]],
    index: usize,
    regs: pac::enet::Enet,
}

struct RxRing<'d> {
    descriptors: &'d mut [descriptors::RDes],
    buffers: &'d mut [[u8; RX_BUFFER_SIZE]],
    index: usize,
    regs: pac::enet::Enet,
}

impl<'d, T: Instance> Ethernet<'d, T> {
    /// Create a new Ethernet driver (RMII mode)
    ///
    /// # Safety
    /// The packet queue must live as long as the driver.
    pub fn new<const TX_COUNT: usize, const RX_COUNT: usize>(
        peri: Peri<'d, T>,
        _irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>>,
        ref_clk: Peri<'d, impl RefClkPin<T>>,
        crs_dv: Peri<'d, impl CrsDvPin<T>>,
        rxd0: Peri<'d, impl Rxd0Pin<T>>,
        rxd1: Peri<'d, impl Rxd1Pin<T>>,
        tx_en: Peri<'d, impl TxEnPin<T>>,
        txd0: Peri<'d, impl Txd0Pin<T>>,
        txd1: Peri<'d, impl Txd1Pin<T>>,
        mdio: Peri<'d, impl MdioPin<T>>,
        mdc: Peri<'d, impl MdcPin<T>>,
        queue: &'d mut PacketQueue<TX_COUNT, RX_COUNT>,
        config: Config,
    ) -> Self {
        #[cfg(feature = "defmt")]
        defmt::info!("ETH: Configuring pins...");

        // Configure pins using WRITE (not modify) to clear all other bits
        // This matches eth_arp_dump.rs behavior
        ref_clk.ioc_pad().func_ctl().write(|w| {
            w.set_alt_select(ref_clk.alt_num());
            w.set_loop_back(true); // Required for internal clock mode
        });
        crs_dv.ioc_pad().func_ctl().write(|w| w.set_alt_select(crs_dv.alt_num()));
        rxd0.ioc_pad().func_ctl().write(|w| w.set_alt_select(rxd0.alt_num()));
        rxd1.ioc_pad().func_ctl().write(|w| w.set_alt_select(rxd1.alt_num()));
        tx_en.ioc_pad().func_ctl().write(|w| w.set_alt_select(tx_en.alt_num()));
        txd0.ioc_pad().func_ctl().write(|w| w.set_alt_select(txd0.alt_num()));
        txd1.ioc_pad().func_ctl().write(|w| w.set_alt_select(txd1.alt_num()));
        mdio.ioc_pad().func_ctl().write(|w| w.set_alt_select(mdio.alt_num()));
        mdc.ioc_pad().func_ctl().write(|w| w.set_alt_select(mdc.alt_num()));

        #[cfg(feature = "defmt")]
        defmt::info!("ETH: Enabling clocks...");

        // Enable clocks - add ETH resource to group 0
        // This is required for ENET peripheral to be accessible
        T::add_resource_group(0);

        // Configure ETH reference clock for HPM6300
        // Using EXTERNAL clock mode: PHY outputs 50MHz, MCU inputs
        // This is simpler than internal mode because:
        // 1. No PLL2 configuration needed
        // 2. No chicken-egg problem with PHY CLKDIR configuration
        //
        // For internal mode (MCU outputs 50MHz), we would need to:
        // 1. Configure PLL2 = 1GHz, PLL2_CLK1 = 250MHz, ETH0 = 50MHz
        // 2. Configure PHY CLKDIR=1 BEFORE enabling REFCLK_OE
        // But that requires MDIO access which needs clock...
        #[cfg(hpm63)]
        {
            #[cfg(feature = "defmt")]
            defmt::info!("ETH: Configuring internal 50MHz reference clock (MCU outputs to PHY)...");
            // Internal clock mode: MCU outputs 50MHz to PHY
            // This matches C SDK's BOARD_ENET_RMII_INT_REF_CLK = enet_phy_rmii_refclk_dir_in
            Self::configure_eth_clock_hpm63();
        }

        let regs = T::info().regs;

        // === DEBUG: Print SYSCTL and clock configuration ===
        #[cfg(feature = "defmt")]
        {
            let sysctl = crate::pac::SYSCTL;
            let global00 = sysctl.global00().read().0;
            defmt::info!("ETH DEBUG: SYSCTL.GLOBAL00 = 0x{:08X} (preset MUX bits = {})",
                global00, global00 & 0xF);
            
            // Read ETH0 clock configuration
            let eth_clock = sysctl.clock(crate::pac::clocks::ETH0).read();
            defmt::info!("ETH DEBUG: ETH0 clock: mux={} div={} preserve={}",
                eth_clock.mux() as u8, eth_clock.div(), eth_clock.preserve());
            
            // Read CPU0 clock for reference
            let cpu_clock = sysctl.clock_cpu(0).read();
            defmt::info!("ETH DEBUG: CPU0 clock: mux={} div={} sub0_div={} sub1_div={}",
                cpu_clock.mux() as u8, cpu_clock.div(), cpu_clock.sub0_div() as u8, cpu_clock.sub1_div() as u8);
        }

        // Configure RMII interface FIRST (C SDK does this before DMA init)
        // HPM6300EVK: Using internal 50MHz reference clock (MCU outputs to PHY, REFCLK_OE=1)
        // PHY (RTL8201) must be configured to INPUT the 50MHz reference clock (CLKDIR=1)
        #[cfg(feature = "defmt")]
        defmt::info!("ETH: Configuring RMII interface (internal refclk, MCU outputs 50MHz)...");
        Self::configure_rmii(regs, true);  // true = internal reference clock (MCU outputs)

        #[cfg(feature = "defmt")]
        defmt::info!("ETH: Initializing DMA (without starting)...");

        // Initialize DMA but DON'T start it yet
        // Key insight: MAC must be configured BEFORE DMA starts
        let (tx, rx) = Self::init_dma_no_start(regs, queue, config.dma_pbl);

        #[cfg(feature = "defmt")]
        defmt::info!("ETH: Configuring MAC (before DMA start)...");

        // Configure MAC BEFORE starting DMA (like eth_arp_dump.rs does)
        Self::configure_mac(regs, &config);

        // Start MAC transmitter and receiver
        #[cfg(feature = "defmt")]
        defmt::info!("ETH: Starting MAC TX/RX...");
        regs.maccfg().modify(|w| {
            w.set_te(true);
            w.set_re(true);
        });

        // NOW start DMA after MAC is configured
        #[cfg(feature = "defmt")]
        defmt::info!("ETH: Starting DMA TX+RX (after MAC configured)...");
        regs.dma_op_mode().modify(|w| {
            w.set_st(true);
            w.set_sr(true);
        });

        // Enable CPU interrupts
        T::Interrupt::unpend();
        unsafe { T::Interrupt::enable() };

        // Check final DMA status
        #[cfg(feature = "defmt")]
        {
            let status = regs.dma_status().read();
            let op_mode = regs.dma_op_mode().read();
            let ts = (status.0 >> 20) & 0x7;
            let rs = (status.0 >> 17) & 0x7;
            defmt::info!("ETH: Final status=0x{:08X} TS={} RS={} TPS={} FBI={}",
                status.0, ts, rs, status.tps(), status.fbi());
            defmt::info!("ETH: Final OpMode: ST={} SR={} TSF={} RSF={}",
                op_mode.st(), op_mode.sr(), op_mode.tsf(), op_mode.rsf());
        }

        // Set default link state to up (100Mbps Full Duplex)
        // User should call set_link() after PHY negotiation if needed
        let state = T::state();
        state.link_up.store(true, Ordering::Relaxed);
        state.speed.store(100, Ordering::Relaxed);

        #[cfg(feature = "defmt")]
        defmt::info!("ETH: Initialization complete!");

        Self {
            _peri: peri,
            tx,
            rx,
            mac_addr: config.mac_addr,
        }
    }

    fn reset(regs: pac::enet::Enet) {
        // Software reset
        regs.dma_bus_mode().modify(|w| w.set_swr(true));

        // Wait for reset to complete with timeout
        let mut timeout = 100_000u32;
        while regs.dma_bus_mode().read().swr() {
            timeout -= 1;
            if timeout == 0 {
                #[cfg(feature = "defmt")]
                defmt::warn!("ETH: Reset timeout!");
                break;
            }
        }

        #[cfg(feature = "defmt")]
        defmt::info!("ETH: Reset complete");
    }

    fn configure_mac(regs: pac::enet::Enet, config: &Config) {
        // Set MAC address
        let mac = &config.mac_addr;
        regs.mac_addr_0_high().write(|w| {
            w.set_addrhi((mac[5] as u16) << 8 | mac[4] as u16);
        });
        regs.mac_addr_0_low().write(|w| {
            w.0 = (mac[3] as u32) << 24
                | (mac[2] as u32) << 16
                | (mac[1] as u32) << 8
                | mac[0] as u32;
        });

        // Configure MAC configuration register
        // Use modify to preserve defaults, only set essential bits
        // (eth_arp_dump.rs uses modify and works correctly)
        regs.maccfg().modify(|w| {
            // Port Select: 1 for 10/100Mbps (RMII)
            w.set_ps(true);
            // Speed: 100Mbps
            w.set_fes(true);
            // Duplex Mode: Full duplex
            w.set_dm(true);
            // Note: TE and RE will be set later
        });

        // Configure frame filter - use modify like eth_arp_dump.rs
        regs.macff().modify(|w| {
            w.set_ra(true);  // Receive all
            w.set_pm(true);  // Pass multicast
            w.set_pr(true);  // Promiscuous mode
        });

        // Configure flow control
        regs.flowctrl().write(|w| {
            w.set_pt(0);      // Pause time
            w.set_dzpq(true); // Disable zero-quanta pause
            w.set_plt(0);     // Pause low threshold
            w.set_up(false);  // Unicast pause frame detect
            w.set_rfe(false); // Receive flow control disable
            w.set_tfe(false); // Transmit flow control disable
            w.set_fcb_bpa(false); // Flow control busy/back pressure
        });

        #[cfg(feature = "defmt")]
        {
            let maccfg = regs.maccfg().read();
            defmt::info!("ETH MACCFG: 0x{:08X}", maccfg.0);
            defmt::info!("  TE={} RE={} PS={} FES={} DM={} IPC={}",
                maccfg.te(), maccfg.re(), maccfg.ps(), maccfg.fes(), maccfg.dm(), maccfg.ipc());
            
            let macff = regs.macff().read();
            defmt::info!("ETH MACFF: 0x{:08X} (RA={} PM={} DBF={})",
                macff.0, macff.ra(), macff.pm(), macff.dbf());
        }
    }

    fn init_dma_no_start<const TX_COUNT: usize, const RX_COUNT: usize>(
        regs: pac::enet::Enet,
        queue: &'d mut PacketQueue<TX_COUNT, RX_COUNT>,
        pbl: u8,
    ) -> (TxRing<'d>, RxRing<'d>) {
        // Wait for clock to stabilize before DMA reset
        // DMA reset requires working RMII RX clock from PHY
        // Give more time for PHY's reference clock output to stabilize
        #[cfg(feature = "defmt")]
        defmt::info!("ETH DMA: Waiting for RMII clock to stabilize...");
        for _ in 0..1_000_000 {
            core::hint::spin_loop();
        }

        // DMA software reset (C SDK does this first in enet_dma_init)
        // NOTE: If DMA reset fails, check RX clock - it requires a working RMII clock
        #[cfg(feature = "defmt")]
        defmt::info!("ETH DMA: Software reset...");
        regs.dma_bus_mode().modify(|w| w.set_swr(true));
        // C SDK uses 10,000,000 iterations for DMA init timeout
        let mut timeout = 10_000_000u32;
        while regs.dma_bus_mode().read().swr() {
            timeout -= 1;
            if timeout == 0 {
                #[cfg(feature = "defmt")]
                defmt::warn!("ETH DMA: Software reset timeout - check RMII RX clock!");
                break;
            }
        }
        #[cfg(feature = "defmt")]
        {
            if timeout > 0 {
                defmt::info!("ETH DMA: Software reset complete (iterations left: {})", timeout);
            }
        }

        #[cfg(feature = "defmt")]
        defmt::info!("ETH DMA: Configuring bus mode...");

        // Configure DMA bus mode - use modify like eth_arp_dump.rs
        // Only set essential bits for 8-word descriptors
        let _ = pbl; // unused for now
        regs.dma_bus_mode().modify(|w| {
            w.set_atds(true);   // 8-word descriptors (required for HPM6300)
            w.set_pblx8(true);  // PBL x 8 mode
            w.set_aal(true);    // Address aligned beats
        });

        #[cfg(feature = "defmt")]
        {
            let bus_mode = regs.dma_bus_mode().read();
            defmt::info!("ETH DMA_BUS_MODE: 0x{:08X}", bus_mode.0);
            defmt::info!("  SWR={} FB={} PBLX8={} PBL={} USP={} AAL={} ATDS={}",
                bus_mode.swr(), bus_mode.fb(), bus_mode.pblx8(),
                bus_mode.pbl(), bus_mode.usp(), bus_mode.aal(), bus_mode.atds());
        }

        // Configure AXI burst length (required when FB=0)
        regs.dma_axi_mode().modify(|w| {
            w.set_blen4(true);
            w.set_blen8(true);
            w.set_blen16(true);
        });

        #[cfg(feature = "defmt")]
        {
            let axi_mode = regs.dma_axi_mode().read();
            defmt::info!("ETH DMA_AXI_MODE: 0x{:08X}", axi_mode.0);
            defmt::info!("  BLEN4={} BLEN8={} BLEN16={}",
                axi_mode.blen4(), axi_mode.blen8(), axi_mode.blen16());
        }

        #[cfg(feature = "defmt")]
        defmt::info!("ETH DMA: Getting buffer addresses...");

        // Get raw pointers for address calculations
        let tx_desc_base = queue.tx_desc.as_ptr();
        let rx_desc_base = queue.rx_desc.as_ptr();
        let tx_buf_base = queue.tx_buf.as_ptr();
        let rx_buf_base = queue.rx_buf.as_ptr();

        #[cfg(feature = "defmt")]
        defmt::info!("ETH DMA: TX desc @ 0x{:08X}, RX desc @ 0x{:08X}",
            tx_desc_base as u32, rx_desc_base as u32);
        #[cfg(feature = "defmt")]
        defmt::info!("ETH DMA: TX buf @ 0x{:08X}, RX buf @ 0x{:08X}",
            tx_buf_base as u32, rx_buf_base as u32);

        #[cfg(feature = "defmt")]
        defmt::info!("ETH DMA: Initializing {} TX descriptors...", TX_COUNT);

        // Initialize TX descriptors (must set buffer address like C SDK!)
        for i in 0..TX_COUNT {
            #[cfg(feature = "defmt")]
            defmt::info!("  TX[{}] init @ 0x{:08X}", i, unsafe { tx_desc_base.add(i) } as u32);

            let desc = &mut queue.tx_desc[i];

            desc.init();

            // Set buffer 1 address (required! C SDK sets this even before transmission)
            let buf_addr = queue.tx_buf[i].as_ptr() as u32;
            desc.set_buffer1(buf_addr);
            #[cfg(feature = "defmt")]
            defmt::info!("  TX[{}] buffer1=0x{:08X}", i, buf_addr);

            // Set up chain mode - point to next descriptor
            let next_idx = (i + 1) % TX_COUNT;
            let next_desc_addr = unsafe { tx_desc_base.add(next_idx) as u32 };
            desc.set_buffer2(next_desc_addr);

            // Write TCH bit directly (tdes0 is 0 after init())
            // TCH = bit 20 = 0x00100000
            unsafe {
                let tdes0_ptr = core::ptr::addr_of!(queue.tx_desc[i]) as *mut u32;
                core::ptr::write_volatile(tdes0_ptr, 0x0010_0000);
            }

            #[cfg(feature = "defmt")]
            defmt::info!("  TX[{}] done: next=0x{:08X}", i, next_desc_addr);
        }
        #[cfg(feature = "defmt")]
        defmt::info!("ETH DMA: TX descriptors OK");

        #[cfg(feature = "defmt")]
        defmt::info!("ETH DMA: Initializing {} RX descriptors...", RX_COUNT);

        // Initialize RX descriptors
        // NOTE: Avoid read-modify-write operations on noncacheable memory
        // which causes hangs on HPM6360. Use direct write operations instead.
        for i in 0..RX_COUNT {
            #[cfg(feature = "defmt")]
            defmt::info!("  RX[{}] init @ 0x{:08X}", i, unsafe { rx_desc_base.add(i) } as u32);

            let buf_addr = unsafe { rx_buf_base.add(i) as u32 };
            let next_idx = (i + 1) % RX_COUNT;
            let next_desc_addr = unsafe { rx_desc_base.add(next_idx) as u32 };

            // Write all descriptor words directly without reading
            // RDes layout: [rdes0, rdes1, rdes2, rdes3]
            unsafe {
                let desc_ptr = rx_desc_base.add(i) as *mut u32;

                // rdes0: OWN bit (bit 31) = 0x8000_0000 (give to DMA)
                core::ptr::write_volatile(desc_ptr, 0x8000_0000);

                // rdes1: RCH (bit 14) + buffer size (bits 12:0) = 0x4000 | RX_BUFFER_SIZE
                core::ptr::write_volatile(desc_ptr.add(1), 0x0000_4000 | (RX_BUFFER_SIZE as u32));

                // rdes2: buffer 1 address
                core::ptr::write_volatile(desc_ptr.add(2), buf_addr);

                // rdes3: next descriptor address (chain mode)
                core::ptr::write_volatile(desc_ptr.add(3), next_desc_addr);
            }

            #[cfg(feature = "defmt")]
            defmt::info!("  RX[{}] done: buf=0x{:08X}, next=0x{:08X}", i, buf_addr, next_desc_addr);
        }

        // Verify RX descriptors were written correctly
        #[cfg(feature = "defmt")]
        {
            for i in 0..RX_COUNT {
                unsafe {
                    let desc_ptr = rx_desc_base.add(i) as *const u32;
                    let rdes0 = core::ptr::read_volatile(desc_ptr);
                    let rdes1 = core::ptr::read_volatile(desc_ptr.add(1));
                    let rdes2 = core::ptr::read_volatile(desc_ptr.add(2));
                    let rdes3 = core::ptr::read_volatile(desc_ptr.add(3));
                    defmt::info!("  RX[{}] verify: rdes0=0x{:08X} rdes1=0x{:08X} rdes2=0x{:08X} rdes3=0x{:08X}",
                        i, rdes0, rdes1, rdes2, rdes3);
                }
            }
        }

        #[cfg(feature = "defmt")]
        defmt::info!("ETH DMA: Setting descriptor list addresses...");
        #[cfg(feature = "defmt")]
        defmt::info!("  TX desc list addr = 0x{:08X}", tx_desc_base as u32);
        #[cfg(feature = "defmt")]
        defmt::info!("  RX desc list addr = 0x{:08X}", rx_desc_base as u32);

        // Set descriptor list addresses
        regs.dma_tx_desc_list_addr()
            .write(|w| w.0 = tx_desc_base as u32);
        regs.dma_rx_desc_list_addr()
            .write(|w| w.0 = rx_desc_base as u32);

        // Verify addresses were set correctly
        #[cfg(feature = "defmt")]
        {
            let tx_list = regs.dma_tx_desc_list_addr().read().0;
            let rx_list = regs.dma_rx_desc_list_addr().read().0;
            defmt::info!("ETH DMA: Verify - TX list reg = 0x{:08X}, RX list reg = 0x{:08X}", tx_list, rx_list);
        }

        #[cfg(feature = "defmt")]
        defmt::info!("ETH DMA: Configuring operation mode...");

        // Configure DMA operation mode - use modify like eth_arp_dump.rs
        // Only set essential bits, preserve defaults
        regs.dma_op_mode().modify(|w| {
            w.set_rsf(true);  // Receive store and forward
            w.set_tsf(true);  // Transmit store and forward
            w.set_efc(true);  // Enable flow control
            w.set_fef(true);  // Forward error frames
        });

        // Enable DMA interrupts before starting DMA (C SDK does this in enet_mode_init)
        regs.dma_intr_en().write(|w| {
            w.set_nie(true);  // Normal interrupt summary enable
            w.set_aie(true);  // Abnormal interrupt summary enable
            w.set_rie(true);  // Receive interrupt enable
            w.set_tie(true);  // Transmit interrupt enable
        });

        // Wait for AXI bus to be idle before starting DMA (C SDK: enet_mode_init)
        #[cfg(feature = "defmt")]
        defmt::info!("ETH DMA: Waiting for AXI bus idle...");
        let mut timeout = 10_000u32;
        while regs.dma_bus_status().read().axirdsts() || regs.dma_bus_status().read().axwhsts() {
            timeout -= 1;
            if timeout == 0 {
                #[cfg(feature = "defmt")]
                defmt::warn!("ETH DMA: AXI bus timeout");
                break;
            }
        }

        // CRITICAL: Flush D-cache for all descriptors before starting DMA
        // CPU wrote descriptor data, DMA needs to see correct values
        let tx_desc_total_size = (TX_COUNT * 32) as u32; // 8 words per descriptor
        let rx_desc_total_size = (RX_COUNT * 32) as u32;
        flush_dcache(tx_desc_base as u32, tx_desc_total_size);
        flush_dcache(rx_desc_base as u32, rx_desc_total_size);
        
        // Ensure all descriptor writes are complete before starting DMA
        // This is critical for proper DMA operation
        unsafe { core::arch::asm!("fence rw, rw") };

        // Verify TX descriptors before starting DMA
        #[cfg(feature = "defmt")]
        {
            defmt::info!("ETH DMA: Verifying TX descriptors before start...");
            for i in 0..TX_COUNT {
                let desc_addr = unsafe { tx_desc_base.add(i) as u32 };
                let tdes0 = unsafe { core::ptr::read_volatile(desc_addr as *const u32) };
                let tdes1 = unsafe { core::ptr::read_volatile((desc_addr + 4) as *const u32) };
                let tdes2 = unsafe { core::ptr::read_volatile((desc_addr + 8) as *const u32) };
                let tdes3 = unsafe { core::ptr::read_volatile((desc_addr + 12) as *const u32) };
                defmt::info!("  TX[{}] @ 0x{:08X}: tdes0=0x{:08X} tdes1=0x{:08X} tdes2=0x{:08X} tdes3=0x{:08X}",
                    i, desc_addr, tdes0, tdes1, tdes2, tdes3);
                defmt::info!("    OWN={} TCH={}", (tdes0 >> 31) & 1, (tdes0 >> 20) & 1);
            }
        }

        // Dump all DMA registers before starting
        #[cfg(feature = "defmt")]
        {
            defmt::info!("ETH DMA: Register dump before start:");
            defmt::info!("  DMA_BUS_MODE = 0x{:08X}", regs.dma_bus_mode().read().0);
            defmt::info!("  DMA_AXI_MODE = 0x{:08X}", regs.dma_axi_mode().read().0);
            defmt::info!("  DMA_TX_DESC_LIST_ADDR = 0x{:08X}", regs.dma_tx_desc_list_addr().read().0);
            defmt::info!("  DMA_RX_DESC_LIST_ADDR = 0x{:08X}", regs.dma_rx_desc_list_addr().read().0);
            defmt::info!("  DMA_OP_MODE = 0x{:08X}", regs.dma_op_mode().read().0);
            defmt::info!("  DMA_STATUS = 0x{:08X}", regs.dma_status().read().0);
            defmt::info!("  DMA_INTR_EN = 0x{:08X}", regs.dma_intr_en().read().0);
        }

        // Flush TX FIFO BEFORE starting DMA (experimental)
        #[cfg(feature = "defmt")]
        defmt::info!("ETH DMA: Pre-flush TX FIFO...");
        regs.dma_op_mode().modify(|w| w.set_ftf(true));
        let mut timeout = 10_000u32;
        while regs.dma_op_mode().read().ftf() {
            timeout -= 1;
            if timeout == 0 {
                #[cfg(feature = "defmt")]
                defmt::warn!("ETH DMA: Pre-flush TX FIFO timeout");
                break;
            }
        }
        #[cfg(feature = "defmt")]
        defmt::info!("ETH DMA: Pre-flush complete");

        // DMA NOT started here - will be started after MAC is configured
        #[cfg(feature = "defmt")]
        defmt::info!("ETH DMA: Init complete (DMA not started yet)");

        fence(Ordering::Release);

        (
            TxRing {
                descriptors: &mut queue.tx_desc,
                buffers: &mut queue.tx_buf,
                index: 0,
                regs,
            },
            RxRing {
                descriptors: &mut queue.rx_desc,
                buffers: &mut queue.rx_buf,
                index: 0,
                regs,
            },
        )
    }

    #[cfg(hpm63)]
    fn configure_rmii(regs: pac::enet::Enet, internal_refclk: bool) {
        // CTRL2 bit definitions for HPM6300
        const PHY_INF_SEL_MASK: u32 = 0xE000; // bits 15:13
        const PHY_INF_SEL_SHIFT: u32 = 13;
        const RMII_TXCLK_SEL: u32 = 1 << 10;  // bit 10
        const REFCLK_OE: u32 = 1 << 19;       // bit 19

        #[cfg(feature = "defmt")]
        {
            let before = regs.ctrl2().read().0;
            let phy_inf_sel = (before & PHY_INF_SEL_MASK) >> PHY_INF_SEL_SHIFT;
            let rmii_txclk_sel = (before & RMII_TXCLK_SEL) != 0;
            let refclk_oe = (before & REFCLK_OE) != 0;
            defmt::info!("ETH CTRL2 BEFORE: 0x{:08X}", before);
            defmt::info!("  PHY_INF_SEL[15:13] = {} (4=RMII, 1=RGMII)", phy_inf_sel);
            defmt::info!("  RMII_TXCLK_SEL[10] = {} (1=PLL clock for TX)", rmii_txclk_sel);
            defmt::info!("  REFCLK_OE[19] = {} (1=output 50MHz, 0=input from PHY)", refclk_oe);
        }

        // Configure CTRL2 for RMII mode (matching C SDK hpm_enet_soc_drv.h)
        // For HPM6300 (HPM6360):
        // - PHY_INF_SEL[15:13] = 100b (4) for RMII mode (001=RGMII, 100=RMII)
        // - RMII_TXCLK_SEL (bit 10) = 1 (use PLL clock for TX)
        // - REFCLK_OE (bit 19) = 1 for internal clock (output 50MHz reference)
        //
        // IMPORTANT: Use modify() instead of write() to preserve other bits in CTRL2
        // C SDK uses |= operator which preserves existing bits

        regs.ctrl2().modify(|w| {
            // Clear PHY_INF_SEL bits and set to RMII mode (4)
            w.0 = (w.0 & !PHY_INF_SEL_MASK) | (4 << PHY_INF_SEL_SHIFT);
            // Set RMII_TXCLK_SEL
            w.0 |= RMII_TXCLK_SEL;
            // Set or clear REFCLK_OE based on internal/external clock mode
            if internal_refclk {
                w.0 |= REFCLK_OE;
            } else {
                w.0 &= !REFCLK_OE;
            }
        });

        #[cfg(feature = "defmt")]
        {
            let after = regs.ctrl2().read().0;
            let phy_inf_sel = (after & PHY_INF_SEL_MASK) >> PHY_INF_SEL_SHIFT;
            let rmii_txclk_sel = (after & RMII_TXCLK_SEL) != 0;
            let refclk_oe = (after & REFCLK_OE) != 0;
            defmt::info!("ETH CTRL2 AFTER: 0x{:08X}", after);
            defmt::info!("  PHY_INF_SEL[15:13] = {} (expected: 4)", phy_inf_sel);
            defmt::info!("  RMII_TXCLK_SEL[10] = {} (expected: true)", rmii_txclk_sel);
            defmt::info!("  REFCLK_OE[19] = {} (expected: {} for {})",
                refclk_oe, internal_refclk, if internal_refclk { "internal" } else { "external" });
        }
    }

    #[cfg(not(hpm63))]
    fn configure_rmii(_regs: pac::enet::Enet, _internal_refclk: bool) {
        // Other chip variants may have different RMII configuration
    }

    /// Configure ETH clock for HPM6300 series
    /// 
    /// Simplified approach matching eth_arp_dump.rs:
    /// - Don't modify PLL2_CLK1 divider (use default ~451MHz)
    /// - Set ETH0 = PLL2CLK1 / 9 ≈ 50MHz
    #[cfg(hpm63)]
    fn configure_eth_clock_hpm63() {
        use crate::pac::SYSCTL;
        
        #[cfg(feature = "defmt")]
        defmt::info!("ETH: Configuring ETH0 = PLL2CLK1 / 9 ≈ 50MHz...");
        
        // Configure ETH0 clock: PLL2CLK1 / 9 (div=8 means divide by 9)
        // This matches eth_arp_dump.rs which works correctly
        SYSCTL.clock(crate::pac::clocks::ETH0).modify(|w| {
            w.set_mux(crate::pac::sysctl::vals::ClockMux::PLL2CLK1);
            w.set_div(8); // /9 ≈ 50MHz
        });
        
        // Wait for clock to stabilize
        while SYSCTL.clock(crate::pac::clocks::ETH0).read().loc_busy() {
            core::hint::spin_loop();
        }
        
        #[cfg(feature = "defmt")]
        {
            let eth_clock = SYSCTL.clock(crate::pac::clocks::ETH0).read();
            defmt::info!("ETH: ETH0 configured: mux={} div={}", 
                eth_clock.mux() as u8, eth_clock.div() + 1);
        }
    }

    /// Start the ethernet peripheral
    pub fn start(&mut self) {
        let regs = T::info().regs;

        // Enable MAC transmitter and receiver
        regs.maccfg().modify(|w| {
            w.set_te(true);
            w.set_re(true);
        });

        // Start DMA transmission and reception
        regs.dma_op_mode().modify(|w| {
            w.set_st(true);
            w.set_sr(true);
        });
    }

    /// Stop the ethernet peripheral
    pub fn stop(&mut self) {
        let regs = T::info().regs;

        // Stop DMA
        regs.dma_op_mode().modify(|w| {
            w.set_st(false);
            w.set_sr(false);
        });

        // Disable MAC
        regs.maccfg().modify(|w| {
            w.set_te(false);
            w.set_re(false);
        });
    }

    /// Set link speed and duplex
    pub fn set_link(&mut self, speed: Speed, duplex: Duplex) {
        let regs = T::info().regs;

        regs.maccfg().modify(|w| {
            w.set_fes(speed.into());
            w.set_dm(duplex.into());
        });

        let state = T::state();
        let speed_val = match speed {
            Speed::Speed10M => 10,
            Speed::Speed100M => 100,
        };
        state.speed.store(speed_val, Ordering::Relaxed);
        state.link_up.store(true, Ordering::Relaxed);
    }

    /// Set link down
    pub fn set_link_down(&mut self) {
        let state = T::state();
        state.link_up.store(false, Ordering::Relaxed);
    }

    /// Get the MAC address
    pub fn mac_addr(&self) -> [u8; 6] {
        self.mac_addr
    }

    /// Create an SMI interface for PHY access
    pub fn smi(&self) -> GenericSmi<T> {
        GenericSmi::new()
    }
}

impl<'d> TxRing<'d> {
    fn available(&self) -> bool {
        // CRITICAL: Invalidate D-cache before reading OWN bit
        // DMA may have cleared OWN after transmission complete
        let desc_addr = &self.descriptors[self.index] as *const _ as u32;
        invalidate_dcache(desc_addr, 32);
        
        !self.descriptors[self.index].is_owned()
    }

    fn transmit(&mut self, len: usize) -> Result<(), Error> {
        let desc = &mut self.descriptors[self.index];
        let desc_addr = desc as *const _ as u32;

        // Invalidate before checking OWN (in case DMA finished)
        invalidate_dcache(desc_addr, 32);

        if desc.is_owned() {
            return Err(Error::TxError);
        }

        // Prepare descriptor for transmission (single write, no read-modify-write)
        let buf_addr = self.buffers[self.index].as_ptr() as u32;

        #[cfg(feature = "defmt")]
        defmt::info!("TX[{}]: buf_addr=0x{:08X}, len={}", self.index, buf_addr, len);

        desc.prepare_tx(buf_addr, len as u16);

        // CRITICAL: Flush D-cache for TX buffer AND descriptor
        // CPU wrote data, DMA needs to see it
        flush_dcache(buf_addr, len as u32);
        flush_dcache(desc_addr, 32);

        #[cfg(feature = "defmt")]
        {
            // Re-read after flush to verify
            invalidate_dcache(desc_addr, 32);
            let tdes0 = unsafe { core::ptr::read_volatile(desc_addr as *const u32) };
            let tdes1 = unsafe { core::ptr::read_volatile((desc_addr + 4) as *const u32) };
            let tdes2 = unsafe { core::ptr::read_volatile((desc_addr + 8) as *const u32) };
            let tdes3 = unsafe { core::ptr::read_volatile((desc_addr + 12) as *const u32) };
            defmt::info!("TX[{}] after prepare: tdes0=0x{:08X} tdes1=0x{:08X} tdes2=0x{:08X} tdes3=0x{:08X}",
                self.index, tdes0, tdes1, tdes2, tdes3);
        }

        // Memory barrier AFTER setting OWN bit (C SDK uses "fence rw, rw")
        fence(Ordering::SeqCst);

        // Debug: Read the full 8-word descriptor that DMA is pointing to
        #[cfg(feature = "defmt")]
        {
            let cur_tx_desc = self.regs.dma_curr_host_tx_desc().read().0;
            if cur_tx_desc != 0 {
                let ptr = cur_tx_desc as *const u32;
                let tdes0 = unsafe { core::ptr::read_volatile(ptr) };
                let tdes1 = unsafe { core::ptr::read_volatile(ptr.add(1)) };
                let tdes2 = unsafe { core::ptr::read_volatile(ptr.add(2)) };
                let tdes3 = unsafe { core::ptr::read_volatile(ptr.add(3)) };
                defmt::info!("TX: cur_desc=0x{:08X} tdes0=0x{:08X} tdes1=0x{:08X} tdes2=0x{:08X} tdes3=0x{:08X}",
                    cur_tx_desc, tdes0, tdes1, tdes2, tdes3);
                defmt::info!("TX:   OWN={} TCH={} FS={} LS={} IC={}",
                    (tdes0 >> 31) & 1, (tdes0 >> 20) & 1, (tdes0 >> 28) & 1, (tdes0 >> 29) & 1, (tdes0 >> 30) & 1);
            } else {
                defmt::info!("TX: DMA cur_desc=0x00000000 (not started)");
            }
        }

        // Check if TX DMA is stopped (TS bits 22:20 in DMA_STATUS == 0)
        // and TPS (bit 1) is set
        let status = self.regs.dma_status().read();
        let ts = (status.0 >> 20) & 0x7;

        if ts == 0 {
            // If FBI (fatal bus error) is set, clear it first
            if status.fbi() {
                #[cfg(feature = "defmt")]
                defmt::warn!("TX: FBI detected! Clearing status before restart...");
                self.regs.dma_status().write(|w| w.0 = 0xFFFF_FFFF);
            }

            #[cfg(feature = "defmt")]
            defmt::info!("TX: TS=0 (stopped), TPS={}, enabling ST only (no toggle)", status.tps());

            // Just enable ST (don't toggle)
            let op_mode = self.regs.dma_op_mode().read();
            if !op_mode.st() {
                #[cfg(feature = "defmt")]
                defmt::info!("TX: ST=false, enabling ST...");
                self.regs.dma_op_mode().modify(|w| w.set_st(true));
            }
        }

        // Clear TU (Transmit Buffer Unavailable) bit before polling
        // TU is write-1-to-clear
        self.regs.dma_status().write(|w| w.0 = 1 << 2); // TU is bit 2
        
        // Poll transmit demand
        self.regs.dma_tx_poll_demand().write(|w| w.0 = 1);
        
        // Small delay for DMA to process
        for _ in 0..100 {
            core::hint::spin_loop();
        }

        // Debug: Check status after poll demand
        #[cfg(feature = "defmt")]
        {
            // Small delay for DMA to process
            for _ in 0..1000 {
                core::hint::spin_loop();
            }
            let new_status = self.regs.dma_status().read();
            let new_ts = (new_status.0 >> 20) & 0x7;
            let new_cur_tx = self.regs.dma_curr_host_tx_desc().read().0;
            let maccfg = self.regs.maccfg().read();
            let op_mode = self.regs.dma_op_mode().read();
            defmt::info!("TX: After poll - status=0x{:08X}, TS={}, cur_desc=0x{:08X}",
                new_status.0, new_ts, new_cur_tx);
            defmt::info!("TX: MACCFG=0x{:08X} (TE={}, RE={}), OpMode ST={} SR={}",
                maccfg.0, maccfg.te(), maccfg.re(), op_mode.st(), op_mode.sr());
        }

        self.index = (self.index + 1) % self.descriptors.len();
        Ok(())
    }

    fn buffer(&mut self) -> Option<&mut [u8]> {
        if self.available() {
            Some(&mut self.buffers[self.index])
        } else {
            None
        }
    }
}

impl<'d> RxRing<'d> {
    fn available(&mut self) -> bool {
        // CRITICAL: Invalidate D-cache before reading OWN bit
        // DMA may have written to descriptor, CPU cache might be stale
        loop {
            let desc_addr = &self.descriptors[self.index] as *const _ as u32;
            invalidate_dcache(desc_addr, 32); // 8-word descriptor = 32 bytes
            
            // Read rdes0 once after invalidate
            let rdes0 = unsafe { core::ptr::read_volatile(desc_addr as *const u32) };
            let owned = (rdes0 & (1 << 31)) != 0;
            let has_error = (rdes0 & (1 << 15)) != 0;
            
            #[cfg(feature = "defmt")]
            {
                static mut COUNTER: u32 = 0;
                static mut LAST_OWNED: bool = true;
                unsafe {
                    COUNTER += 1;
                    if COUNTER <= 5 || owned != LAST_OWNED || has_error {
                        defmt::info!("RX available[{}]: idx={} rdes0=0x{:08X} owned={} err={}", 
                            COUNTER, self.index, rdes0, owned, has_error);
                        LAST_OWNED = owned;
                    }
                }
            }
            
            if owned {
                // DMA still owns this descriptor
                return false;
            }
            
            if has_error {
                // Frame has error - release descriptor and check next
                #[cfg(feature = "defmt")]
                defmt::warn!("RX available: skipping error frame, rdes0=0x{:08X}", rdes0);
                self.release();
                // Continue loop to check next descriptor
                continue;
            }
            
            // Good frame available
            return true;
        }
    }

    fn receive(&mut self) -> Option<(&[u8], usize)> {
        let desc = &self.descriptors[self.index];

        // Invalidate descriptor (already done in available(), but be safe)
        let desc_addr = desc as *const _ as u32;
        invalidate_dcache(desc_addr, 32);

        // Read rdes0 ONCE after invalidate, use this value for all checks
        let rdes0 = unsafe { core::ptr::read_volatile(desc_addr as *const u32) };
        
        // Check OWN bit (bit 31)
        if rdes0 & (1 << 31) != 0 {
            #[cfg(feature = "defmt")]
            defmt::warn!("RX receive(): still owned after invalidate, rdes0=0x{:08X}", rdes0);
            return None;
        }

        // Check for errors (ES bit 15)
        if rdes0 & (1 << 15) != 0 {
            #[cfg(feature = "defmt")]
            defmt::warn!("RX receive(): frame has error (ES=1), rdes0=0x{:08X}, releasing", rdes0);
            // Return descriptor to DMA and skip
            self.release();
            return None;
        }

        // Extract frame length (bits 29:16)
        let len = ((rdes0 >> 16) & 0x3FFF) as usize;
        
        #[cfg(feature = "defmt")]
        defmt::info!("RX receive(): idx={} len={} rdes0=0x{:08X}", self.index, len, rdes0);
        
        // CRITICAL: Invalidate D-cache for RX buffer before reading
        // DMA wrote packet data, CPU cache might be stale
        let buf_addr = self.buffers[self.index].as_ptr() as u32;
        invalidate_dcache(buf_addr, len as u32);
        
        Some((&self.buffers[self.index][..len], len))
    }

    fn release(&mut self) {
        let desc = &mut self.descriptors[self.index];
        let buf_addr = self.buffers[self.index].as_ptr() as u32;

        // Re-initialize descriptor for DMA (single write, no read-modify-write)
        desc.prepare_rx(buf_addr, RX_BUFFER_SIZE as u16);

        // CRITICAL: Flush D-cache after writing descriptor
        // CPU wrote descriptor, DMA needs to see updated values
        let desc_addr = desc as *const _ as u32;
        flush_dcache(desc_addr, 32);

        // Poll receive demand
        self.regs.dma_rx_poll_demand().write(|w| w.0 = 1);

        self.index = (self.index + 1) % self.descriptors.len();
    }
}

// ============================================================================
// Driver trait implementation
// ============================================================================

/// Rx token for embassy-net-driver
pub struct RxToken<'a, 'd> {
    rx: &'a mut RxRing<'d>,
}

/// Tx token for embassy-net-driver
pub struct TxToken<'a, 'd> {
    tx: &'a mut TxRing<'d>,
}

impl<'a, 'd> embassy_net_driver::RxToken for RxToken<'a, 'd> {
    fn consume<R, F>(self, f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        if let Some((_ptr, len)) = self.rx.receive() {
            let result = f(&mut self.rx.buffers[self.rx.index][..len]);
            self.rx.release();
            result
        } else {
            // This shouldn't happen if we properly check availability
            panic!("RxToken consumed but no packet available");
        }
    }
}

impl<'a, 'd> embassy_net_driver::TxToken for TxToken<'a, 'd> {
    fn consume<R, F>(self, len: usize, f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        #[cfg(feature = "defmt")]
        defmt::info!("TxToken::consume len={}", len);

        let buf = self.tx.buffer().expect("TxToken consumed but no buffer available");
        let result = f(&mut buf[..len]);

        #[cfg(feature = "defmt")]
        defmt::info!("TX: calling transmit()");

        self.tx.transmit(len).ok();

        #[cfg(feature = "defmt")]
        defmt::info!("TX: transmit() done");

        result
    }
}

impl<'d, T: Instance> embassy_net_driver::Driver for Ethernet<'d, T> {
    type RxToken<'a> = RxToken<'a, 'd> where Self: 'a;
    type TxToken<'a> = TxToken<'a, 'd> where Self: 'a;

    fn receive(&mut self, cx: &mut core::task::Context) -> Option<(Self::RxToken<'_>, Self::TxToken<'_>)> {
        let state = T::state();

        if self.rx.available() && self.tx.available() {
            Some((RxToken { rx: &mut self.rx }, TxToken { tx: &mut self.tx }))
        } else {
            state.rx_waker.register(cx.waker());
            None
        }
    }

    fn transmit(&mut self, cx: &mut core::task::Context) -> Option<Self::TxToken<'_>> {
        let state = T::state();

        let avail = self.tx.available();
        #[cfg(feature = "defmt")]
        defmt::trace!("Driver::transmit() called, tx.available()={}", avail);

        if avail {
            Some(TxToken { tx: &mut self.tx })
        } else {
            state.tx_waker.register(cx.waker());
            None
        }
    }

    fn capabilities(&self) -> Capabilities {
        let mut caps = Capabilities::default();
        caps.max_transmission_unit = 1514;
        caps.max_burst_size = Some(self.tx.descriptors.len());
        caps
    }

    fn link_state(&mut self, _cx: &mut core::task::Context) -> LinkState {
        let state = T::state();
        if state.link_up.load(Ordering::Relaxed) {
            LinkState::Up
        } else {
            LinkState::Down
        }
    }

    fn hardware_address(&self) -> HardwareAddress {
        HardwareAddress::Ethernet(self.mac_addr)
    }
}

// ============================================================================
// Instance trait and peripheral definitions
// ============================================================================

trait SealedInstance {
    fn info() -> &'static Info;
    fn state() -> &'static State;
}

/// ENET Instance trait
#[allow(private_bounds)]
pub trait Instance: SealedInstance + SealedClockPeripheral + PeripheralType + 'static {
    /// Interrupt for this instance
    type Interrupt: interrupt::typelevel::Interrupt;
}

// ============================================================================
// Pin traits
// ============================================================================

/// RMII Reference Clock pin
pub trait RefClkPin<T: Instance>: crate::gpio::Pin {
    /// Get the alternate function number
    fn alt_num(&self) -> u8;
}

/// RMII CRS_DV pin
pub trait CrsDvPin<T: Instance>: crate::gpio::Pin {
    fn alt_num(&self) -> u8;
}

/// RMII RXD0 pin
pub trait Rxd0Pin<T: Instance>: crate::gpio::Pin {
    fn alt_num(&self) -> u8;
}

/// RMII RXD1 pin
pub trait Rxd1Pin<T: Instance>: crate::gpio::Pin {
    fn alt_num(&self) -> u8;
}

/// RMII TX_EN pin
pub trait TxEnPin<T: Instance>: crate::gpio::Pin {
    fn alt_num(&self) -> u8;
}

/// RMII TXD0 pin
pub trait Txd0Pin<T: Instance>: crate::gpio::Pin {
    fn alt_num(&self) -> u8;
}

/// RMII TXD1 pin
pub trait Txd1Pin<T: Instance>: crate::gpio::Pin {
    fn alt_num(&self) -> u8;
}

/// MDIO pin
pub trait MdioPin<T: Instance>: crate::gpio::Pin {
    fn alt_num(&self) -> u8;
}

/// MDC pin
pub trait MdcPin<T: Instance>: crate::gpio::Pin {
    fn alt_num(&self) -> u8;
}

// Manual pin implementations for ENET0 on HPM6300 series
// These should be auto-generated from hpm-data in the future

macro_rules! impl_enet_pin {
    ($peri:ident, $pin:ident, $trait:ident, $alt:expr) => {
        impl $trait<crate::peripherals::$peri> for crate::peripherals::$pin {
            fn alt_num(&self) -> u8 {
                $alt
            }
        }
    };
}

// ENET0 pins for HPM6300 series (from pinmux data)
// RMII Group AB (PA06-PA13)
#[cfg(peri_enet0)]
impl_enet_pin!(ENET0, PA06, TxEnPin, 18);
#[cfg(peri_enet0)]
impl_enet_pin!(ENET0, PA07, Txd1Pin, 18);
#[cfg(peri_enet0)]
impl_enet_pin!(ENET0, PA08, Txd0Pin, 18);
#[cfg(peri_enet0)]
impl_enet_pin!(ENET0, PA09, Rxd1Pin, 18);
#[cfg(peri_enet0)]
impl_enet_pin!(ENET0, PA10, Rxd0Pin, 18);
#[cfg(peri_enet0)]
impl_enet_pin!(ENET0, PA11, CrsDvPin, 18);
#[cfg(peri_enet0)]
impl_enet_pin!(ENET0, PA12, RefClkPin, 18);
#[cfg(peri_enet0)]
impl_enet_pin!(ENET0, PA13, MdioPin, 19);
#[cfg(peri_enet0)]
impl_enet_pin!(ENET0, PA14, MdcPin, 19);

// RMII Group CD (PB00-PB07) - alternative pins
#[cfg(peri_enet0)]
impl_enet_pin!(ENET0, PB00, Rxd1Pin, 18);
#[cfg(peri_enet0)]
impl_enet_pin!(ENET0, PB01, Rxd0Pin, 18);
#[cfg(peri_enet0)]
impl_enet_pin!(ENET0, PB02, CrsDvPin, 18);
#[cfg(peri_enet0)]
impl_enet_pin!(ENET0, PB03, Txd0Pin, 18);
#[cfg(peri_enet0)]
impl_enet_pin!(ENET0, PB04, Txd1Pin, 18);
#[cfg(peri_enet0)]
impl_enet_pin!(ENET0, PB05, RefClkPin, 18);
#[cfg(peri_enet0)]
impl_enet_pin!(ENET0, PB06, TxEnPin, 18);
#[cfg(peri_enet0)]
impl_enet_pin!(ENET0, PB09, MdioPin, 19);
#[cfg(peri_enet0)]
impl_enet_pin!(ENET0, PB10, MdcPin, 19);

// RMII Group EF (PA15-PA26) - HPM6300EVK uses this group
#[cfg(peri_enet0)]
impl_enet_pin!(ENET0, PA15, MdioPin, 19);
#[cfg(peri_enet0)]
impl_enet_pin!(ENET0, PA16, MdcPin, 19);
#[cfg(peri_enet0)]
impl_enet_pin!(ENET0, PA17, Rxd1Pin, 18);
#[cfg(peri_enet0)]
impl_enet_pin!(ENET0, PA18, Rxd0Pin, 18);
#[cfg(peri_enet0)]
impl_enet_pin!(ENET0, PA19, CrsDvPin, 18);
#[cfg(peri_enet0)]
impl_enet_pin!(ENET0, PA20, Txd0Pin, 18);
#[cfg(peri_enet0)]
impl_enet_pin!(ENET0, PA21, Txd1Pin, 18);
#[cfg(peri_enet0)]
impl_enet_pin!(ENET0, PA22, RefClkPin, 18);
#[cfg(peri_enet0)]
impl_enet_pin!(ENET0, PA23, TxEnPin, 18);
#[cfg(peri_enet0)]
impl_enet_pin!(ENET0, PA25, MdcPin, 19);
#[cfg(peri_enet0)]
impl_enet_pin!(ENET0, PA26, MdioPin, 19);

// Instance implementation for ENET0
#[cfg(peri_enet0)]
static ENET0_STATE: State = State::new();

#[cfg(peri_enet0)]
impl SealedInstance for crate::peripherals::ENET0 {
    fn info() -> &'static Info {
        static INFO: Info = Info {
            regs: unsafe { pac::enet::Enet::from_ptr(pac::ENET0.as_ptr()) },
        };
        &INFO
    }

    fn state() -> &'static State {
        &ENET0_STATE
    }
}

#[cfg(peri_enet0)]
impl Instance for crate::peripherals::ENET0 {
    type Interrupt = crate::interrupt::typelevel::ENET0;
}
