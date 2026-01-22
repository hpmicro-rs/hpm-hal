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
        // Configure pins using WRITE (not modify) to clear all other bits
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

        // Enable clocks - add ETH resource to group 0
        T::add_resource_group(0);

        // Configure ETH reference clock for HPM6300
        #[cfg(hpm63)]
        Self::configure_eth_clock_hpm63();

        let regs = T::info().regs;

        // Configure RMII interface (internal refclk, MCU outputs 50MHz)
        Self::configure_rmii(regs, true);

        // Initialize DMA but DON'T start it yet (MAC must be configured first)
        let (tx, rx) = Self::init_dma_no_start(regs, queue, config.dma_pbl);

        // Configure MAC BEFORE starting DMA
        Self::configure_mac(regs, &config);

        // Start MAC transmitter and receiver
        regs.maccfg().modify(|w| {
            w.set_te(true);
            w.set_re(true);
        });

        // Start DMA after MAC is configured
        regs.dma_op_mode().modify(|w| {
            w.set_st(true);
            w.set_sr(true);
        });

        // Enable CPU interrupts
        T::Interrupt::unpend();
        unsafe { T::Interrupt::enable() };

        // Set default link state to up (100Mbps Full Duplex)
        let state = T::state();
        state.link_up.store(true, Ordering::Relaxed);
        state.speed.store(100, Ordering::Relaxed);

        #[cfg(feature = "defmt")]
        defmt::info!("ENET: initialized");

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

        // Configure MAC: 100Mbps, Full duplex, RMII
        regs.maccfg().modify(|w| {
            w.set_ps(true);   // Port Select: 10/100Mbps
            w.set_fes(true);  // Speed: 100Mbps
            w.set_dm(true);   // Full duplex
        });

        // Configure frame filter: promiscuous mode
        regs.macff().modify(|w| {
            w.set_ra(true);   // Receive all
            w.set_pm(true);   // Pass multicast
            w.set_pr(true);   // Promiscuous mode
        });

        // Configure flow control (disabled)
        regs.flowctrl().write(|w| {
            w.set_pt(0);
            w.set_dzpq(true);
            w.set_plt(0);
            w.set_up(false);
            w.set_rfe(false);
            w.set_tfe(false);
            w.set_fcb_bpa(false);
        });
    }

    fn init_dma_no_start<const TX_COUNT: usize, const RX_COUNT: usize>(
        regs: pac::enet::Enet,
        queue: &'d mut PacketQueue<TX_COUNT, RX_COUNT>,
        _pbl: u8,
    ) -> (TxRing<'d>, RxRing<'d>) {
        // Wait for clock to stabilize before DMA reset
        for _ in 0..1_000_000 {
            core::hint::spin_loop();
        }

        // DMA software reset
        regs.dma_bus_mode().modify(|w| w.set_swr(true));
        let mut timeout = 10_000_000u32;
        while regs.dma_bus_mode().read().swr() {
            timeout -= 1;
            if timeout == 0 {
                #[cfg(feature = "defmt")]
                defmt::warn!("ENET: DMA reset timeout - check RMII clock");
                break;
            }
        }

        // Configure DMA bus mode (8-word descriptors)
        regs.dma_bus_mode().modify(|w| {
            w.set_atds(true);   // 8-word descriptors
            w.set_pblx8(true);  // PBL x 8 mode
            w.set_aal(true);    // Address aligned beats
        });

        // Configure AXI burst length
        regs.dma_axi_mode().modify(|w| {
            w.set_blen4(true);
            w.set_blen8(true);
            w.set_blen16(true);
        });

        let tx_desc_base = queue.tx_desc.as_ptr();
        let rx_desc_base = queue.rx_desc.as_ptr();
        let rx_buf_base = queue.rx_buf.as_ptr();

        // Initialize TX descriptors
        for i in 0..TX_COUNT {
            let desc = &mut queue.tx_desc[i];
            desc.init();
            desc.set_buffer1(queue.tx_buf[i].as_ptr() as u32);
            let next_idx = (i + 1) % TX_COUNT;
            desc.set_buffer2(unsafe { tx_desc_base.add(next_idx) as u32 });
            // Set TCH bit (chain mode)
            unsafe {
                let tdes0_ptr = core::ptr::addr_of!(queue.tx_desc[i]) as *mut u32;
                core::ptr::write_volatile(tdes0_ptr, 0x0010_0000);
            }
        }

        // Initialize RX descriptors (direct writes to avoid cache issues)
        for i in 0..RX_COUNT {
            let buf_addr = unsafe { rx_buf_base.add(i) as u32 };
            let next_idx = (i + 1) % RX_COUNT;
            let next_desc_addr = unsafe { rx_desc_base.add(next_idx) as u32 };
            unsafe {
                let desc_ptr = rx_desc_base.add(i) as *mut u32;
                core::ptr::write_volatile(desc_ptr, 0x8000_0000); // OWN
                core::ptr::write_volatile(desc_ptr.add(1), 0x4000 | (RX_BUFFER_SIZE as u32)); // RCH + size
                core::ptr::write_volatile(desc_ptr.add(2), buf_addr);
                core::ptr::write_volatile(desc_ptr.add(3), next_desc_addr);
            }
        }

        // Set descriptor list addresses
        regs.dma_tx_desc_list_addr().write(|w| w.0 = tx_desc_base as u32);
        regs.dma_rx_desc_list_addr().write(|w| w.0 = rx_desc_base as u32);

        // Configure DMA operation mode
        regs.dma_op_mode().modify(|w| {
            w.set_rsf(true);  // Receive store and forward
            w.set_tsf(true);  // Transmit store and forward
            w.set_efc(true);  // Enable flow control
            w.set_fef(true);  // Forward error frames
        });

        // Enable DMA interrupts
        regs.dma_intr_en().write(|w| {
            w.set_nie(true);  // Normal interrupt summary
            w.set_aie(true);  // Abnormal interrupt summary
            w.set_rie(true);  // Receive interrupt
            w.set_tie(true);  // Transmit interrupt
        });

        // Wait for AXI bus idle
        let mut timeout = 10_000u32;
        while regs.dma_bus_status().read().axirdsts() || regs.dma_bus_status().read().axwhsts() {
            timeout -= 1;
            if timeout == 0 {
                break;
            }
        }

        // Flush D-cache for descriptors
        let tx_desc_total_size = (TX_COUNT * 32) as u32;
        let rx_desc_total_size = (RX_COUNT * 32) as u32;
        flush_dcache(tx_desc_base as u32, tx_desc_total_size);
        flush_dcache(rx_desc_base as u32, rx_desc_total_size);
        unsafe { core::arch::asm!("fence rw, rw") };

        // Flush TX FIFO
        regs.dma_op_mode().modify(|w| w.set_ftf(true));
        let mut timeout = 10_000u32;
        while regs.dma_op_mode().read().ftf() {
            timeout -= 1;
            if timeout == 0 {
                break;
            }
        }

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
        const PHY_INF_SEL_MASK: u32 = 0xE000; // bits 15:13
        const PHY_INF_SEL_SHIFT: u32 = 13;
        const RMII_TXCLK_SEL: u32 = 1 << 10;
        const REFCLK_OE: u32 = 1 << 19;

        regs.ctrl2().modify(|w| {
            w.0 = (w.0 & !PHY_INF_SEL_MASK) | (4 << PHY_INF_SEL_SHIFT); // RMII mode
            w.0 |= RMII_TXCLK_SEL;
            if internal_refclk {
                w.0 |= REFCLK_OE;
            } else {
                w.0 &= !REFCLK_OE;
            }
        });
    }

    #[cfg(not(hpm63))]
    fn configure_rmii(_regs: pac::enet::Enet, _internal_refclk: bool) {
        // Other chip variants may have different RMII configuration
    }

    /// Configure ETH clock for HPM6300 series (PLL2CLK1 / 9 ≈ 50MHz)
    #[cfg(hpm63)]
    fn configure_eth_clock_hpm63() {
        use crate::pac::SYSCTL;
        
        SYSCTL.clock(crate::pac::clocks::ETH0).modify(|w| {
            w.set_mux(crate::pac::sysctl::vals::ClockMux::PLL2CLK1);
            w.set_div(8); // /9 ≈ 50MHz
        });
        
        while SYSCTL.clock(crate::pac::clocks::ETH0).read().loc_busy() {
            core::hint::spin_loop();
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

        // Prepare descriptor for transmission
        let buf_addr = self.buffers[self.index].as_ptr() as u32;
        desc.prepare_tx(buf_addr, len as u16);

        // Flush D-cache for TX buffer and descriptor
        flush_dcache(buf_addr, len as u32);
        flush_dcache(desc_addr, 32);
        fence(Ordering::SeqCst);

        // Restart TX DMA if stopped
        let status = self.regs.dma_status().read();
        let ts = (status.0 >> 20) & 0x7;
        if ts == 0 {
            if status.fbi() {
                self.regs.dma_status().write(|w| w.0 = 0xFFFF_FFFF);
            }
            if !self.regs.dma_op_mode().read().st() {
                self.regs.dma_op_mode().modify(|w| w.set_st(true));
            }
        }

        // Clear TU and poll transmit demand
        self.regs.dma_status().write(|w| w.0 = 1 << 2);
        self.regs.dma_tx_poll_demand().write(|w| w.0 = 1);

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
            invalidate_dcache(desc_addr, 32);
            
            let rdes0 = unsafe { core::ptr::read_volatile(desc_addr as *const u32) };
            let owned = (rdes0 & (1 << 31)) != 0;
            let has_error = (rdes0 & (1 << 15)) != 0;
            
            if owned {
                return false;
            }
            
            if has_error {
                // Skip error frames
                self.release();
                continue;
            }
            
            return true;
        }
    }

    fn receive(&mut self) -> Option<(&[u8], usize)> {
        let desc = &self.descriptors[self.index];
        let desc_addr = desc as *const _ as u32;
        invalidate_dcache(desc_addr, 32);

        let rdes0 = unsafe { core::ptr::read_volatile(desc_addr as *const u32) };
        
        if rdes0 & (1 << 31) != 0 {
            return None; // Still owned by DMA
        }

        if rdes0 & (1 << 15) != 0 {
            self.release(); // Error frame
            return None;
        }

        let len = ((rdes0 >> 16) & 0x3FFF) as usize;
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
        let buf = self.tx.buffer().expect("TxToken consumed but no buffer available");
        let result = f(&mut buf[..len]);
        self.tx.transmit(len).ok();
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
