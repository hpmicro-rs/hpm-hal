//! DMA v1, 8 input channels
//!
//! hpm67

use core::future::Future;
use core::pin::Pin;
use core::sync::atomic::{fence, AtomicUsize, Ordering};
use core::task::{Context, Poll};

use embassy_hal_internal::{into_ref, Peripheral, PeripheralRef};
use embassy_sync::waitqueue::AtomicWaker;

use super::word::{Word, WordSize};
use super::{AnyChannel, Channel, Dir, Request, STATE};
use crate::internal::BitIter;
use crate::interrupt::typelevel::Interrupt;
use crate::interrupt::InterruptExt;
use crate::pac;
use crate::pac::dma::vals::{self, AddrCtrl};

/// DMA transfer options.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct TransferOptions {
    pub burst: Burst,
    // /// Enable handshake, transfer by peripheral request
    // pub handshake: bool,
    /// Is high priority
    pub priority: bool,
    /// Circular transfer mode, aka. loop mode(INFINITELOOP)
    pub circular: bool,
    // No half transfer interrupt
    // /// Enable half transfer interrupt
    // pub half_transfer_irq: bool,
    /// Enable transfer complete interrupt
    pub complete_transfer_irq: bool,
}

impl Default for TransferOptions {
    fn default() -> Self {
        Self {
            burst: Burst::Exponential(0),
            priority: false,
            circular: false,
            // half_transfer_irq: false,
            complete_transfer_irq: true,
        }
    }
}

/// DMA transfer burst setting.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Burst {
    /*
    0x0: 1 transfer
    0x1: 2 transfers
    0x2: 4 transfers
    0x3: 8 transfers
    0x4: 16 transfers
    0x5: 32 transfers
    0x6: 64 transfers
    0x7: 128 transfers
    0x8: 256 transfers
    0x9:512 transfers
    0xa: 1024 transfers
    */
    Exponential(u8),
}

impl Burst {
    pub fn from_size(n: usize) -> Self {
        let mut i = 0;
        let mut n = n;
        while n > 1 {
            n >>= 1;
            i += 1;
        }
        Self::Exponential(i as u8)
    }

    pub fn to_size(&self) -> usize {
        match self {
            Self::Exponential(n) => 1 << *n,
        }
    }

    // srcburstsize
    fn burstsize(&self) -> u8 {
        match self {
            Self::Exponential(n) => *n,
        }
    }
}

pub(crate) struct ChannelState {
    waker: AtomicWaker,
    complete_count: AtomicUsize,
}

impl ChannelState {
    pub(crate) const NEW: Self = Self {
        waker: AtomicWaker::new(),
        complete_count: AtomicUsize::new(0),
    };
}

pub(crate) unsafe fn init(cs: critical_section::CriticalSection) {
    use crate::interrupt;
    use crate::sysctl::SealedClockPeripheral;

    crate::peripherals::HDMA::add_resource_group(0);

    pac::HDMA.dmactrl().modify(|w| w.set_reset(true));

    interrupt::typelevel::HDMA::set_priority_with_cs(cs, interrupt::Priority::P1);
    interrupt::typelevel::HDMA::enable();

    #[cfg(peri_xdma)]
    {
        crate::peripherals::XDMA::add_resource_group(0);

        pac::XDMA.dmactrl().modify(|w| w.set_reset(true));

        interrupt::typelevel::XDMA::set_priority_with_cs(cs, interrupt::Priority::P1);
        interrupt::typelevel::XDMA::enable();
    }
}

impl super::ControllerInterrupt for crate::peripherals::HDMA {
    unsafe fn on_irq() {
        dma_on_irq(pac::HDMA, 0);

        crate::interrupt::HDMA.complete(); // notify PLIC
    }
}

impl super::ControllerInterrupt for crate::peripherals::XDMA {
    unsafe fn on_irq() {
        dma_on_irq(pac::XDMA, 8);

        crate::interrupt::XDMA.complete(); // notify PLIC
    }
}

unsafe fn dma_on_irq(r: pac::dma::Dma, mux_num_base: u32) {
    let intstatus = r.int_status().read().0;

    let err = intstatus & 0xFF;
    let abort = (intstatus >> 8) & 0xFF;
    let tc = (intstatus >> 16) & 0xFF;

    // possible errors:
    // - bus error
    // - address alignment error
    // - bit width alignment error
    // DMA error: this is normally a hardware error(memory alignment or access), but we can't do anything about it
    if err != 0 {
        panic!("DMA: error on DMA@{:08x}, errsts=0b{:08b}", r.as_ptr() as u32, err);
    }

    if tc != 0 {
        r.int_status().write(|w| w.0 = tc << 16); // W1C
    }
    if abort != 0 {
        r.int_status().write(|w| w.0 = abort << 8); // W1C
    }

    for i in BitIter(tc | abort) {
        let id = (i + mux_num_base) as usize;
        STATE[id].waker.wake();
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum HandshakeMode {
    /// Software enable contro
    Normal,
    /// Source DMAMUX request
    Source,
    /// Destination DMAMUX request
    Destination,
}

impl HandshakeMode {
    fn src_mode(self) -> vals::Mode {
        match self {
            Self::Normal => vals::Mode::NORMAL,
            Self::Source => vals::Mode::HANDSHAKE,
            Self::Destination => vals::Mode::NORMAL,
        }
    }

    fn dst_mode(self) -> vals::Mode {
        match self {
            Self::Normal => vals::Mode::NORMAL,
            Self::Source => vals::Mode::NORMAL,
            Self::Destination => vals::Mode::HANDSHAKE,
        }
    }
}

impl AnyChannel {
    unsafe fn configure(
        &self,
        request: Request, // DMA request number in DMAMUX
        dir: Dir,
        src_addr: *const u32,
        src_width: WordSize,
        src_addr_ctrl: AddrCtrl,
        dst_addr: *mut u32,
        dst_width: WordSize,
        dst_addr_ctrl: AddrCtrl,
        // TRANSIZE
        size_in_words: usize,
        handshake: HandshakeMode,
        options: TransferOptions,
    ) {
        let info = self.info();

        let r = info.dma.regs();
        let ch = info.num; // channel number in current dma controller
        let mux_ch = info.mux_num; // channel number in dma mux, (XDMA_CH0 = HDMA_CH7+1 = 8)

        // follow the impl of dma_setup_channel

        // check alignment
        if !dst_width.aligned((size_in_words as u32) * (dst_width.bytes() as u32))
            || !src_width.aligned(src_addr as u32)
            || !dst_width.aligned(dst_addr as u32)
        {
            panic!("DMA address not aligned");
        }

        // TODO: check transfer width against DMA controller

        let ch_cr = r.chctrl(ch);

        let mut src_addr = src_addr as u32;
        let mut dst_addr = dst_addr as u32;

        #[cfg(hpm67)]
        {
            let core_id = 0;
            src_addr = core_local_mem_to_sys_address(core_id, src_addr);
            dst_addr = core_local_mem_to_sys_address(core_id, dst_addr);
        }

        // configure DMAMUX request and output channel
        super::dmamux::configure_dmamux(info.mux_num, request);

        ch_cr.src_addr().write_value(src_addr);
        ch_cr.dst_addr().write_value(dst_addr);
        ch_cr.tran_size().modify(|w| w.0 = size_in_words as u32);
        ch_cr.llpointer().modify(|w| w.0 = 0x0);

        // TODO: LLPointer handling

        self.clear_irqs();

        ch_cr.ctrl().write(|w| {
            w.set_srcbusinfidx(false);
            w.set_dstbusinfidx(false);

            w.set_priority(options.priority);
            w.set_srcburstsize(options.burst.burstsize());
            w.set_srcwidth(src_width.width());
            w.set_dstwidth(dst_width.width());
            w.set_srcmode(handshake.src_mode());
            w.set_dstmode(handshake.dst_mode());
            w.set_srcaddrctrl(src_addr_ctrl);
            w.set_dstaddrctrl(dst_addr_ctrl);

            if dir == Dir::MemoryToPeripheral {
                w.set_dstreqsel(mux_ch as u8);
            } else {
                w.set_srcreqsel(mux_ch as u8);
            }
            // unmask interrupts
            w.set_inttcmask(!options.complete_transfer_irq);
            w.set_interrmask(false);
            w.set_intabtmask(true); // handled via blocking

            w.set_enable(false); // don't start yet
        });
    }

    fn start(&self) {
        let info = self.info();
        let r = info.dma.regs();
        let ch = info.num; // channel number in current dma controller

        let ch_cr = r.chctrl(ch);

        ch_cr.ctrl().modify(|w| w.set_enable(true));
    }

    fn clear_irqs(&self) {
        let info = self.info();

        let r = info.dma.regs();
        let ch = info.num; // channel number in current dma controller

        // clear transfer irq status (W1C)
        r.int_status().write(|w| {
            w.set_abort(ch, true);
            w.set_tc(ch, true);
            w.set_error(ch, true);
        });
    }

    // requrest stop
    fn abort(&self) {
        let r = self.info().dma.regs();

        r.ch_abort().write(|w| w.set_chabort(self.info().num, true));
    }

    fn is_running(&self) -> bool {
        let r = self.info().dma.regs();
        let num = self.info().num;
        let ch_cr = r.chctrl(num);

        // enabled, not aborted
        ch_cr.ctrl().read().enable() && !r.int_status().read().abort(num) && (!r.int_status().read().tc(num))
    }

    fn get_remaining_transfers(&self) -> u32 {
        let r = self.info().dma.regs();
        let num = self.info().num;
        let ch_cr = r.chctrl(num);

        ch_cr.tran_size().read().transize()
    }

    fn disable_circular_mode(&self) {
        let r = self.info().dma.regs();
        let num = self.info().num;
        let _ch_cr = r.chctrl(num);

        todo!("disable circular mode");
    }

    fn poll_stop(&self) -> Poll<()> {
        use core::sync::atomic::compiler_fence;
        compiler_fence(Ordering::SeqCst);

        if self.is_running() {
            Poll::Pending
        } else {
            Poll::Ready(())
        }
    }
}

/// DMA transfer.
#[must_use = "futures do nothing unless you `.await` or poll them"]
pub struct Transfer<'a> {
    channel: PeripheralRef<'a, AnyChannel>,
}

impl<'a> Transfer<'a> {
    /// Create a new read DMA transfer (peripheral to memory).
    pub unsafe fn new_read<W: Word>(
        channel: impl Peripheral<P = impl Channel> + 'a,
        request: Request,
        peri_addr: *mut W,
        buf: &'a mut [W],
        options: TransferOptions,
    ) -> Self {
        Self::new_read_raw(channel, request, peri_addr, buf, options)
    }

    /// Create a new read DMA transfer (peripheral to memory), using raw pointers.
    pub unsafe fn new_read_raw<W: Word>(
        channel: impl Peripheral<P = impl Channel> + 'a,
        request: Request,
        peri_addr: *mut W,
        buf: *mut [W],
        options: TransferOptions,
    ) -> Self {
        into_ref!(channel);

        Self::new_inner(
            channel.map_into(),
            request,
            Dir::PeripheralToMemory,
            peri_addr as *const u32,
            buf as *mut W as *mut u32,
            buf.len(),
            true,
            W::size(),
            options,
        )
    }

    /// Create a new write DMA transfer (memory to peripheral).
    pub unsafe fn new_write<W: Word>(
        channel: impl Peripheral<P = impl Channel> + 'a,
        request: Request,
        buf: &'a [W],
        peri_addr: *mut W,
        options: TransferOptions,
    ) -> Self {
        Self::new_write_raw(channel, request, buf, peri_addr, options)
    }

    /// Create a new write DMA transfer (memory to peripheral), using raw pointers.
    pub unsafe fn new_write_raw<W: Word>(
        channel: impl Peripheral<P = impl Channel> + 'a,
        request: Request,
        buf: *const [W],
        peri_addr: *mut W,
        options: TransferOptions,
    ) -> Self {
        into_ref!(channel);

        Self::new_inner(
            channel.map_into(),
            request,
            Dir::MemoryToPeripheral,
            peri_addr as *const u32,
            buf as *const W as *mut u32,
            buf.len(),
            true,
            W::size(),
            options,
        )
    }

    /// Create a new write DMA transfer (memory to peripheral), writing the same value repeatedly.
    pub unsafe fn new_write_repeated<W: Word>(
        channel: impl Peripheral<P = impl Channel> + 'a,
        request: Request,
        repeated: &'a W,
        count: usize,
        peri_addr: *mut W,
        options: TransferOptions,
    ) -> Self {
        into_ref!(channel);

        Self::new_inner(
            channel.map_into(),
            request,
            Dir::MemoryToPeripheral,
            peri_addr as *const u32,
            repeated as *const W as *mut u32,
            count,
            false,
            W::size(),
            options,
        )
    }

    // Restrictions comparing to DMA capabilities:
    // - No AddrCtrl::DECREMENT
    unsafe fn new_inner(
        channel: PeripheralRef<'a, AnyChannel>,
        request: Request,
        dir: Dir,
        peri_addr: *const u32,
        mem_addr: *mut u32,
        mem_len: usize,
        incr_mem: bool,
        data_size: WordSize,
        options: TransferOptions,
    ) -> Self {
        assert!(mem_len > 0);

        let src_addr;
        let dst_addr;
        let mut src_addr_ctrl = AddrCtrl::FIXED;
        let mut dst_addr_ctrl = AddrCtrl::FIXED;
        let handshake;
        match dir {
            Dir::MemoryToPeripheral => {
                src_addr = mem_addr;
                dst_addr = peri_addr as *mut _;
                if incr_mem {
                    src_addr_ctrl = AddrCtrl::INCREMENT;
                }
                handshake = HandshakeMode::Destination; // destination trigger
            }
            Dir::PeripheralToMemory => {
                src_addr = peri_addr as *mut _;
                dst_addr = mem_addr;
                if incr_mem {
                    dst_addr_ctrl = AddrCtrl::INCREMENT;
                }
                handshake = HandshakeMode::Source; // source trigger
            }
        };

        channel.configure(
            request,
            dir,
            src_addr,
            data_size,
            src_addr_ctrl,
            dst_addr,
            data_size,
            dst_addr_ctrl,
            mem_len,
            handshake,
            options,
        );
        channel.start();

        Self { channel }
    }

    /// Request the transfer to stop.
    ///
    /// This doesn't immediately stop the transfer, you have to wait until [`is_running`](Self::is_running) returns false.
    pub fn request_abort(&mut self) {
        self.channel.abort()
    }

    /// Return whether this transfer is still running.
    ///
    /// If this returns `false`, it can be because either the transfer finished, or
    /// it was requested to stop early with [`request_abort`](Self::request_abort).
    pub fn is_running(&mut self) -> bool {
        self.channel.is_running()
    }

    /// Gets the total remaining transfers for the channel
    /// Note: this will be zero for transfers that completed without cancellation.
    pub fn get_remaining_transfers(&self) -> u32 {
        self.channel.get_remaining_transfers()
    }

    /// Blocking wait until the transfer finishes.
    pub fn blocking_wait(mut self) {
        while self.is_running() {}

        // "Subsequent reads and writes cannot be moved ahead of preceding reads."
        fence(Ordering::SeqCst);

        core::mem::forget(self);
    }
}

impl<'a> Drop for Transfer<'a> {
    fn drop(&mut self) {
        self.request_abort();
        while self.is_running() {}

        // "Subsequent reads and writes cannot be moved ahead of preceding reads."
        fence(Ordering::SeqCst);
    }
}

impl<'a> Unpin for Transfer<'a> {}
impl<'a> Future for Transfer<'a> {
    type Output = ();
    fn poll(mut self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        let state: &ChannelState = &STATE[self.channel.id as usize];

        state.waker.register(cx.waker());

        if self.is_running() {
            Poll::Pending
        } else {
            Poll::Ready(())
        }
    }
}

#[cfg(hpm67)]
fn core_local_mem_to_sys_address(core_id: u8, addr: u32) -> u32 {
    // const ILM_LOCAL_BASE: u32 = 0x0;
    const ILM_SIZE_IN_BYTE: u32 = 0x40000;
    const DLM_LOCAL_BASE: u32 = 0x80000;
    const DLM_SIZE_IN_BYTE: u32 = 0x40000;
    const CORE0_ILM_SYSTEM_BASE: u32 = 0x1000000;
    const CORE0_DLM_SYSTEM_BASE: u32 = 0x1040000;
    const CORE1_ILM_SYSTEM_BASE: u32 = 0x1180000;
    // const CORE1_DLM_SYSTEM_BASE: u32 = 0x11C0000;

    let sys_addr = if addr < ILM_SIZE_IN_BYTE {
        // Address in ILM
        addr | CORE0_ILM_SYSTEM_BASE
    } else if (addr >= DLM_LOCAL_BASE) && (addr < DLM_LOCAL_BASE + DLM_SIZE_IN_BYTE) {
        // Address in DLM
        (addr - DLM_LOCAL_BASE) | CORE0_DLM_SYSTEM_BASE
    } else {
        return addr;
    };

    if core_id == 1 {
        sys_addr - CORE0_ILM_SYSTEM_BASE + CORE1_ILM_SYSTEM_BASE
    } else {
        sys_addr
    }
}
