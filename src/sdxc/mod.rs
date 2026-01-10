//! SDXC (SD/MMC Card Interface) driver
//!
//! Embassy-style driver for SD cards on HPMicro MCUs.
//!
//! # Current Status
//!
//! - ✅ Blocking single-block read/write (SDMA mode, errata E00033 workaround)
//! - ✅ FAT32 filesystem via embedded-sdmmc BlockDevice trait
//! - ⚠️ Multi-block read/write: uses single-block loop (PIO mode broken)
//!
//! # TODO (Phase 2/3)
//!
//! - [ ] ADMA2 support for efficient multi-block transfers
//! - [ ] Async mode with interrupt-driven transfers
//! - [ ] High speed modes (SDR50, SDR104) with 1.8V signaling
//! - [ ] Fix CSD v2.0 parsing for SDHC capacity display
//!
//! See `PHASE2_PLAN.md` for implementation details.
//!
//! # Hardware Note
//!
//! Due to errata E00033, PIO/FIFO mode is unreliable. This driver uses
//! SDMA for all data transfers.
//!
//! # Examples
//!
//! ## Blocking mode
//!
//! ```no_run
//! let mut sdxc = sdxc::Sdxc::new_blocking_4bit(
//!     p.SDXC0,
//!     p.PA11, p.PA10, p.PA12, p.PA13, p.PA08, p.PA09,
//!     Default::default()
//! );
//!
//! sdxc.init_sd_card(Hertz::mhz(25))?;
//!
//! let mut block = sdxc::DataBlock::new();
//! sdxc.read_block(0, &mut block)?;
//! ```

use core::marker::PhantomData;
use core::sync::atomic::{AtomicU32, Ordering};

use embassy_hal_internal::Peri;
use embassy_sync::waitqueue::AtomicWaker;
use sdio_host::Cmd;

use crate::gpio::AnyPin;
use crate::interrupt::typelevel::Interrupt as _;
use crate::mode::{Async, Blocking, Mode};
use crate::time::Hertz;
use crate::{interrupt, peripherals};

mod types;

pub use types::*;

/// Frequency used for SD Card initialization. Must be no higher than 400 kHz.
const SD_INIT_FREQ: Hertz = Hertz(400_000);

// State and Info structures (following Embassy pattern)
struct State {
    waker: AtomicWaker,
    last_error: AtomicU32,
}

impl State {
    const fn new() -> Self {
        Self {
            waker: AtomicWaker::new(),
            last_error: AtomicU32::new(0),
        }
    }

    fn wake(&self) {
        self.waker.wake();
    }

    fn set_error(&self, error: u32) {
        self.last_error.store(error, Ordering::Release);
    }

    #[allow(dead_code)]
    fn get_error(&self) -> u32 {
        self.last_error.load(Ordering::Acquire)
    }

    #[allow(dead_code)]
    fn clear_error(&self) {
        self.last_error.store(0, Ordering::Release);
    }
}

struct Info {
    regs: crate::pac::sdxc::Sdxc,
}

// Instance trait using peri_trait! macro
peri_trait!(
    irqs: [Interrupt],
);

// Instance implementations using foreach_peripheral! macro
foreach_peripheral!(
    (sdxc, $inst:ident) => {
        #[allow(private_interfaces)]
        impl SealedInstance for peripherals::$inst {
            fn info() -> &'static Info {
                static INFO: Info = Info {
                    regs: crate::pac::$inst,
                };
                &INFO
            }
            fn state() -> &'static State {
                static STATE: State = State::new();
                &STATE
            }
        }

        impl Instance for peripherals::$inst {
            type Interrupt = crate::interrupt::typelevel::$inst;
        }
    };
);

// Pin traits using pin_trait! macro
pin_trait!(ClkPin, Instance);
pin_trait!(CmdPin, Instance);
pin_trait!(D0Pin, Instance);
pin_trait!(D1Pin, Instance);
pin_trait!(D2Pin, Instance);
pin_trait!(D3Pin, Instance);
pin_trait!(D4Pin, Instance);
pin_trait!(D5Pin, Instance);
pin_trait!(D6Pin, Instance);
pin_trait!(D7Pin, Instance);

/// SDXC interrupt handler
pub struct InterruptHandler<T: Instance> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> interrupt::typelevel::Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        let info = T::info();
        let state = T::state();
        let regs = info.regs;

        let status = regs.int_stat().read();

        // Save error status for async functions to check
        if status.0 & 0xFFFF_8000 != 0 {
            state.set_error(status.0);
        }

        // Disable interrupt signals to prevent re-triggering
        // The async function will re-enable if needed after checking status
        // DO NOT clear int_stat here - let the async function read it first
        regs.int_signal_en().write(|_| {});

        // Wake waiting task - it will read int_stat and clear it
        state.wake();
    }
}

/// Convert types::ResponseLen to hardware RESP_TYPE_SELECT value
fn get_resp_type_select(resp_len: types::ResponseLen) -> u8 {
    match resp_len {
        types::ResponseLen::Zero => 0,
        types::ResponseLen::R136 => 1,
        types::ResponseLen::R48 => 2,
    }
}

/// Check if command requires R1b (busy) response
/// These commands use 48-bit response with busy check (resp_type_select = 3)
fn needs_busy_response(cmd_index: u8) -> bool {
    matches!(
        cmd_index,
        7 |  // SELECT_CARD
        12 | // STOP_TRANSMISSION
        28 | // SET_WRITE_PROT
        29 | // CLR_WRITE_PROT
        38   // ERASE
    )
}

/// SDXC driver
pub struct Sdxc<'d, M: Mode> {
    info: &'static Info,
    #[allow(dead_code)]
    state: &'static State,
    kernel_clock: Hertz,
    clock: Hertz,

    _clk: Peri<'d, AnyPin>,
    _cmd: Peri<'d, AnyPin>,
    _d0: Peri<'d, AnyPin>,
    _d1: Option<Peri<'d, AnyPin>>,
    _d2: Option<Peri<'d, AnyPin>>,
    _d3: Option<Peri<'d, AnyPin>>,
    _d4: Option<Peri<'d, AnyPin>>,
    _d5: Option<Peri<'d, AnyPin>>,
    _d6: Option<Peri<'d, AnyPin>>,
    _d7: Option<Peri<'d, AnyPin>>,

    card: Option<Card>,
    config: Config,

    _phantom: PhantomData<M>,
}

impl<'d, M: Mode> Sdxc<'d, M> {
    /// Internal constructor - pins must already be configured
    fn new_inner<T: Instance>(
        _peri: Peri<'d, T>,
        clk: Peri<'d, AnyPin>,
        cmd: Peri<'d, AnyPin>,
        d0: Peri<'d, AnyPin>,
        d1: Option<Peri<'d, AnyPin>>,
        d2: Option<Peri<'d, AnyPin>>,
        d3: Option<Peri<'d, AnyPin>>,
        d4: Option<Peri<'d, AnyPin>>,
        d5: Option<Peri<'d, AnyPin>>,
        d6: Option<Peri<'d, AnyPin>>,
        d7: Option<Peri<'d, AnyPin>>,
        config: Config,
    ) -> Self {
        // Add peripheral to resource group (enables clock)
        T::add_resource_group(0);

        let info = T::info();
        let state = T::state();
        let regs = info.regs;

        // HPM6360 specific: Configure MISC_CTRL0 first
        regs.misc_ctrl0().modify(|w| w.set_cardclk_inv_en(false));

        // Disable SD clock during configuration
        regs.sys_ctrl().modify(|w| w.set_sd_clk_en(false));

        // Reset controller
        regs.sys_ctrl().modify(|w| w.set_sw_rst_all(true));
        while regs.sys_ctrl().read().sw_rst_all() {}

        // Enable timeout clock (critical for HPM6360)
        regs.misc_ctrl0().modify(|w| w.set_tmclk_en(true));

        // Configure timeout
        regs.sys_ctrl().modify(|w| w.set_tout_cnt(0x0E));

        // Configure power (3.3V)
        regs.prot_ctrl().modify(|w| {
            w.set_sd_bus_vol_vdd1(7); // 3.3V
            w.set_sd_bus_pwr_vdd1(true);
        });

        // Enable interrupt status reporting
        regs.int_stat_en().write(|w| w.0 = 0xFFFFFFFF);
        regs.int_signal_en().write(|w| w.0 = 0);
        regs.int_stat().write(|w| w.0 = 0xFFFFFFFF);

        let kernel_clock = T::frequency();

        Self {
            info,
            state,
            kernel_clock,
            clock: Hertz(0),
            _clk: clk,
            _cmd: cmd,
            _d0: d0,
            _d1: d1,
            _d2: d2,
            _d3: d3,
            _d4: d4,
            _d5: d5,
            _d6: d6,
            _d7: d7,
            card: None,
            config,
            _phantom: PhantomData,
        }
    }

    /// Get the card information, if initialized
    pub fn card(&self) -> Option<&Card> {
        self.card.as_ref()
    }

    /// Check if a card is inserted
    pub fn is_card_inserted(&self) -> bool {
        self.info.regs.pstate().read().card_inserted()
    }

    /// Get current clock frequency
    pub fn clock(&self) -> Hertz {
        self.clock
    }

    /// Set clock frequency
    pub fn set_clock(&mut self, freq: Hertz) {
        let regs = self.info.regs;

        // Calculate divider
        let divider = (self.kernel_clock.0 / freq.0).max(1) - 1;
        let divider = divider.min(1023) as u16;

        // Disable SD clock
        regs.sys_ctrl().modify(|w| w.set_sd_clk_en(false));

        // Set divider using MISC_CTRL0 (HPM specific)
        regs.misc_ctrl0().modify(|w| {
            w.set_freq_sel_sw(divider);
            w.set_freq_sel_sw_en(true);
        });

        // Enable internal clock and wait for stable
        regs.sys_ctrl().modify(|w| w.set_internal_clk_en(true));
        while !regs.sys_ctrl().read().internal_clk_stable() {}

        // Enable PLL and wait for stable again (required per C SDK)
        regs.sys_ctrl().modify(|w| w.set_pll_enable(true));
        while !regs.sys_ctrl().read().internal_clk_stable() {}

        // Enable SD clock
        regs.sys_ctrl().modify(|w| w.set_sd_clk_en(true));

        // Calculate actual clock
        self.clock = Hertz(self.kernel_clock.0 / (divider as u32 + 1));
    }

    /// Send 74+ clock cycles to activate card
    pub fn wait_card_active(&self) {
        let regs = self.info.regs;
        regs.misc_ctrl1().modify(|w| w.set_card_active(true));
        while regs.misc_ctrl1().read().card_active() {}
    }

    /// Wait for card to be ready (not in programming state)
    ///
    /// Uses CMD13 (SEND_STATUS) to check card state.
    fn wait_card_ready(&self) -> Result<(), Error> {
        let rca = self.card.as_ref().ok_or(Error::NoCard)?.rca;

        for _ in 0..10000 {
            // CMD13: SEND_STATUS
            self.cmd(types::common_cmd::card_status(rca, false), false)?;
            let status = types::CardStatus::<types::SD>::from(self.get_response());

            // Check if card is ready (not in programming state)
            if status.ready_for_data() {
                return Ok(());
            }

            // Small delay
            for _ in 0..1000 {
                core::hint::spin_loop();
            }
        }

        Err(Error::SoftwareTimeout)
    }

    /// Send a command and wait for response (blocking)
    fn cmd<R: Resp>(&self, cmd: Cmd<R>, data: bool) -> Result<(), Error> {
        let regs = self.info.regs;

        // Clear interrupt status
        regs.int_stat().write(|w| w.0 = 0xFFFFFFFF);

        // Wait for CMD line to be free
        while regs.pstate().read().cmd_inhibit() {}

        if data {
            while regs.pstate().read().dat_inhibit() {}
        }

        // Set command argument
        regs.cmd_arg().write(|w| w.0 = cmd.arg);

        // Build CMD_XFER
        let resp_len = cmd.response_len();

        // Determine response type: use R1b (3) for commands that need busy check
        let resp_type_sel = if needs_busy_response(cmd.cmd) {
            3 // R1b: 48-bit with busy check
        } else {
            get_resp_type_select(resp_len)
        };

        // R3 (OCR) has no CRC
        let (crc_check, idx_check) = match resp_len {
            types::ResponseLen::Zero => (false, false),
            types::ResponseLen::R136 => (true, false),
            types::ResponseLen::R48 => {
                if cmd.cmd == 41 {
                    (false, false) // R3 - no CRC
                } else {
                    (true, true)
                }
            }
        };

        regs.cmd_xfer().write(|w| {
            w.set_cmd_index(cmd.cmd);
            w.set_resp_type_select(resp_type_sel);
            w.set_cmd_crc_chk_enable(crc_check);
            w.set_cmd_idx_chk_enable(idx_check);
            w.set_data_present_sel(data);
            if data {
                w.set_data_xfer_dir(true);
            }
        });

        // Wait for completion with timeout counter
        let mut timeout_count = 0u32;
        loop {
            let status = regs.int_stat().read();

            // Check for success FIRST - if command completed, consider it successful
            // even if error bits are set (they may be from previous operations)
            if status.cmd_complete() {
                regs.int_stat().write(|w| w.set_cmd_complete(true));
                return Ok(());
            }

            // Only check errors if command hasn't completed
            if status.cmd_tout_err() {
                return Err(Error::Timeout);
            }
            if status.cmd_crc_err() {
                return Err(Error::Crc);
            }
            if status.cmd_end_bit_err() {
                return Err(Error::CmdEndBit);
            }
            if status.cmd_idx_err() {
                return Err(Error::CmdIndex);
            }

            // Safety timeout to prevent infinite loop
            timeout_count += 1;
            if timeout_count > 10_000_000 {
                // Software timeout - hardware didn't respond
                return Err(Error::SoftwareTimeout);
            }
        }
    }

    /// Send a data read command with DMA enabled (blocking) - for single block SDMA
    fn cmd_dma<R: Resp>(&self, cmd: Cmd<R>) -> Result<(), Error> {
        let regs = self.info.regs;

        // Clear interrupt status
        regs.int_stat().write(|w| w.0 = 0xFFFFFFFF);

        // Wait for CMD and DAT lines to be free
        while regs.pstate().read().cmd_inhibit() {}
        while regs.pstate().read().dat_inhibit() {}

        // Set command argument
        regs.cmd_arg().write(|w| w.0 = cmd.arg);

        // Build CMD_XFER with DMA enabled
        let resp_len = cmd.response_len();
        let resp_type_sel = get_resp_type_select(resp_len);

        regs.cmd_xfer().write(|w| {
            w.set_cmd_index(cmd.cmd);
            w.set_resp_type_select(resp_type_sel);
            w.set_cmd_crc_chk_enable(true);
            w.set_cmd_idx_chk_enable(true);
            w.set_data_present_sel(true);
            w.set_data_xfer_dir(true); // Read
            w.set_dma_enable(true); // Enable DMA
        });

        // Wait for command complete
        let mut timeout_count = 0u32;
        loop {
            let status = regs.int_stat().read();

            if status.cmd_complete() {
                regs.int_stat().write(|w| w.set_cmd_complete(true));
                return Ok(());
            }

            if status.cmd_tout_err() {
                return Err(Error::Timeout);
            }
            if status.cmd_crc_err() {
                return Err(Error::Crc);
            }
            if status.cmd_end_bit_err() {
                return Err(Error::CmdEndBit);
            }
            if status.cmd_idx_err() {
                return Err(Error::CmdIndex);
            }

            timeout_count += 1;
            if timeout_count > 10_000_000 {
                return Err(Error::SoftwareTimeout);
            }
        }
    }

    /// Send a data write command with DMA enabled (blocking) - for single block SDMA
    fn cmd_dma_write<R: Resp>(&self, cmd: Cmd<R>) -> Result<(), Error> {
        let regs = self.info.regs;

        // Clear interrupt status
        regs.int_stat().write(|w| w.0 = 0xFFFFFFFF);

        // Wait for CMD and DAT lines to be free
        while regs.pstate().read().cmd_inhibit() {}
        while regs.pstate().read().dat_inhibit() {}

        // Set command argument
        regs.cmd_arg().write(|w| w.0 = cmd.arg);

        // Build CMD_XFER with DMA enabled
        let resp_len = cmd.response_len();
        let resp_type_sel = get_resp_type_select(resp_len);

        regs.cmd_xfer().write(|w| {
            w.set_cmd_index(cmd.cmd);
            w.set_resp_type_select(resp_type_sel);
            w.set_cmd_crc_chk_enable(true);
            w.set_cmd_idx_chk_enable(true);
            w.set_data_present_sel(true);
            w.set_data_xfer_dir(false); // Write
            w.set_dma_enable(true); // Enable DMA
        });

        // Wait for command complete
        let mut timeout_count = 0u32;
        loop {
            let status = regs.int_stat().read();

            if status.cmd_complete() {
                regs.int_stat().write(|w| w.set_cmd_complete(true));
                return Ok(());
            }

            if status.cmd_tout_err() {
                return Err(Error::Timeout);
            }
            if status.cmd_crc_err() {
                return Err(Error::Crc);
            }
            if status.cmd_end_bit_err() {
                return Err(Error::CmdEndBit);
            }
            if status.cmd_idx_err() {
                return Err(Error::CmdIndex);
            }

            timeout_count += 1;
            if timeout_count > 10_000_000 {
                return Err(Error::SoftwareTimeout);
            }
        }
    }

    /// Send a multi-block data read command with ADMA2 enabled (blocking)
    fn cmd_adma2_multi_read<R: Resp>(&self, cmd: Cmd<R>) -> Result<(), Error> {
        let regs = self.info.regs;

        // Wait for CMD and DAT lines to be free
        while regs.pstate().read().cmd_inhibit() {}
        while regs.pstate().read().dat_inhibit() {}

        // Set command argument
        regs.cmd_arg().write(|w| w.0 = cmd.arg);

        // Build CMD_XFER with DMA enabled for multi-block
        let resp_len = cmd.response_len();
        let resp_type_sel = get_resp_type_select(resp_len);

        regs.cmd_xfer().write(|w| {
            w.set_cmd_index(cmd.cmd);
            w.set_resp_type_select(resp_type_sel);
            w.set_cmd_crc_chk_enable(true);
            w.set_cmd_idx_chk_enable(true);
            w.set_data_present_sel(true);
            w.set_data_xfer_dir(true); // Read
            w.set_dma_enable(true);
            w.set_multi_blk_sel(true);        // Multi-block transfer
            w.set_block_count_enable(true);   // Enable block count (per HPM SDK)
            w.set_auto_cmd_enable(1);         // Auto CMD12 after transfer
        });

        // Wait for command complete
        let mut timeout_count = 0u32;
        loop {
            let status = regs.int_stat().read();

            if status.cmd_complete() {
                regs.int_stat().write(|w| w.set_cmd_complete(true));
                return Ok(());
            }

            if status.cmd_tout_err() {
                return Err(Error::Timeout);
            }
            if status.cmd_crc_err() {
                return Err(Error::Crc);
            }
            if status.cmd_end_bit_err() {
                return Err(Error::CmdEndBit);
            }
            if status.cmd_idx_err() {
                return Err(Error::CmdIndex);
            }

            timeout_count += 1;
            if timeout_count > 10_000_000 {
                return Err(Error::SoftwareTimeout);
            }
        }
    }

    /// Send a multi-block data write command with ADMA2 enabled (blocking)
    fn cmd_adma2_multi_write<R: Resp>(&self, cmd: Cmd<R>) -> Result<(), Error> {
        let regs = self.info.regs;

        // Wait for CMD and DAT lines to be free
        while regs.pstate().read().cmd_inhibit() {}
        while regs.pstate().read().dat_inhibit() {}

        // Set command argument
        regs.cmd_arg().write(|w| w.0 = cmd.arg);

        // Build CMD_XFER with DMA enabled for multi-block
        let resp_len = cmd.response_len();
        let resp_type_sel = get_resp_type_select(resp_len);

        regs.cmd_xfer().write(|w| {
            w.set_cmd_index(cmd.cmd);
            w.set_resp_type_select(resp_type_sel);
            w.set_cmd_crc_chk_enable(true);
            w.set_cmd_idx_chk_enable(true);
            w.set_data_present_sel(true);
            w.set_data_xfer_dir(false); // Write
            w.set_dma_enable(true);
            w.set_multi_blk_sel(true);        // Multi-block transfer
            w.set_block_count_enable(true);   // Enable block count (per HPM SDK)
            w.set_auto_cmd_enable(1);         // Auto CMD12 after transfer
        });

        // Wait for command complete
        let mut timeout_count = 0u32;
        loop {
            let status = regs.int_stat().read();

            if status.cmd_complete() {
                regs.int_stat().write(|w| w.set_cmd_complete(true));
                return Ok(());
            }

            if status.cmd_tout_err() {
                return Err(Error::Timeout);
            }
            if status.cmd_crc_err() {
                return Err(Error::Crc);
            }
            if status.cmd_end_bit_err() {
                return Err(Error::CmdEndBit);
            }
            if status.cmd_idx_err() {
                return Err(Error::CmdIndex);
            }

            timeout_count += 1;
            if timeout_count > 10_000_000 {
                return Err(Error::SoftwareTimeout);
            }
        }
    }

    /// Get 48-bit response
    fn get_response(&self) -> u32 {
        self.info.regs.resp(0).read().0
    }

    /// Get 136-bit response (R2)
    ///
    /// Note: The SD Host Controller stores R2 response right-shifted by 8 bits
    /// (CRC7 + end bit removed). We shift left to restore original bit positions
    /// for sdio-host CID/CSD parsing.
    fn get_response_r2(&self) -> [u32; 4] {
        let regs = self.info.regs;
        let r0 = regs.resp(0).read().0;
        let r1 = regs.resp(1).read().0;
        let r2 = regs.resp(2).read().0;
        let r3 = regs.resp(3).read().0;

        // Shift left by 8 bits to restore original CID/CSD bit positions
        [
            r0 << 8,
            (r1 << 8) | (r0 >> 24),
            (r2 << 8) | (r1 >> 24),
            (r3 << 8) | (r2 >> 24),
        ]
    }

    /// Set bus width
    fn set_bus_width(&self, width: BusWidth) {
        let regs = self.info.regs;
        regs.prot_ctrl().modify(|w| {
            match width {
                BusWidth::One => {
                    w.set_dat_xfer_width(false);
                    w.set_ext_dat_xfer(false);
                }
                BusWidth::Four => {
                    w.set_dat_xfer_width(true);
                    w.set_ext_dat_xfer(false);
                }
                BusWidth::Eight => {
                    w.set_dat_xfer_width(false);
                    w.set_ext_dat_xfer(true);
                }
            }
        });
    }

    /// Initialize an SD card
    ///
    /// This performs the SD card identification and initialization sequence.
    /// After calling this method, the card is ready for data transfers.
    pub fn init_sd_card(&mut self, freq: Hertz) -> Result<(), Error> {
        let regs = self.info.regs;

        // Clear all interrupt status
        regs.int_stat().write(|w| w.0 = 0xFFFFFFFF);

        // Enable Host Version 4 mode and ADMA2 26-bit length mode (required for ADMA2)
        regs.ac_host_ctrl().modify(|w| {
            w.set_host_ver4_enable(true);
            w.set_adma2_len_mode(true);
        });

        // Set 400kHz for identification
        self.set_clock(SD_INIT_FREQ);

        // Send 74+ clock cycles
        self.wait_card_active();

        // Delay after card activation for connection stability (per C SDK: 100ms)
        // Using ~10ms which should be sufficient for most cards
        for _ in 0..500000 {
            core::hint::spin_loop();
        }

        // CMD0: GO_IDLE_STATE
        self.cmd(types::common_cmd::idle(), false)?;

        // CMD8: SEND_IF_COND
        self.cmd(types::sd_cmd::send_if_cond(1, 0xAA), false)?;
        let cic = types::CIC::from(self.get_response());
        if cic.pattern() != 0xAA {
            return Err(Error::UnsupportedCardVersion);
        }

        // ACMD41: SD_SEND_OP_COND (repeated until ready)
        let mut ocr: types::OCR<types::SD>;
        for _ in 0..1000 {
            self.cmd(types::common_cmd::app_cmd(0), false)?;
            match self.cmd(types::sd_cmd::sd_send_op_cond(true, false, false, 0x1FF), false) {
                Ok(_) | Err(Error::Crc) => {}
                Err(e) => return Err(e),
            }

            ocr = self.get_response().into();
            if !ocr.is_busy() {
                // Card is ready (bit 31 = 1 means power up complete)
                let card_type = if ocr.high_capacity() {
                    types::CardCapacity::HighCapacity
                } else {
                    types::CardCapacity::StandardCapacity
                };

                // CMD2: ALL_SEND_CID
                self.cmd(types::common_cmd::all_send_cid(), false)?;
                let cid: types::CID<types::SD> = self.get_response_r2().into();

                // CMD3: SEND_RELATIVE_ADDR
                self.cmd(types::sd_cmd::send_relative_address(), false)?;
                let rca = types::RCA::<types::SD>::from(self.get_response()).address();

                // CMD9: SEND_CSD
                self.cmd(types::common_cmd::send_csd(rca), false)?;
                let csd: types::CSD<types::SD> = self.get_response_r2().into();

                // CMD7: SELECT_CARD
                self.cmd(types::common_cmd::select_card(rca), false)?;

                // Store card info
                self.card = Some(Card {
                    card_type,
                    ocr,
                    rca,
                    cid,
                    csd,
                    scr: types::SCR::default(),
                });

                // Switch to target frequency
                self.set_clock(freq);

                // After clock change, send 74+ clocks to re-synchronize card
                self.wait_card_active();

                // Set 4-bit bus width if available
                if self._d3.is_some() {
                    self.cmd(types::common_cmd::app_cmd(rca), false)?;
                    self.cmd(types::sd_cmd::set_bus_width(true), false)?;
                    self.set_bus_width(BusWidth::Four);
                }

                return Ok(());
            }

            // Small delay between retries
            for _ in 0..10000 {
                core::hint::spin_loop();
            }
        }

        Err(Error::Timeout)
    }
}

// Helper functions for pin configuration
fn configure_clk_pin<T: Instance>(pin: &impl ClkPin<T>) {
    pin.ioc_pad().func_ctl().write(|w| {
        w.set_alt_select(pin.alt_num());
        w.set_loop_back(true);
    });
    pin.ioc_pad().pad_ctl().write(|w| {
        w.set_ds(7);
    });
}

fn configure_cmd_pin<T: Instance>(pin: &impl CmdPin<T>) {
    pin.ioc_pad().func_ctl().write(|w| {
        w.set_alt_select(pin.alt_num());
        w.set_loop_back(true);
    });
    pin.ioc_pad().pad_ctl().write(|w| {
        w.set_ds(7);
        w.set_pe(true);
        w.set_ps(true);
        // Note: SD spec requires open-drain during init for multi-card,
        // but push-pull works for single card and is required for high-speed
        w.set_od(false);
    });
}

fn configure_d0_pin<T: Instance>(pin: &impl D0Pin<T>) {
    pin.ioc_pad().func_ctl().write(|w| {
        w.set_alt_select(pin.alt_num());
        w.set_loop_back(true);
    });
    pin.ioc_pad().pad_ctl().write(|w| {
        w.set_ds(7);
        w.set_pe(true);
        w.set_ps(true);
    });
}

fn configure_d1_pin<T: Instance>(pin: &impl D1Pin<T>) {
    pin.ioc_pad().func_ctl().write(|w| {
        w.set_alt_select(pin.alt_num());
        w.set_loop_back(true);
    });
    pin.ioc_pad().pad_ctl().write(|w| {
        w.set_ds(7);
        w.set_pe(true);
        w.set_ps(true);
    });
}

fn configure_d2_pin<T: Instance>(pin: &impl D2Pin<T>) {
    pin.ioc_pad().func_ctl().write(|w| {
        w.set_alt_select(pin.alt_num());
        w.set_loop_back(true);
    });
    pin.ioc_pad().pad_ctl().write(|w| {
        w.set_ds(7);
        w.set_pe(true);
        w.set_ps(true);
    });
}

fn configure_d3_pin<T: Instance>(pin: &impl D3Pin<T>) {
    pin.ioc_pad().func_ctl().write(|w| {
        w.set_alt_select(pin.alt_num());
        w.set_loop_back(true);
    });
    pin.ioc_pad().pad_ctl().write(|w| {
        w.set_ds(7);
        w.set_pe(true);
        w.set_ps(true);
    });
}

// ============================================================================
// Blocking mode implementation
// ============================================================================

impl<'d> Sdxc<'d, Blocking> {
    /// Create a new blocking SDXC driver with 1-bit bus width
    pub fn new_blocking_1bit<T: Instance>(
        peri: Peri<'d, T>,
        clk: Peri<'d, impl ClkPin<T>>,
        cmd: Peri<'d, impl CmdPin<T>>,
        d0: Peri<'d, impl D0Pin<T>>,
        config: Config,
    ) -> Self {
        configure_clk_pin::<T>(&*clk);
        configure_cmd_pin::<T>(&*cmd);
        configure_d0_pin::<T>(&*d0);

        Self::new_inner(
            peri,
            clk.into(),
            cmd.into(),
            d0.into(),
            None, None, None, None, None, None, None,
            config,
        )
    }

    /// Create a new blocking SDXC driver with 4-bit bus width
    pub fn new_blocking_4bit<T: Instance>(
        peri: Peri<'d, T>,
        clk: Peri<'d, impl ClkPin<T>>,
        cmd: Peri<'d, impl CmdPin<T>>,
        d0: Peri<'d, impl D0Pin<T>>,
        d1: Peri<'d, impl D1Pin<T>>,
        d2: Peri<'d, impl D2Pin<T>>,
        d3: Peri<'d, impl D3Pin<T>>,
        config: Config,
    ) -> Self {
        configure_clk_pin::<T>(&*clk);
        configure_cmd_pin::<T>(&*cmd);
        configure_d0_pin::<T>(&*d0);
        configure_d1_pin::<T>(&*d1);
        configure_d2_pin::<T>(&*d2);
        configure_d3_pin::<T>(&*d3);

        Self::new_inner(
            peri,
            clk.into(),
            cmd.into(),
            d0.into(),
            Some(d1.into()),
            Some(d2.into()),
            Some(d3.into()),
            None, None, None, None,
            config,
        )
    }

    /// Read a single 512-byte block
    ///
    /// Note: Uses SDMA mode due to hardware errata E00033 (PIO mode unreliable).
    pub fn read_block(&mut self, block_idx: u32, buffer: &mut DataBlock) -> Result<(), Error> {
        let card = self.card.as_ref().ok_or(Error::NoCard)?;
        let regs = self.info.regs;

        // Address conversion: SDHC/SDXC use block address, SDSC uses byte address
        let address = match card.card_type {
            types::CardCapacity::StandardCapacity => block_idx * 512,
            types::CardCapacity::HighCapacity => block_idx,
            _ => block_idx,
        };

        // Configure block transfer
        regs.blk_attr().write(|w| {
            w.set_xfer_block_size(512);
            w.set_block_cnt(1);
        });

        // Clear status
        regs.int_stat().write(|w| w.0 = 0xFFFFFFFF);

        // Configure SDMA: set DMA type to SDMA (0) and set buffer address
        // Due to errata E00033, PIO/FIFO mode is unreliable, must use DMA
        regs.prot_ctrl().modify(|w| w.set_dma_sel(0)); // SDMA
        regs.adma_sys_addr().write(|w| w.0 = buffer.0.as_ptr() as u32);

        // CMD16: SET_BLOCKLEN
        self.cmd(types::common_cmd::set_block_length(512), false)?;

        // CMD17: READ_SINGLE_BLOCK with DMA enabled
        self.cmd_dma(types::common_cmd::read_single_block(address))?;

        // Wait for transfer complete
        loop {
            let status = regs.int_stat().read();
            if status.data_tout_err() {
                return Err(Error::DataTimeout);
            }
            if status.data_crc_err() {
                return Err(Error::DataCrc);
            }
            if status.xfer_complete() {
                regs.int_stat().write(|w| w.set_xfer_complete(true));
                break;
            }
        }

        Ok(())
    }

    /// Read multiple 512-byte blocks using ADMA2
    ///
    /// Uses CMD18 (READ_MULTIPLE_BLOCK) with ADMA2 for efficient multi-block transfer.
    /// The descriptor table must be provided by the caller and placed in DMA-accessible memory.
    ///
    /// # Arguments
    /// - `block_idx`: Starting block index
    /// - `buffers`: Array of DataBlocks to read into
    /// - `adma_table`: ADMA2 descriptor table (must have at least `buffers.len()` entries)
    pub fn read_blocks_adma2<const N: usize>(
        &mut self,
        block_idx: u32,
        buffers: &mut [DataBlock],
        adma_table: &mut types::Adma2Table<N>,
    ) -> Result<(), Error> {
        if buffers.is_empty() {
            return Ok(());
        }
        if buffers.len() == 1 {
            return self.read_block(block_idx, &mut buffers[0]);
        }

        let card = self.card.as_ref().ok_or(Error::NoCard)?;
        let regs = self.info.regs;

        // Address conversion
        let address = match card.card_type {
            types::CardCapacity::StandardCapacity => block_idx * 512,
            types::CardCapacity::HighCapacity => block_idx,
            _ => block_idx,
        };

        let block_count = buffers.len() as u16;

        // Setup ADMA2 descriptors
        let desc_count = adma_table.setup_read(buffers);

        #[cfg(feature = "defmt")]
        defmt::debug!("ADMA2 read: block_idx={}, count={}, desc_count={}", block_idx, block_count, desc_count);
        #[cfg(feature = "defmt")]
        defmt::debug!("ADMA2 desc table @ 0x{:08X}", adma_table.as_ptr() as u32);

        // Configure block transfer
        regs.blk_attr().write(|w| {
            w.set_xfer_block_size(512);
            w.set_block_cnt(block_count);
        });

        // Clear status
        regs.int_stat().write(|w| w.0 = 0xFFFFFFFF);

        // Configure ADMA2
        regs.prot_ctrl().modify(|w| w.set_dma_sel(2)); // ADMA2
        regs.adma_sys_addr().write(|w| w.0 = adma_table.as_ptr() as u32);

        #[cfg(feature = "defmt")]
        defmt::debug!("PROT_CTRL: 0x{:08X}", regs.prot_ctrl().read().0);

        // CMD16: SET_BLOCKLEN
        self.cmd(types::common_cmd::set_block_length(512), false)?;

        #[cfg(feature = "defmt")]
        defmt::debug!("Sending CMD18 (READ_MULTIPLE_BLOCK), addr={}", address);

        // CMD18: READ_MULTIPLE_BLOCK with ADMA2 multi-block mode
        self.cmd_adma2_multi_read(types::common_cmd::read_multiple_blocks(address))?;

        #[cfg(feature = "defmt")]
        defmt::debug!("CMD18 sent, waiting for xfer_complete...");

        // Wait for transfer complete with timeout
        let mut timeout_count = 0u32;
        loop {
            let status = regs.int_stat().read();

            if status.data_tout_err() {
                #[cfg(feature = "defmt")]
                defmt::error!("Data timeout error! INT_STAT=0x{:08X}", status.0);
                let _ = self.cmd(types::common_cmd::stop_transmission(), false);
                return Err(Error::DataTimeout);
            }
            if status.data_crc_err() {
                #[cfg(feature = "defmt")]
                defmt::error!("Data CRC error! INT_STAT=0x{:08X}", status.0);
                let _ = self.cmd(types::common_cmd::stop_transmission(), false);
                return Err(Error::DataCrc);
            }
            if status.adma_err() {
                #[cfg(feature = "defmt")]
                {
                    let adma_err = regs.adma_err_stat().read();
                    defmt::error!("ADMA error! INT_STAT=0x{:08X}, ADMA_ERR=0x{:08X}", status.0, adma_err.0);
                }
                let _ = self.cmd(types::common_cmd::stop_transmission(), false);
                return Err(Error::AdmaError);
            }
            if status.xfer_complete() {
                regs.int_stat().write(|w| w.set_xfer_complete(true));
                #[cfg(feature = "defmt")]
                defmt::debug!("Transfer complete!");
                break;
            }

            timeout_count += 1;
            if timeout_count > 50_000_000 {
                #[cfg(feature = "defmt")]
                defmt::error!("Software timeout! INT_STAT=0x{:08X}, PSTATE=0x{:08X}", status.0, regs.pstate().read().0);
                let _ = self.cmd(types::common_cmd::stop_transmission(), false);
                return Err(Error::SoftwareTimeout);
            }
        }

        Ok(())
    }

    /// Read multiple 512-byte blocks (simple API using SDMA)
    ///
    /// Note: For better performance with many blocks, use `read_blocks_adma2` instead.
    /// This function uses repeated single-block SDMA reads for reliability.
    pub fn read_blocks(&mut self, block_idx: u32, buffers: &mut [DataBlock]) -> Result<(), Error> {
        // Use repeated single-block reads with SDMA (simpler and reliable)
        for (i, buffer) in buffers.iter_mut().enumerate() {
            self.read_block(block_idx + i as u32, buffer)?;
        }
        Ok(())
    }

    /// Write a single 512-byte block
    ///
    /// Note: Uses SDMA mode due to hardware errata E00033 (PIO mode unreliable).
    pub fn write_block(&mut self, block_idx: u32, buffer: &DataBlock) -> Result<(), Error> {
        let card = self.card.as_ref().ok_or(Error::NoCard)?;
        let regs = self.info.regs;

        // Address conversion: SDHC/SDXC use block address, SDSC uses byte address
        let address = match card.card_type {
            types::CardCapacity::StandardCapacity => block_idx * 512,
            types::CardCapacity::HighCapacity => block_idx,
            _ => block_idx,
        };

        // Configure block transfer
        regs.blk_attr().write(|w| {
            w.set_xfer_block_size(512);
            w.set_block_cnt(1);
        });

        // Clear status
        regs.int_stat().write(|w| w.0 = 0xFFFFFFFF);

        // Configure SDMA: set DMA type to SDMA (0) and set buffer address
        // Due to errata E00033, PIO/FIFO mode is unreliable, must use DMA
        regs.prot_ctrl().modify(|w| w.set_dma_sel(0)); // SDMA
        regs.adma_sys_addr().write(|w| w.0 = buffer.0.as_ptr() as u32);

        // CMD16: SET_BLOCKLEN
        self.cmd(types::common_cmd::set_block_length(512), false)?;

        // CMD24: WRITE_SINGLE_BLOCK with DMA enabled
        self.cmd_dma_write(types::common_cmd::write_single_block(address))?;

        // Wait for transfer complete
        loop {
            let status = regs.int_stat().read();
            if status.data_tout_err() {
                return Err(Error::DataTimeout);
            }
            if status.data_crc_err() {
                return Err(Error::DataCrc);
            }
            if status.xfer_complete() {
                regs.int_stat().write(|w| w.set_xfer_complete(true));
                break;
            }
        }

        Ok(())
    }

    /// Write multiple 512-byte blocks
    /// Write multiple 512-byte blocks using ADMA2
    ///
    /// Uses CMD25 (WRITE_MULTIPLE_BLOCK) with ADMA2 for efficient multi-block transfer.
    /// The descriptor table must be provided by the caller and placed in DMA-accessible memory.
    ///
    /// # Arguments
    /// - `block_idx`: Starting block index
    /// - `buffers`: Array of DataBlocks to write
    /// - `adma_table`: ADMA2 descriptor table (must have at least `buffers.len()` entries)
    pub fn write_blocks_adma2<const N: usize>(
        &mut self,
        block_idx: u32,
        buffers: &[DataBlock],
        adma_table: &mut types::Adma2Table<N>,
    ) -> Result<(), Error> {
        if buffers.is_empty() {
            return Ok(());
        }
        if buffers.len() == 1 {
            return self.write_block(block_idx, &buffers[0]);
        }

        let card = self.card.as_ref().ok_or(Error::NoCard)?;
        let regs = self.info.regs;

        // Address conversion
        let address = match card.card_type {
            types::CardCapacity::StandardCapacity => block_idx * 512,
            types::CardCapacity::HighCapacity => block_idx,
            _ => block_idx,
        };

        let block_count = buffers.len() as u16;

        // Setup ADMA2 descriptors
        let _desc_count = adma_table.setup_write(buffers);

        // Configure block transfer
        regs.blk_attr().write(|w| {
            w.set_xfer_block_size(512);
            w.set_block_cnt(block_count);
        });

        // Clear status
        regs.int_stat().write(|w| w.0 = 0xFFFFFFFF);

        // Configure ADMA2
        regs.prot_ctrl().modify(|w| w.set_dma_sel(2)); // ADMA2
        regs.adma_sys_addr().write(|w| w.0 = adma_table.as_ptr() as u32);

        // CMD16: SET_BLOCKLEN
        self.cmd(types::common_cmd::set_block_length(512), false)?;

        // CMD25: WRITE_MULTIPLE_BLOCK with ADMA2 multi-block mode
        self.cmd_adma2_multi_write(types::common_cmd::write_multiple_blocks(address))?;

        // Wait for transfer complete
        loop {
            let status = regs.int_stat().read();
            if status.data_tout_err() {
                let _ = self.cmd(types::common_cmd::stop_transmission(), false);
                return Err(Error::DataTimeout);
            }
            if status.data_crc_err() {
                let _ = self.cmd(types::common_cmd::stop_transmission(), false);
                return Err(Error::DataCrc);
            }
            if status.adma_err() {
                let _ = self.cmd(types::common_cmd::stop_transmission(), false);
                return Err(Error::AdmaError);
            }
            if status.xfer_complete() {
                regs.int_stat().write(|w| w.set_xfer_complete(true));
                break;
            }
        }

        // Note: Auto CMD12 is enabled, so no need to manually send STOP_TRANSMISSION

        // Wait for card to be ready (programming complete)
        self.wait_card_ready()?;

        Ok(())
    }

    /// Write multiple 512-byte blocks (simple API using SDMA)
    ///
    /// Note: For better performance with many blocks, use `write_blocks_adma2` instead.
    /// This function uses repeated single-block SDMA writes for reliability.
    pub fn write_blocks(&mut self, block_idx: u32, buffers: &[DataBlock]) -> Result<(), Error> {
        // Use repeated single-block writes with SDMA (simpler and reliable)
        for (i, buffer) in buffers.iter().enumerate() {
            self.write_block(block_idx + i as u32, buffer)?;
        }
        Ok(())
    }
}

// ============================================================================
// Async mode implementation
// ============================================================================

impl<'d> Sdxc<'d, Async> {
    /// Create a new async SDXC driver with 4-bit bus width
    pub fn new_4bit<T: Instance>(
        peri: Peri<'d, T>,
        _irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        clk: Peri<'d, impl ClkPin<T>>,
        cmd: Peri<'d, impl CmdPin<T>>,
        d0: Peri<'d, impl D0Pin<T>>,
        d1: Peri<'d, impl D1Pin<T>>,
        d2: Peri<'d, impl D2Pin<T>>,
        d3: Peri<'d, impl D3Pin<T>>,
        config: Config,
    ) -> Self {
        T::Interrupt::unpend();
        unsafe { T::Interrupt::enable() };

        configure_clk_pin::<T>(&*clk);
        configure_cmd_pin::<T>(&*cmd);
        configure_d0_pin::<T>(&*d0);
        configure_d1_pin::<T>(&*d1);
        configure_d2_pin::<T>(&*d2);
        configure_d3_pin::<T>(&*d3);

        Self::new_inner(
            peri,
            clk.into(),
            cmd.into(),
            d0.into(),
            Some(d1.into()),
            Some(d2.into()),
            Some(d3.into()),
            None, None, None, None,
            config,
        )
    }

    /// Async read single block using SDMA + interrupt
    pub async fn read_block_async(&mut self, block_idx: u32, buffer: &mut DataBlock) -> Result<(), Error> {
        use core::future::poll_fn;
        use core::task::Poll;

        let card = self.card.as_ref().ok_or(Error::NoCard)?;
        let regs = self.info.regs;
        let state = self.state;

        // Address conversion
        let address = match card.card_type {
            types::CardCapacity::StandardCapacity => block_idx * 512,
            types::CardCapacity::HighCapacity => block_idx,
            _ => block_idx,
        };

        // Configure block transfer
        regs.blk_attr().write(|w| {
            w.set_xfer_block_size(512);
            w.set_block_cnt(1);
        });

        // Clear all status
        regs.int_stat().write(|w| w.0 = 0xFFFFFFFF);
        state.clear_error();

        // Configure SDMA
        regs.prot_ctrl().modify(|w| w.set_dma_sel(0)); // SDMA
        regs.adma_sys_addr().write(|w| w.0 = buffer.0.as_ptr() as u32);

        // Enable interrupts (status enable + signal enable)
        regs.int_stat_en().write(|w| {
            w.set_cmd_complete_stat_en(true);
            w.set_xfer_complete_stat_en(true);
            w.set_dma_interrupt_stat_en(true);
            w.set_data_tout_err_stat_en(true);
            w.set_data_crc_err_stat_en(true);
            w.set_cmd_tout_err_stat_en(true);
        });
        regs.int_signal_en().write(|w| {
            w.set_xfer_complete_signal_en(true);
            w.set_data_tout_err_signal_en(true);
            w.set_data_crc_err_signal_en(true);
        });

        // CMD16: SET_BLOCKLEN
        self.cmd(types::common_cmd::set_block_length(512), false)?;

        // CMD17: READ_SINGLE_BLOCK with DMA
        self.cmd_dma(types::common_cmd::read_single_block(address))?;

        // Async wait for transfer complete
        let result = poll_fn(|cx| {
            state.waker.register(cx.waker());
            let status = regs.int_stat().read();

            if status.data_tout_err() {
                return Poll::Ready(Err(Error::DataTimeout));
            }
            if status.data_crc_err() {
                return Poll::Ready(Err(Error::DataCrc));
            }
            if status.xfer_complete() {
                return Poll::Ready(Ok(()));
            }
            Poll::Pending
        })
        .await;

        // Disable interrupts
        regs.int_signal_en().write(|_| {});

        // Clear status
        regs.int_stat().write(|w| w.0 = 0xFFFFFFFF);

        result
    }

    /// Async write single block using SDMA + interrupt
    pub async fn write_block_async(&mut self, block_idx: u32, buffer: &DataBlock) -> Result<(), Error> {
        use core::future::poll_fn;
        use core::task::Poll;

        let card = self.card.as_ref().ok_or(Error::NoCard)?;
        let regs = self.info.regs;
        let state = self.state;

        // Address conversion
        let address = match card.card_type {
            types::CardCapacity::StandardCapacity => block_idx * 512,
            types::CardCapacity::HighCapacity => block_idx,
            _ => block_idx,
        };

        // Configure block transfer
        regs.blk_attr().write(|w| {
            w.set_xfer_block_size(512);
            w.set_block_cnt(1);
        });

        // Clear all status
        regs.int_stat().write(|w| w.0 = 0xFFFFFFFF);
        state.clear_error();

        // Configure SDMA
        regs.prot_ctrl().modify(|w| w.set_dma_sel(0)); // SDMA
        regs.adma_sys_addr().write(|w| w.0 = buffer.0.as_ptr() as u32);

        // Enable interrupts
        regs.int_stat_en().write(|w| {
            w.set_cmd_complete_stat_en(true);
            w.set_xfer_complete_stat_en(true);
            w.set_dma_interrupt_stat_en(true);
            w.set_data_tout_err_stat_en(true);
            w.set_data_crc_err_stat_en(true);
            w.set_cmd_tout_err_stat_en(true);
        });
        regs.int_signal_en().write(|w| {
            w.set_xfer_complete_signal_en(true);
            w.set_data_tout_err_signal_en(true);
            w.set_data_crc_err_signal_en(true);
        });

        // CMD16: SET_BLOCKLEN
        self.cmd(types::common_cmd::set_block_length(512), false)?;

        // CMD24: WRITE_SINGLE_BLOCK with DMA
        self.cmd_dma_write(types::common_cmd::write_single_block(address))?;

        // Async wait for transfer complete
        let result = poll_fn(|cx| {
            state.waker.register(cx.waker());
            let status = regs.int_stat().read();

            if status.data_tout_err() {
                return Poll::Ready(Err(Error::DataTimeout));
            }
            if status.data_crc_err() {
                return Poll::Ready(Err(Error::DataCrc));
            }
            if status.xfer_complete() {
                return Poll::Ready(Ok(()));
            }
            Poll::Pending
        })
        .await;

        // Disable interrupts
        regs.int_signal_en().write(|_| {});

        // Clear status
        regs.int_stat().write(|w| w.0 = 0xFFFFFFFF);

        // Wait for card to be ready
        if result.is_ok() {
            self.wait_card_ready()?;
        }

        result
    }

    /// Async multi-block read using ADMA2 + interrupt
    pub async fn read_blocks_async<const N: usize>(
        &mut self,
        block_idx: u32,
        buffers: &mut [DataBlock],
        adma_table: &mut types::Adma2Table<N>,
    ) -> Result<(), Error> {
        use core::future::poll_fn;
        use core::task::Poll;

        if buffers.is_empty() {
            return Ok(());
        }
        if buffers.len() == 1 {
            return self.read_block_async(block_idx, &mut buffers[0]).await;
        }

        let card = self.card.as_ref().ok_or(Error::NoCard)?;
        let regs = self.info.regs;
        let state = self.state;

        // Address conversion
        let address = match card.card_type {
            types::CardCapacity::StandardCapacity => block_idx * 512,
            types::CardCapacity::HighCapacity => block_idx,
            _ => block_idx,
        };

        let block_count = buffers.len() as u16;

        // Setup ADMA2 descriptors
        let _ = adma_table.setup_read(buffers);

        // Configure block transfer
        regs.blk_attr().write(|w| {
            w.set_xfer_block_size(512);
            w.set_block_cnt(block_count);
        });

        // Clear all status
        regs.int_stat().write(|w| w.0 = 0xFFFFFFFF);
        state.clear_error();

        // Configure ADMA2
        regs.prot_ctrl().modify(|w| w.set_dma_sel(2)); // ADMA2
        regs.adma_sys_addr().write(|w| w.0 = adma_table.as_ptr() as u32);

        // Enable interrupts
        regs.int_stat_en().write(|w| {
            w.set_cmd_complete_stat_en(true);
            w.set_xfer_complete_stat_en(true);
            w.set_dma_interrupt_stat_en(true);
            w.set_data_tout_err_stat_en(true);
            w.set_data_crc_err_stat_en(true);
            w.set_cmd_tout_err_stat_en(true);
            w.set_adma_err_stat_en(true);
        });
        regs.int_signal_en().write(|w| {
            w.set_xfer_complete_signal_en(true);
            w.set_data_tout_err_signal_en(true);
            w.set_data_crc_err_signal_en(true);
            w.set_adma_err_signal_en(true);
        });

        // CMD16: SET_BLOCKLEN
        self.cmd(types::common_cmd::set_block_length(512), false)?;

        // CMD18: READ_MULTIPLE_BLOCK with ADMA2
        self.cmd_adma2_multi_read(types::common_cmd::read_multiple_blocks(address))?;

        // Async wait for transfer complete
        let result = poll_fn(|cx| {
            state.waker.register(cx.waker());
            let status = regs.int_stat().read();

            if status.data_tout_err() {
                return Poll::Ready(Err(Error::DataTimeout));
            }
            if status.data_crc_err() {
                return Poll::Ready(Err(Error::DataCrc));
            }
            if status.adma_err() {
                return Poll::Ready(Err(Error::AdmaError));
            }
            if status.xfer_complete() {
                return Poll::Ready(Ok(()));
            }
            Poll::Pending
        })
        .await;

        // Disable interrupts
        regs.int_signal_en().write(|_| {});

        // Clear status
        regs.int_stat().write(|w| w.0 = 0xFFFFFFFF);

        result
    }

    /// Async multi-block write using ADMA2 + interrupt
    pub async fn write_blocks_async<const N: usize>(
        &mut self,
        block_idx: u32,
        buffers: &[DataBlock],
        adma_table: &mut types::Adma2Table<N>,
    ) -> Result<(), Error> {
        use core::future::poll_fn;
        use core::task::Poll;

        if buffers.is_empty() {
            return Ok(());
        }
        if buffers.len() == 1 {
            return self.write_block_async(block_idx, &buffers[0]).await;
        }

        let card = self.card.as_ref().ok_or(Error::NoCard)?;
        let regs = self.info.regs;
        let state = self.state;

        // Address conversion
        let address = match card.card_type {
            types::CardCapacity::StandardCapacity => block_idx * 512,
            types::CardCapacity::HighCapacity => block_idx,
            _ => block_idx,
        };

        let block_count = buffers.len() as u16;

        // Setup ADMA2 descriptors
        let _ = adma_table.setup_write(buffers);

        // Configure block transfer
        regs.blk_attr().write(|w| {
            w.set_xfer_block_size(512);
            w.set_block_cnt(block_count);
        });

        // Clear all status
        regs.int_stat().write(|w| w.0 = 0xFFFFFFFF);
        state.clear_error();

        // Configure ADMA2
        regs.prot_ctrl().modify(|w| w.set_dma_sel(2)); // ADMA2
        regs.adma_sys_addr().write(|w| w.0 = adma_table.as_ptr() as u32);

        // Enable interrupts
        regs.int_stat_en().write(|w| {
            w.set_cmd_complete_stat_en(true);
            w.set_xfer_complete_stat_en(true);
            w.set_dma_interrupt_stat_en(true);
            w.set_data_tout_err_stat_en(true);
            w.set_data_crc_err_stat_en(true);
            w.set_cmd_tout_err_stat_en(true);
            w.set_adma_err_stat_en(true);
        });
        regs.int_signal_en().write(|w| {
            w.set_xfer_complete_signal_en(true);
            w.set_data_tout_err_signal_en(true);
            w.set_data_crc_err_signal_en(true);
            w.set_adma_err_signal_en(true);
        });

        // CMD16: SET_BLOCKLEN
        self.cmd(types::common_cmd::set_block_length(512), false)?;

        // CMD25: WRITE_MULTIPLE_BLOCK with ADMA2
        self.cmd_adma2_multi_write(types::common_cmd::write_multiple_blocks(address))?;

        // Async wait for transfer complete
        let result = poll_fn(|cx| {
            state.waker.register(cx.waker());
            let status = regs.int_stat().read();

            if status.data_tout_err() {
                return Poll::Ready(Err(Error::DataTimeout));
            }
            if status.data_crc_err() {
                return Poll::Ready(Err(Error::DataCrc));
            }
            if status.adma_err() {
                return Poll::Ready(Err(Error::AdmaError));
            }
            if status.xfer_complete() {
                return Poll::Ready(Ok(()));
            }
            Poll::Pending
        })
        .await;

        // Disable interrupts
        regs.int_signal_en().write(|_| {});

        // Clear status
        regs.int_stat().write(|w| w.0 = 0xFFFFFFFF);

        // Wait for card to be ready
        if result.is_ok() {
            self.wait_card_ready()?;
        }

        result
    }
}

impl<'d, M: Mode> Drop for Sdxc<'d, M> {
    fn drop(&mut self) {
        let regs = self.info.regs;
        regs.int_stat_en().write(|_| {});
        regs.int_signal_en().write(|_| {});
        regs.sys_ctrl().modify(|w| w.set_sw_rst_all(true));
    }
}

// ============================================================================
// embedded-sdmmc BlockDevice implementation
// ============================================================================

#[cfg(feature = "embedded-sdmmc")]
mod sdmmc_impl {
    use super::*;
    use core::cell::RefCell;

    /// Wrapper type for implementing `embedded_sdmmc::BlockDevice` trait.
    ///
    /// This wrapper provides interior mutability via `RefCell` to satisfy
    /// the `&self` requirement of the `BlockDevice` trait while allowing
    /// the underlying `Sdxc` driver to mutate its state.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use hpm_hal::sdxc::{Sdxc, SdCard, Config, DataBlock};
    /// use embedded_sdmmc::{BlockDevice, VolumeManager, TimeSource};
    ///
    /// let sdxc = Sdxc::new_blocking_4bit(/* ... */);
    /// sdxc.init_sd_card(Hertz::mhz(25)).unwrap();
    ///
    /// let sd_card = SdCard::new(sdxc);
    /// // Now sd_card implements BlockDevice
    /// ```
    pub struct SdCard<'d> {
        inner: RefCell<Sdxc<'d, Blocking>>,
    }

    impl<'d> SdCard<'d> {
        /// Create a new SdCard wrapper from an initialized Sdxc driver.
        ///
        /// The Sdxc driver should already have `init_sd_card()` called.
        pub fn new(sdxc: Sdxc<'d, Blocking>) -> Self {
            Self {
                inner: RefCell::new(sdxc),
            }
        }

        /// Get a reference to the underlying Sdxc driver.
        ///
        /// # Panics
        /// Panics if the driver is currently borrowed mutably.
        pub fn inner(&self) -> core::cell::Ref<'_, Sdxc<'d, Blocking>> {
            self.inner.borrow()
        }

        /// Get a mutable reference to the underlying Sdxc driver.
        ///
        /// # Panics
        /// Panics if the driver is currently borrowed.
        pub fn inner_mut(&self) -> core::cell::RefMut<'_, Sdxc<'d, Blocking>> {
            self.inner.borrow_mut()
        }

        /// Consume the wrapper and return the underlying Sdxc driver.
        pub fn into_inner(self) -> Sdxc<'d, Blocking> {
            self.inner.into_inner()
        }
    }

    impl embedded_sdmmc::BlockDevice for SdCard<'_> {
        type Error = Error;

        fn read(
            &self,
            blocks: &mut [embedded_sdmmc::Block],
            start_block_idx: embedded_sdmmc::BlockIdx,
        ) -> Result<(), Self::Error> {
            let mut driver = self.inner.borrow_mut();

            for (i, block) in blocks.iter_mut().enumerate() {
                let idx = start_block_idx.0 + i as u32;
                let mut data = DataBlock::new();
                driver.read_block(idx, &mut data)?;
                block.contents.copy_from_slice(data.as_slice());
            }
            Ok(())
        }

        fn write(
            &self,
            blocks: &[embedded_sdmmc::Block],
            start_block_idx: embedded_sdmmc::BlockIdx,
        ) -> Result<(), Self::Error> {
            let mut driver = self.inner.borrow_mut();

            for (i, block) in blocks.iter().enumerate() {
                let idx = start_block_idx.0 + i as u32;
                let data = DataBlock::from_slice(&block.contents);
                driver.write_block(idx, &data)?;
            }
            Ok(())
        }

        fn num_blocks(&self) -> Result<embedded_sdmmc::BlockCount, Self::Error> {
            let driver = self.inner.borrow();
            let card = driver.card().ok_or(Error::NoCard)?;
            // Calculate block count from CSD
            // For SDHC/SDXC, CSD v2.0 uses C_SIZE directly
            let block_count = card.csd.block_count();
            Ok(embedded_sdmmc::BlockCount(block_count as u32))
        }
    }
}

#[cfg(feature = "embedded-sdmmc")]
pub use sdmmc_impl::SdCard;
