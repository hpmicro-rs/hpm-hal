//! SDXC (SD/MMC Card Interface) driver
//!
//! Embassy-style driver for SD cards on HPMicro MCUs.
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

        // Save error status
        if status.0 & 0xFFFF_8000 != 0 {
            state.set_error(status.0);
        }

        // Clear interrupt flags
        regs.int_stat().write(|w| w.0 = status.0);

        // Disable interrupts that have fired
        regs.int_stat_en().modify(|w| {
            if status.cmd_complete() {
                w.set_cmd_complete_stat_en(false);
            }
            if status.xfer_complete() {
                w.set_xfer_complete_stat_en(false);
            }
            if status.dma_interrupt() {
                w.set_dma_interrupt_stat_en(false);
            }
        });

        // Wake waiting task
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

    /// Get 48-bit response
    fn get_response(&self) -> u32 {
        self.info.regs.resp(0).read().0
    }

    /// Get 136-bit response
    fn get_response_r2(&self) -> [u32; 4] {
        let regs = self.info.regs;
        [
            regs.resp(0).read().0,
            regs.resp(1).read().0,
            regs.resp(2).read().0,
            regs.resp(3).read().0,
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

    /// Initialize an SD card
    ///
    /// This performs the SD card identification and initialization sequence.
    /// After calling this method, the card is ready for data transfers.
    pub fn init_sd_card(&mut self, freq: Hertz) -> Result<(), Error> {
        let regs = self.info.regs;

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

    /// Read a single 512-byte block
    pub fn read_block(&mut self, block_idx: u32, buffer: &mut DataBlock) -> Result<(), Error> {
        let card = self.card.as_ref().ok_or(Error::NoCard)?;
        let regs = self.info.regs;

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

        // Clear status
        regs.int_stat().write(|w| w.0 = 0xFFFFFFFF);

        // CMD16: SET_BLOCKLEN
        self.cmd(types::common_cmd::set_block_length(512), false)?;

        // CMD17: READ_SINGLE_BLOCK
        self.cmd(types::common_cmd::read_single_block(address), true)?;

        // Wait for buffer ready
        loop {
            let status = regs.int_stat().read();
            if status.data_tout_err() {
                return Err(Error::DataTimeout);
            }
            if status.data_crc_err() {
                return Err(Error::DataCrc);
            }
            if status.buf_rd_ready() {
                regs.int_stat().write(|w| w.set_buf_rd_ready(true));
                break;
            }
        }

        // Read data (PIO mode)
        let data = buffer.as_mut_slice();
        for i in 0..128 {
            let word = regs.buf_data().read().0;
            let offset = i * 4;
            data[offset] = word as u8;
            data[offset + 1] = (word >> 8) as u8;
            data[offset + 2] = (word >> 16) as u8;
            data[offset + 3] = (word >> 24) as u8;
        }

        // Wait for transfer complete
        loop {
            let status = regs.int_stat().read();
            if status.xfer_complete() {
                regs.int_stat().write(|w| w.set_xfer_complete(true));
                break;
            }
        }

        Ok(())
    }

    /// Read multiple 512-byte blocks
    ///
    /// Uses CMD18 (READ_MULTIPLE_BLOCK) followed by CMD12 (STOP_TRANSMISSION).
    /// This is more efficient than calling `read_block` multiple times.
    pub fn read_blocks(&mut self, block_idx: u32, buffers: &mut [DataBlock]) -> Result<(), Error> {
        if buffers.is_empty() {
            return Ok(());
        }

        // Use single block read for just one block
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

        // Configure multi-block transfer
        regs.blk_attr().write(|w| {
            w.set_xfer_block_size(512);
            w.set_block_cnt(block_count);
        });

        // Clear status
        regs.int_stat().write(|w| w.0 = 0xFFFFFFFF);

        // CMD16: SET_BLOCKLEN
        self.cmd(types::common_cmd::set_block_length(512), false)?;

        // CMD18: READ_MULTIPLE_BLOCK
        // Wait for CMD and DAT lines to be free
        while regs.pstate().read().cmd_inhibit() {}
        while regs.pstate().read().dat_inhibit() {}

        // Set command argument first
        regs.cmd_arg().write(|w| w.0 = address);

        // Configure for multi-block data transfer
        regs.cmd_xfer().write(|w| {
            w.set_cmd_index(18);
            w.set_resp_type_select(2); // R1
            w.set_cmd_crc_chk_enable(true);
            w.set_cmd_idx_chk_enable(true);
            w.set_data_present_sel(true);
            w.set_data_xfer_dir(true); // Read
            w.set_multi_blk_sel(true);
            w.set_block_count_enable(true);
        });

        // Wait for command complete
        loop {
            let status = regs.int_stat().read();
            if status.cmd_complete() {
                regs.int_stat().write(|w| w.set_cmd_complete(true));
                break;
            }
            if status.cmd_tout_err() {
                return Err(Error::Timeout);
            }
            if status.cmd_crc_err() {
                return Err(Error::Crc);
            }
        }

        // Read each block using PIO
        for buffer in buffers.iter_mut() {
            // Wait for buffer ready
            loop {
                let status = regs.int_stat().read();
                if status.data_tout_err() {
                    // Try to send STOP command before returning error
                    let _ = self.cmd(types::common_cmd::stop_transmission(), false);
                    return Err(Error::DataTimeout);
                }
                if status.data_crc_err() {
                    let _ = self.cmd(types::common_cmd::stop_transmission(), false);
                    return Err(Error::DataCrc);
                }
                if status.buf_rd_ready() {
                    regs.int_stat().write(|w| w.set_buf_rd_ready(true));
                    break;
                }
            }

            // Read data (PIO mode)
            let data = buffer.as_mut_slice();
            for i in 0..128 {
                let word = regs.buf_data().read().0;
                let offset = i * 4;
                data[offset] = word as u8;
                data[offset + 1] = (word >> 8) as u8;
                data[offset + 2] = (word >> 16) as u8;
                data[offset + 3] = (word >> 24) as u8;
            }
        }

        // Wait for transfer complete
        loop {
            let status = regs.int_stat().read();
            if status.xfer_complete() {
                regs.int_stat().write(|w| w.set_xfer_complete(true));
                break;
            }
            if status.data_tout_err() {
                let _ = self.cmd(types::common_cmd::stop_transmission(), false);
                return Err(Error::DataTimeout);
            }
        }

        // CMD12: STOP_TRANSMISSION
        self.cmd(types::common_cmd::stop_transmission(), false)?;

        Ok(())
    }

    /// Write a single 512-byte block
    pub fn write_block(&mut self, block_idx: u32, buffer: &DataBlock) -> Result<(), Error> {
        let card = self.card.as_ref().ok_or(Error::NoCard)?;
        let regs = self.info.regs;

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

        // Clear status
        regs.int_stat().write(|w| w.0 = 0xFFFFFFFF);

        // CMD16: SET_BLOCKLEN
        self.cmd(types::common_cmd::set_block_length(512), false)?;

        // CMD24: WRITE_SINGLE_BLOCK
        self.cmd(types::common_cmd::write_single_block(address), true)?;

        // Wait for buffer ready
        loop {
            let status = regs.int_stat().read();
            if status.data_tout_err() {
                return Err(Error::DataTimeout);
            }
            if status.buf_wr_ready() {
                regs.int_stat().write(|w| w.set_buf_wr_ready(true));
                break;
            }
        }

        // Write data (PIO mode)
        let data = buffer.as_slice();
        for i in 0..128 {
            let offset = i * 4;
            let word = (data[offset] as u32)
                | ((data[offset + 1] as u32) << 8)
                | ((data[offset + 2] as u32) << 16)
                | ((data[offset + 3] as u32) << 24);
            regs.buf_data().write(|w| w.0 = word);
        }

        // Wait for transfer complete
        loop {
            let status = regs.int_stat().read();
            if status.xfer_complete() {
                regs.int_stat().write(|w| w.set_xfer_complete(true));
                break;
            }
        }

        Ok(())
    }

    /// Write multiple 512-byte blocks
    ///
    /// Uses CMD25 (WRITE_MULTIPLE_BLOCK) followed by CMD12 (STOP_TRANSMISSION).
    /// This is more efficient than calling `write_block` multiple times.
    pub fn write_blocks(&mut self, block_idx: u32, buffers: &[DataBlock]) -> Result<(), Error> {
        if buffers.is_empty() {
            return Ok(());
        }

        // Use single block write for just one block
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

        // Configure multi-block transfer
        regs.blk_attr().write(|w| {
            w.set_xfer_block_size(512);
            w.set_block_cnt(block_count);
        });

        // Clear status
        regs.int_stat().write(|w| w.0 = 0xFFFFFFFF);

        // CMD16: SET_BLOCKLEN
        self.cmd(types::common_cmd::set_block_length(512), false)?;

        // CMD25: WRITE_MULTIPLE_BLOCK
        // Wait for CMD and DAT lines to be free
        while regs.pstate().read().cmd_inhibit() {}
        while regs.pstate().read().dat_inhibit() {}

        // Set command argument first
        regs.cmd_arg().write(|w| w.0 = address);

        // Configure for multi-block data transfer
        regs.cmd_xfer().write(|w| {
            w.set_cmd_index(25);
            w.set_resp_type_select(2); // R1
            w.set_cmd_crc_chk_enable(true);
            w.set_cmd_idx_chk_enable(true);
            w.set_data_present_sel(true);
            w.set_data_xfer_dir(false); // Write
            w.set_multi_blk_sel(true);
            w.set_block_count_enable(true);
        });

        // Wait for command complete
        loop {
            let status = regs.int_stat().read();
            if status.cmd_complete() {
                regs.int_stat().write(|w| w.set_cmd_complete(true));
                break;
            }
            if status.cmd_tout_err() {
                return Err(Error::Timeout);
            }
            if status.cmd_crc_err() {
                return Err(Error::Crc);
            }
        }

        // Write each block using PIO
        for buffer in buffers.iter() {
            // Wait for buffer ready
            loop {
                let status = regs.int_stat().read();
                if status.data_tout_err() {
                    // Try to send STOP command before returning error
                    let _ = self.cmd(types::common_cmd::stop_transmission(), false);
                    return Err(Error::DataTimeout);
                }
                if status.buf_wr_ready() {
                    regs.int_stat().write(|w| w.set_buf_wr_ready(true));
                    break;
                }
            }

            // Write data (PIO mode)
            let data = buffer.as_slice();
            for i in 0..128 {
                let offset = i * 4;
                let word = (data[offset] as u32)
                    | ((data[offset + 1] as u32) << 8)
                    | ((data[offset + 2] as u32) << 16)
                    | ((data[offset + 3] as u32) << 24);
                regs.buf_data().write(|w| w.0 = word);
            }
        }

        // Wait for transfer complete
        loop {
            let status = regs.int_stat().read();
            if status.xfer_complete() {
                regs.int_stat().write(|w| w.set_xfer_complete(true));
                break;
            }
            if status.data_tout_err() {
                let _ = self.cmd(types::common_cmd::stop_transmission(), false);
                return Err(Error::DataTimeout);
            }
        }

        // CMD12: STOP_TRANSMISSION
        self.cmd(types::common_cmd::stop_transmission(), false)?;

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

    // TODO: Implement async methods
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
