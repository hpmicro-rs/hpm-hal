//! I2S, inter-IC sound interface

use core::marker::PhantomData;

use embassy_hal_internal::{into_ref, Peripheral, PeripheralRef};

use crate::gpio::AnyPin;
use crate::{interrupt, pac};

/// I2S mode
#[derive(Copy, Clone)]
pub enum Mode {
    /// Master mode
    Master,
    /// Slave mode
    Slave,
}

/// I2C standard
#[derive(Copy, Clone)]
pub enum Standard {
    /// Philips
    Philips,
    /// Most significant bit first.
    MsbFirst,
    /// Least significant bit first.
    LsbFirst,
    /// PCM
    Pcm,
}

/// I2S data format.
#[derive(Copy, Clone)]
pub enum Format {
    /// 16 bit data length on 16 bit wide channel
    Data16Channel16,
    /// 16 bit data length on 32 bit wide channel
    Data16Channel32,
    /// 24 bit data length on 32 bit wide channel
    Data24Channel32,
    /// 32 bit data length on 32 bit wide channel
    Data32Channel32,
}

#[non_exhaustive]
#[derive(Copy, Clone)]
pub struct Config {
    /// Mode
    pub mode: Mode,
    /// Which I2S standard to use.
    pub standard: Standard,
    /// Data format.
    pub format: Format,

    pub invert_bclk_out: bool,
    pub invert_bclk_in: bool,
    pub use_external_bclk: bool,
    pub invert_fclk_out: bool,
    pub invert_fclk_in: bool,
    pub use_external_fclk: bool,
    pub invert_mclk_out: bool,
    pub invert_mclk_in: bool,
    pub use_external_mclk: bool,
    pub enable_mclk_out: bool,
    pub frame_start_at_rising_edge: bool,
    pub tx_fifo_threshold: u16,
    pub rx_fifo_threshold: u16,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            mode: Mode::Master,
            standard: Standard::Philips,
            format: Format::Data16Channel32,
            // actual config
            invert_mclk_out: false,
            invert_mclk_in: false,
            use_external_mclk: false,
            invert_bclk_out: false,
            invert_bclk_in: false,
            use_external_bclk: false,
            invert_fclk_out: false,
            invert_fclk_in: false,
            use_external_fclk: false,
            enable_mclk_out: false,
            frame_start_at_rising_edge: false,
            tx_fifo_threshold: 4,
            rx_fifo_threshold: 4,
        }
    }
}

/// Interrupt handler.
pub struct InterruptHandler<T: Instance> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> interrupt::typelevel::Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        //       let device = Device::<T>::new();
    }
}

// - MARK: I2S driver

/// I2S driver.
pub struct I2S<'d, T: Instance> {
    i2s: PeripheralRef<'d, T>,
    mclk: Option<PeripheralRef<'d, AnyPin>>,
    bclk: PeripheralRef<'d, AnyPin>,
    fclk: PeripheralRef<'d, AnyPin>,
    //sdin: Option<PeripheralRef<'d, AnyPin>>,
    //sdout: Option<PeripheralRef<'d, AnyPin>>,
    // master_clock: Option<MasterClock>,
    config: Config,
}

impl<'d, T: Instance> I2S<'d, T> {
    /// Create a new I2S in master mode
    pub fn new_master(
        i2s: impl Peripheral<P = T> + 'd,
        _irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        mclk: impl Peripheral<P = impl MclkPin<T>> + 'd,
        bclk: impl Peripheral<P = impl BclkPin<T>> + 'd,
        fclk: impl Peripheral<P = impl FclkPin<T>> + 'd,
        //     master_clock: MasterClock,
        config: Config,
    ) -> Self {
        todo!()
    }

    #[inline]
    fn disable(&mut self) {
        let r = T::REGS;
        r.ctrl().modify(|w| w.set_i2s_en(false));
    }
}

// The I2S software reset function relies on a working BCLK
#[inline]
fn reset_all(r: pac::i2s::I2s) {
    // disable I2S
    r.ctrl().modify(|w| w.set_i2s_en(false));

    // enable internal clock for software reset function

    let cfgr_temp = r.cfgr().read();

    r.cfgr().modify(|w| {
        w.set_bclk_div(1);
        w.set_mck_sel_op(false);
        w.set_bclk_sel_op(false);
        w.set_fclk_sel_op(false);
        w.set_bclk_gateoff(false);
    });

    let misc_cfgr_temp = r.misc_cfgr().read();
    r.misc_cfgr().modify(|w| w.set_mclk_gateoff(false));

    // reset function block and clear fifo
    r.ctrl().modify(|w| {
        w.set_txfifoclr(true);
        w.set_rxfifoclr(true);
        w.set_sftrst_clkgen(true);
        w.set_sftrst_tx(true);
        w.set_sftrst_rx(true);
    });
    r.ctrl().modify(|w| {
        w.set_rxfifoclr(false);
        w.set_txfifoclr(false);
        w.set_sftrst_clkgen(false);
        w.set_sftrst_tx(false);
        w.set_sftrst_rx(false);
    });

    // Restore the value of the register
    r.cfgr().write_value(cfgr_temp);
    r.misc_cfgr().write_value(misc_cfgr_temp);
}

fn init<T: Instance>(config: Config) {
    let r = T::REGS;

    reset_all(r);

    r.cfgr().write(|w| {
        w.set_inv_mclk_out(config.invert_mclk_out);
        w.set_inv_mclk_in(config.invert_mclk_in);
        w.set_mck_sel_op(config.use_external_mclk);
        w.set_inv_bclk_out(config.invert_bclk_out);
        w.set_inv_bclk_in(config.invert_bclk_in);
        w.set_bclk_sel_op(config.use_external_bclk);
        w.set_inv_fclk_out(config.invert_fclk_out);
        w.set_inv_fclk_in(config.invert_fclk_in);
        w.set_fclk_sel_op(config.use_external_fclk);
        w.set_frame_edge(config.frame_start_at_rising_edge);
    });
    r.misc_cfgr().modify(|w| {
        w.set_mclkoe(false);
        w.set_mclk_gateoff(true);
    });
    r.fifo_thresh().write(|w| {
        w.set_tx(config.tx_fifo_threshold as _);
        w.set_rx(config.rx_fifo_threshold as _);
    });

    // TODO: HPM_IP_FEATURE_I2S_BUFF_ALIGN_FRAME
}

// - MARK: instance

pub(crate) trait SealedInstance {
    const REGS: crate::pac::i2s::I2s;
}

#[allow(private_bounds)]
pub trait Instance: SealedInstance + crate::sysctl::ClockPeripheral + 'static {
    /// Interrupt for this peripheral.
    type Interrupt: crate::interrupt::typelevel::Interrupt;
}

foreach_peripheral!(
    (i2s, $inst:ident) => {
        impl SealedInstance for crate::peripherals::$inst {
            const REGS: crate::pac::i2s::I2s = crate::pac::$inst;
        }

        impl Instance for crate::peripherals::$inst {
            type Interrupt = crate::interrupt::typelevel::$inst;
        }
    };
);

pin_trait!(MclkPin, Instance);
pin_trait!(BclkPin, Instance);
pin_trait!(FclkPin, Instance);

pin_trait!(TxPin, Instance);
pin_trait!(RxPin, Instance);

dma_trait!(TxDma, Instance);
dma_trait!(RxDma, Instance);
