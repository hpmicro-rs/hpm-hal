#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]
#![feature(impl_trait_in_assoc_type)]

use embassy_executor::Spawner;
use embassy_time::Timer;
use embedded_io::Write as _;
use hal::gpio::{AnyPin, Level, Output, Pin as _};
use hal::mode::Blocking;
use hal::pac;
use hpm_hal::pac::pwm::vals;
use hpm_hal::pac::{iomux, pins};
use {defmt_rtt as _, hpm_hal as hal};

const BANNER: &str = include_str!("../../../assets/BANNER");

static mut UART: Option<hal::uart::UartTx<'static, Blocking>> = None;

const UNLOCK_KEY: u32 = 0xB0382607;

macro_rules! println {
    ($($arg:tt)*) => {
        unsafe {
            if let Some(uart) = UART.as_mut() {
                let _ = writeln!(uart, $($arg)*);
            }
        }
    };
}

#[embassy_executor::main(entry = "hpm_hal::entry")]
async fn main(_spawner: Spawner) -> ! {
    let p = hal::init(Default::default());

    let uart = hal::uart::UartTx::new_blocking(p.UART0, p.PA00, Default::default()).unwrap();
    unsafe {
        UART = Some(uart);
    }

    println!("Board init!");

    //let key_a = p.PB24;
    //let key_b = p.PB25;

    let led_r = p.PE14; // PWM1_P_6
    let led_g = p.PE15; // PWM1_P_7
    let led_b = p.PE04; // PWM0_P_4

    // init pins as pwm
    Output::new(led_r, Level::Low, Default::default());
    Output::new(led_g, Level::Low, Default::default());
    Output::new(led_b, Level::Low, Default::default());

    println!("{}", BANNER);

    println!("Clocks: {:?}", hal::sysctl::clocks());
    println!(
        "XPI0: {}Hz (noinit if running from ram)",
        hal::sysctl::clocks().get_clock_freq(pac::clocks::XPI0).0
    );
    println!("MCT0: {}Hz", hal::sysctl::clocks().get_clock_freq(pac::clocks::MCT0).0);

    pac::IOC
        .pad(pins::PE14)
        .func_ctl()
        .write(|w| w.set_alt_select(iomux::IOC_PE14_FUNC_CTL_PWM1_P_6));
    pac::IOC
        .pad(pins::PE15)
        .func_ctl()
        .write(|w| w.set_alt_select(iomux::IOC_PE15_FUNC_CTL_PWM1_P_7));
    pac::IOC
        .pad(pins::PE04)
        .func_ctl()
        .write(|w| w.set_alt_select(iomux::IOC_PE04_FUNC_CTL_PWM0_P_4));

    // =============

    hal::sysctl::clock_add_to_group(pac::resources::PWM0, 0); // open pwm0 clock

    let r = pac::PWM0;

    pac::PWM0.cnt_glbcfg().modify(|w| {
        w.set_timer_enable(0b0000);
        w.set_cnt_sw_start(0b0000); // enable pwm output
    });
    pac::PWM0.cnt_glbcfg().modify(|w| w.set_timer_reset(0b1111)); // reset all counter

    // unlock shadow?
    pac::PWM0.work_ctrl0().write(|w| w.0 = UNLOCK_KEY);
    //pac::PWM0.unlock().modify(|w| {
    //    w.set_unlock_bit(0b00111111_11111111_11111111_11111100);
    //});

    println!("unlock => 0x{:08x}", pac::PWM0.unlock().read().0); // defaults 0xffffffff, no need to unlock

    println!("shadow0 0x{:08x}", pac::PWM0.shadow_val(0).read().value());

    pac::PWM0.shadow_val(0).write(|w| w.0 = 0x1234_5678);

    println!("shadow0 0x{:08x}", pac::PWM0.shadow_val(0).read().value());

    r.shadow_val(0).write(|w| {
        w.set_frac(0);
        w.set_int(0xFFFF);
    }); // reload

    // cmp8 dynamic
    r.shadow_val(1).write(|w| {
        w.set_frac(0);
        w.set_int(0x1FFF);
    });
    // cmp9 = rld
    r.shadow_val(2).write(|w| {
        w.set_frac(0);
        w.set_int(0xFFFF);
    });

    // update
    r.cnt(2).cfg0().write(|w| {
        w.set_rld_update_time(vals::ReloadUpdateTrigger::ON_RELOAD); // on shadow lock
    });
    // cnt 2
    // cmp 8, 9. must 8 < 9
    // pwm p4

    r.cmp(8).cfg().write(|w| {
        w.set_cmp_update_time(vals::CmpShadowUpdateTrigger::ON_MODIFY);
        w.set_cmp_in_sel((vals::CmpSource::SHADOW_VAL as u8 + 1).into()); // from shadow 1
    });
    r.cmp(9).cfg().write(|w| {
        w.set_cmp_update_time(vals::CmpShadowUpdateTrigger::ON_MODIFY);
        w.set_cmp_in_sel((vals::CmpSource::SHADOW_VAL as u8 + 2).into()); // from shadow 1
    });

    pac::PWM0.pwm(4).cfg0().modify(|w| w.set_trig_sel4(false)); // 2 compare points
    pac::PWM0.pwm(4).cfg1().modify(|w| {
        w.set_pair_mode(false);
    });

    //    选择 2 个比较点模式 (N*2~N*2+1)
    // 8 to 9

    pac::PWM0.trigger_cfg(4).modify(|w| w.set_trigger_out_sel(0));

    // channel enable output
    r.pwm(4).cfg1().modify(|w| w.set_highz_en_n(true));

    pac::PWM0.cnt_glbcfg().modify(|w| {
        w.set_timer_enable(0b1111);
        w.set_cnt_sw_start(0b1111); // enable pwm output
    });

    println!("Hello, world!");

    let mut i = 0x4FFF;
    let mut inc = true;
    loop {
        defmt::info!("tick");

        Timer::after_millis(10).await;

        r.shadow_val(1).write(|w| {
            w.set_frac(0);
            w.set_int(i);
        });

        if i > 0xFFFF {
            i -= 0x1ff;
            inc = false;
        }
        if i <= 0x4Fff {
            i += 0x1ff;
            inc = true;
        }

        if inc {
            i += 0x1ff;
        } else {
            i -= 0x1ff;
        }

        let cnt = pac::PWM0.cnt_val(2).read().value();
        let rld = pac::PWM0.cnt_reload_work(2).read().value();
        let cmp8 = pac::PWM0.cmp_val_work(8).read().value(); // selected
        let cmp9 = pac::PWM0.cmp_val_work(9).read().value(); // selected

        /*   println!("cnt:  0x{:08x} {0}", cnt);
        println!("rld:  0x{:08x} {0}", rld);
        println!("cmp8: 0x{:08x} {0}", cmp8);
        println!("cmp9: 0x{:08x} {0}", cmp9);

        println!("Hello, world! xxx");
        */
    }
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    let mut err = heapless::String::<1024>::new();

    use core::fmt::Write as _;

    write!(err, "panic: {}", _info).ok();

    defmt::info!("{}", err.as_str());
    loop {}
}
