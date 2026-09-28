#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]
#![feature(impl_trait_in_assoc_type)]

use andes_riscv::register;
use defmt::println;
use embassy_executor::Spawner;
use embassy_time::Timer;
use embedded_io::Write as _;
use hal::gpio::{AnyPin, Flex, Pin};
use hal::pac;
use hal::pac::MCHTMR;
use hpm_hal::mode::Blocking;
use {defmt_rtt as _, hpm_hal as hal};

const BOARD_NAME: &str = "HPM5300EVK";
const BANNER: &str = include_str!("../../../assets/BANNER");

#[embassy_executor::task]
async fn blink(pin: AnyPin) {
    let mut led = Flex::new(pin);
    led.set_as_output(Default::default());
    led.set_high();

    loop {
        led.toggle();

        Timer::after_millis(500).await;
    }
}

macro_rules! println {
    ($($arg:tt)*) => {
        let _ = writeln!(unsafe {(&mut *(&raw mut UART)).as_mut().unwrap()}, $($arg)*);
    };
}

static mut UART: Option<hal::uart::Uart<'static, Blocking>> = None;

#[embassy_executor::main(entry = "hpm_hal::entry")]
async fn main(spawner: Spawner) -> ! {
    let p = hal::init(Default::default());
    // let button = Input::new(p.PA03, Pull::Down); // hpm5300evklite, BOOT1_KEY
    let uart = hal::uart::Uart::new_blocking(p.UART0, p.PA01, p.PA00, Default::default()).unwrap();
    unsafe {
        UART = Some(uart);
    }

    println!("{}", BANNER);
    println!("{} init OK!", BOARD_NAME);

    println!("Clock summary:");
    println!("  CPU0:\t{}Hz", hal::sysctl::clocks().cpu0.0);
    println!("  AHB:\t{}Hz", hal::sysctl::clocks().ahb.0);
    println!(
        "  XPI0:\t{}Hz",
        hal::sysctl::clocks().get_clock_freq(hal::pac::clocks::XPI0).0
    );
    println!(
        "  MTMR:\t{}Hz",
        hal::sysctl::clocks().get_clock_freq(pac::clocks::MCT0).0
    );

    println!("==============================");

    println!("Hello, world!");

    let r = register::mcache_ctl().read();

    defmt::info!("mcache_ctl: {:08x} dc {} ic {}", r.0, r.dc_en(), r.ic_en());

    loop {
        Timer::after_millis(1000).await;

        defmt::info!("tick {}", MCHTMR.mtime().read());
    }
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    println!("\n\n\nPANIC:\n{}", info);

    loop {}
}

/*
let v = andes_riscv::register::micm_cfg().read();

defmt::info!("micm_cfg: {:08x}", v.0);

defmt::info!("ilmsz {}", v.ilmsz());

let mdcm_cfg = andes_riscv::register::mdcm_cfg().read();

defmt::info!("mdcm_cfg: {:08x}", mdcm_cfg.0);
defmt::info!("dlmsz: {}", mdcm_cfg.dlmsz());
//let mie = riscv::register::mie::read();
//println!("mie: {:?}", mie);

let mmsc_cfg = andes_riscv::register::mmsc_cfg().read();
defmt::info!("mmsc_cfg: {:08x}", mmsc_cfg.0);
if mmsc_cfg.msc_ext() {
    let r = andes_riscv::register::mmsc_cfg2().read();

    defmt::info!("mmsc_cfg2: {:08x}", r.0);
}

let r = andes_riscv::register::mrvarch_cfg().read();

defmt::info!("mrvarch_cfg: {:08x}", r.0);

spawner.spawn(blink(p.PA23.degrade())).unwrap();

register::mcache_ctl().modify(|w| w.set_ic_en(false));
 */
