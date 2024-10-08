#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]
#![feature(impl_trait_in_assoc_type)]

use core::mem;

use embassy_executor::Spawner;
use embassy_time::Timer;
use embedded_io::Write as _;
use hal::gpio::{AnyPin, Level, Output, Pin as _};
use hal::mode::Blocking;
use hal::pac;
use {defmt_rtt as _, hpm_hal as hal};

const BANNER: &str = include_str!("../../../assets/BANNER");

static mut UART: Option<hal::uart::Uart<'static, Blocking>> = None;

macro_rules! println {
    ($($arg:tt)*) => {
        #[allow(unused_unsafe)]
        unsafe {
            if let Some(uart) = UART.as_mut() {
                let _ = writeln!(uart, $($arg)*);
            }
        }
    };
}

fn init_femc_pins() {
    use pac::iomux::*;
    use pac::pins::*;
    use pac::IOC;

    IOC.pad(PD02)
        .func_ctl()
        .write(|w| w.set_alt_select(IOC_PD02_FUNC_CTL_FEMC_A_00));
    IOC.pad(PD03)
        .func_ctl()
        .write(|w| w.set_alt_select(IOC_PD03_FUNC_CTL_FEMC_A_01));
    IOC.pad(PD00)
        .func_ctl()
        .write(|w| w.set_alt_select(IOC_PD00_FUNC_CTL_FEMC_A_02));
    IOC.pad(PD01)
        .func_ctl()
        .write(|w| w.set_alt_select(IOC_PD01_FUNC_CTL_FEMC_A_03));
    IOC.pad(PC18)
        .func_ctl()
        .write(|w| w.set_alt_select(IOC_PC18_FUNC_CTL_FEMC_A_04));
    IOC.pad(PC19)
        .func_ctl()
        .write(|w| w.set_alt_select(IOC_PC19_FUNC_CTL_FEMC_A_05));
    IOC.pad(PC20)
        .func_ctl()
        .write(|w| w.set_alt_select(IOC_PC20_FUNC_CTL_FEMC_A_06));
    IOC.pad(PC21)
        .func_ctl()
        .write(|w| w.set_alt_select(IOC_PC21_FUNC_CTL_FEMC_A_07));
    IOC.pad(PC23)
        .func_ctl()
        .write(|w| w.set_alt_select(IOC_PC23_FUNC_CTL_FEMC_A_08));
    IOC.pad(PC24)
        .func_ctl()
        .write(|w| w.set_alt_select(IOC_PC24_FUNC_CTL_FEMC_A_09));
    IOC.pad(PD04)
        .func_ctl()
        .write(|w| w.set_alt_select(IOC_PD04_FUNC_CTL_FEMC_A_10));
    IOC.pad(PC25)
        .func_ctl()
        .write(|w| w.set_alt_select(IOC_PC25_FUNC_CTL_FEMC_A_11)); /* SRAM: NWE */
    IOC.pad(PC26)
        .func_ctl()
        .write(|w| w.set_alt_select(IOC_PC26_FUNC_CTL_FEMC_A_12)); /* SRAM: NOE */

    IOC.pad(PD31)
        .func_ctl()
        .write(|w| w.set_alt_select(IOC_PD31_FUNC_CTL_FEMC_DQ_00));
    IOC.pad(PD30)
        .func_ctl()
        .write(|w| w.set_alt_select(IOC_PD30_FUNC_CTL_FEMC_DQ_01));
    IOC.pad(PD29)
        .func_ctl()
        .write(|w| w.set_alt_select(IOC_PD29_FUNC_CTL_FEMC_DQ_02));
    IOC.pad(PD28)
        .func_ctl()
        .write(|w| w.set_alt_select(IOC_PD28_FUNC_CTL_FEMC_DQ_03));
    IOC.pad(PD27)
        .func_ctl()
        .write(|w| w.set_alt_select(IOC_PD27_FUNC_CTL_FEMC_DQ_04));
    IOC.pad(PD26)
        .func_ctl()
        .write(|w| w.set_alt_select(IOC_PD26_FUNC_CTL_FEMC_DQ_05));
    IOC.pad(PD24)
        .func_ctl()
        .write(|w| w.set_alt_select(IOC_PD24_FUNC_CTL_FEMC_DQ_06));
    IOC.pad(PD25)
        .func_ctl()
        .write(|w| w.set_alt_select(IOC_PD25_FUNC_CTL_FEMC_DQ_07));
    IOC.pad(PD14)
        .func_ctl()
        .write(|w| w.set_alt_select(IOC_PD14_FUNC_CTL_FEMC_DQ_08));
    IOC.pad(PD17)
        .func_ctl()
        .write(|w| w.set_alt_select(IOC_PD17_FUNC_CTL_FEMC_DQ_09));
    IOC.pad(PD16)
        .func_ctl()
        .write(|w| w.set_alt_select(IOC_PD16_FUNC_CTL_FEMC_DQ_10));
    IOC.pad(PD19)
        .func_ctl()
        .write(|w| w.set_alt_select(IOC_PD19_FUNC_CTL_FEMC_DQ_11));
    IOC.pad(PD18)
        .func_ctl()
        .write(|w| w.set_alt_select(IOC_PD18_FUNC_CTL_FEMC_DQ_12));
    IOC.pad(PD21)
        .func_ctl()
        .write(|w| w.set_alt_select(IOC_PD21_FUNC_CTL_FEMC_DQ_13));
    IOC.pad(PD20)
        .func_ctl()
        .write(|w| w.set_alt_select(IOC_PD20_FUNC_CTL_FEMC_DQ_14));
    IOC.pad(PD22)
        .func_ctl()
        .write(|w| w.set_alt_select(IOC_PD22_FUNC_CTL_FEMC_DQ_15));
    IOC.pad(PC16)
        .func_ctl()
        .write(|w| w.set_alt_select(IOC_PC16_FUNC_CTL_FEMC_DQ_16));
    IOC.pad(PC17)
        .func_ctl()
        .write(|w| w.set_alt_select(IOC_PC17_FUNC_CTL_FEMC_DQ_17));
    IOC.pad(PC13)
        .func_ctl()
        .write(|w| w.set_alt_select(IOC_PC13_FUNC_CTL_FEMC_DQ_18));
    IOC.pad(PC14)
        .func_ctl()
        .write(|w| w.set_alt_select(IOC_PC14_FUNC_CTL_FEMC_DQ_19));
    IOC.pad(PC10)
        .func_ctl()
        .write(|w| w.set_alt_select(IOC_PC10_FUNC_CTL_FEMC_DQ_20));
    IOC.pad(PC11)
        .func_ctl()
        .write(|w| w.set_alt_select(IOC_PC11_FUNC_CTL_FEMC_DQ_21));
    IOC.pad(PC02)
        .func_ctl()
        .write(|w| w.set_alt_select(IOC_PC02_FUNC_CTL_FEMC_DQ_22));
    IOC.pad(PC09)
        .func_ctl()
        .write(|w| w.set_alt_select(IOC_PC09_FUNC_CTL_FEMC_DQ_23));
    IOC.pad(PC00)
        .func_ctl()
        .write(|w| w.set_alt_select(IOC_PC00_FUNC_CTL_FEMC_DQ_24));
    IOC.pad(PC01)
        .func_ctl()
        .write(|w| w.set_alt_select(IOC_PC01_FUNC_CTL_FEMC_DQ_25));
    IOC.pad(PC03)
        .func_ctl()
        .write(|w| w.set_alt_select(IOC_PC03_FUNC_CTL_FEMC_DQ_26));
    IOC.pad(PC04)
        .func_ctl()
        .write(|w| w.set_alt_select(IOC_PC04_FUNC_CTL_FEMC_DQ_27));
    IOC.pad(PC05)
        .func_ctl()
        .write(|w| w.set_alt_select(IOC_PC05_FUNC_CTL_FEMC_DQ_28));
    IOC.pad(PC06)
        .func_ctl()
        .write(|w| w.set_alt_select(IOC_PC06_FUNC_CTL_FEMC_DQ_29));
    IOC.pad(PC07)
        .func_ctl()
        .write(|w| w.set_alt_select(IOC_PC07_FUNC_CTL_FEMC_DQ_30));
    IOC.pad(PC08)
        .func_ctl()
        .write(|w| w.set_alt_select(IOC_PC08_FUNC_CTL_FEMC_DQ_31));

    IOC.pad(PD23)
        .func_ctl()
        .write(|w| w.set_alt_select(IOC_PD23_FUNC_CTL_FEMC_DM_0)); /* SRAM: NLB */
    IOC.pad(PD15)
        .func_ctl()
        .write(|w| w.set_alt_select(IOC_PD15_FUNC_CTL_FEMC_DM_1)); /* SRAM: NUB */
    IOC.pad(PC12)
        .func_ctl()
        .write(|w| w.set_alt_select(IOC_PC12_FUNC_CTL_FEMC_DM_2));
    IOC.pad(PC15)
        .func_ctl()
        .write(|w| w.set_alt_select(IOC_PC15_FUNC_CTL_FEMC_DM_3));

    IOC.pad(PX07).func_ctl().write(|w| {
        w.set_alt_select(IOC_PX07_FUNC_CTL_FEMC_DQS);
        w.set_loop_back(true);
    });

    /* SDRAM */
    IOC.pad(PD05)
        .func_ctl()
        .write(|w| w.set_alt_select(IOC_PD05_FUNC_CTL_FEMC_BA0));
    IOC.pad(PD06)
        .func_ctl()
        .write(|w| w.set_alt_select(IOC_PD06_FUNC_CTL_FEMC_BA1)); /* SRAM: NADV */
    IOC.pad(PD10)
        .func_ctl()
        .write(|w| w.set_alt_select(IOC_PD10_FUNC_CTL_FEMC_RAS));
    IOC.pad(PD13)
        .func_ctl()
        .write(|w| w.set_alt_select(IOC_PD13_FUNC_CTL_FEMC_CAS));
    IOC.pad(PC28)
        .func_ctl()
        .write(|w| w.set_alt_select(IOC_PC28_FUNC_CTL_FEMC_CKE));
    IOC.pad(PC27)
        .func_ctl()
        .write(|w| w.set_alt_select(IOC_PC27_FUNC_CTL_FEMC_CLK_0));
    IOC.pad(PD12)
        .func_ctl()
        .write(|w| w.set_alt_select(IOC_PD12_FUNC_CTL_FEMC_WE));
    IOC.pad(PD11)
        .func_ctl()
        .write(|w| w.set_alt_select(IOC_PD11_FUNC_CTL_FEMC_CS_0));
    IOC.pad(PD08)
        .func_ctl()
        .write(|w| w.set_alt_select(IOC_PD08_FUNC_CTL_FEMC_CS_1));

    /* SRAM */
    IOC.pad(PC29)
        .func_ctl()
        .write(|w| w.set_alt_select(IOC_PC29_FUNC_CTL_FEMC_SCLK_0));
    IOC.pad(PD07)
        .func_ctl()
        .write(|w| w.set_alt_select(IOC_PD07_FUNC_CTL_FEMC_SCLK_1));
    IOC.pad(PC30)
        .func_ctl()
        .write(|w| w.set_alt_select(IOC_PC30_FUNC_CTL_FEMC_SCS_0));
    IOC.pad(PC31)
        .func_ctl()
        .write(|w| w.set_alt_select(IOC_PC31_FUNC_CTL_FEMC_SCS_1));
    IOC.pad(PC22)
        .func_ctl()
        .write(|w| w.set_alt_select(IOC_PC22_FUNC_CTL_FEMC_SRDY));
}

fn init_ext_ram() {
    use hal::femc::*;

    init_femc_pins();

    let clk_in = hal::sysctl::clocks().get_clock_freq(pac::clocks::FEMC);

    let femc_config = FemcConfig::default();

    let mut femc = unsafe { Femc::new_raw(hal::peripherals::FEMC::steal()) };

    femc.init(femc_config);

    let mut sdram_config = FemcSdramConfig::default();

    sdram_config.bank_num = Bank2Sel::BANK_NUM_4;
    sdram_config.prescaler = 0x3;
    sdram_config.burst_len = BurstLen::_8;
    sdram_config.auto_refresh_count_in_one_burst = 1;
    //  Column address: A0-A8
    sdram_config.col_addr_bits = ColAddrBits::_9BIT;
    sdram_config.cas_latency = CasLatency::_3;

    // AC Characteristics and Operating Condition
    sdram_config.refresh_to_refresh_in_ns = 60; /* Trc */
    sdram_config.refresh_recover_in_ns = 60; /* Trc */
    sdram_config.act_to_precharge_in_ns = 42; /* Tras */
    sdram_config.act_to_rw_in_ns = 18; /* Trcd */
    sdram_config.precharge_to_act_in_ns = 18; /* Trp */
    sdram_config.act_to_act_in_ns = 12; /* Trrd */
    sdram_config.write_recover_in_ns = 12; /* Twr/Tdpl */
    sdram_config.self_refresh_recover_in_ns = 72; /* Txsr */

    sdram_config.cs = 0; // BOARD_SDRAM_CS; = FEMC_SDRAM_CS0 = 0
    sdram_config.base_address = 0x40000000; // BOARD_SDRAM_ADDRESS;
    sdram_config.size = MemorySize::_32MB;
    sdram_config.port_size = SdramPortSize::_16BIT;
    sdram_config.refresh_count = 8192; // BOARD_SDRAM_REFRESH_COUNT;
    sdram_config.refresh_in_ms = 64; // Tref, BOARD_SDRAM_REFRESH_IN_MS;

    sdram_config.delay_cell_disable = true;
    sdram_config.delay_cell_value = 0;

    let _ = femc.configure_sdram(clk_in.0, sdram_config).unwrap();
    pac::FEMC.sdrctrl0().modify(|w| w.set_highband(true));

    //    pac::FEMC.sdrctrl0().modify(|w| w.set_highband(true));
}

#[embassy_executor::task(pool_size = 3)]
async fn blink(pin: AnyPin, interval_ms: u32) {
    // all leds are active low
    let mut led = Output::new(pin, Level::Low, Default::default());

    loop {
        led.toggle();

        Timer::after_millis(interval_ms as u64).await;
    }
}

const MEM_START: usize = 0x40000000; // Start address for memory test
const MEM_SIZE: usize = 0x2000000; // 32MB

#[link_section = ".fast"]
fn memtest_operation(addr: *mut u32, data: u32, operation: u8) -> bool {
    unsafe {
        match operation {
            0 => {
                // Write operation
                core::ptr::write_volatile(addr, data);
                return true;
            }
            1 => {
                // Read and verify operation
                let read_data = core::ptr::read_volatile(addr);
                if read_data != data {
                    println!(
                        "Memory test failed at address 0x{:08X} (expected: 0x{:08X}, found: 0x{:08X})",
                        addr as usize, data, read_data
                    );
                    return false;
                }
                return true;
            }
            _ => unreachable!(),
        }
    }
}

#[link_section = ".fast"]
fn memtest() {
    let mem_start = MEM_START as *mut u32;
    let pattern = 0x12345678;

    println!(
        "Starting memory test from 0x{:08X} to 0x{:08X}",
        MEM_START,
        MEM_START + MEM_SIZE - 1
    );

    // Write pattern
    // Write pattern
    let mut start = riscv::register::mcycle::read64();
    for i in 0..(MEM_SIZE / mem::size_of::<u32>()) {
        let addr = unsafe { mem_start.offset(i as isize) };
        if !memtest_operation(addr, pattern, 0) {
            println!("Memory test failed!");
            return;
        }
        if i % 0x100000 == 0 {
            if i != 0 {
                let elapsed = riscv::register::mcycle::read64() - start;
                let elapsed_ms = elapsed * 1000 / (hal::sysctl::clocks().cpu0.0 as u64);
                let byte_per_sec = 0x100000 * mem::size_of::<u32>() as u64 * 1000 / elapsed_ms;
                println!(
                    "Memory test: 0x{:08X} of 0x{:08X} written, elapsed {}ms, {} bytes/s",
                    i,
                    MEM_SIZE / mem::size_of::<u32>(),
                    elapsed_ms,
                    byte_per_sec
                );
            }
            start = riscv::register::mcycle::read64();
        }
    }

    // Read and verify pattern
    for i in 0..(MEM_SIZE / mem::size_of::<u32>()) {
        let addr = unsafe { mem_start.offset(i as isize) };
        if !memtest_operation(addr, pattern, 1) {
            println!("Memory test failed!");
            return;
        }
        if i % 0x100000 == 0 {
            if i != 0 {
                let elapsed = riscv::register::mcycle::read64() - start;
                let elapsed_ms = elapsed * 1000 / (hal::sysctl::clocks().cpu0.0 as u64);
                let byte_per_sec = 0x100000 * mem::size_of::<u32>() as u64 * 1000 / elapsed_ms;
                println!(
                    "Memory test: 0x{:08X} of 0x{:08X} read, elapsed {}ms, {} bytes/s",
                    i,
                    MEM_SIZE / mem::size_of::<u32>(),
                    elapsed_ms,
                    byte_per_sec
                );
            }
            start = riscv::register::mcycle::read64();
        }
    }

    println!("Memory test completed successfully!");
}

#[embassy_executor::main(entry = "hpm_hal::entry")]
async fn main(spawner: Spawner) -> ! {
    let p = hal::init(Default::default());

    defmt::info!("Board init!");

    //let key_a = p.PB24;
    //let key_b = p.PB25;

    let led_r = p.PE14;
    let led_g = p.PE15;
    let led_b = p.PE04;

    spawner.spawn(blink(led_r.degrade(), 1000)).unwrap();
    spawner.spawn(blink(led_g.degrade(), 2000)).unwrap();
    spawner.spawn(blink(led_b.degrade(), 3000)).unwrap();
    defmt::info!("Tasks init!");

    let uart = hal::uart::Uart::new_blocking(p.UART0, p.PA01, p.PA00, Default::default()).unwrap();
    unsafe {
        UART = Some(uart);
    }

    println!("{}", BANNER);

    println!("Clocks: {:?}", hal::sysctl::clocks());
    println!(
        "XPI0: {}Hz (noinit if running from ram)",
        hal::sysctl::clocks().get_clock_freq(pac::clocks::XPI0).0
    );
    println!("MCT0: {}Hz", hal::sysctl::clocks().get_clock_freq(pac::clocks::MCT0).0);

    println!("Hello, world!");

    {
        pac::SYSCTL.monitor(0).control().modify(|w| {
            w.set_accuracy(true); // 1Hz
            w.set_reference(true); // 24M
            w.set_mode(true); // save to min and max
            w.set_selection(pac::sysctl::vals::MonitorSelection::CLK_TOP_FEMC); // pll0 clk0
            w.set_start(true);
        });
        while !pac::SYSCTL.monitor(0).control().read().valid() {}
        println!(
            "Monitor 0 measure: {} min={} max={}!",
            pac::SYSCTL.monitor(0).current().read().frequency(),
            pac::SYSCTL.monitor(0).low_limit().read().frequency(),
            pac::SYSCTL.monitor(0).high_limit().read().frequency()
        );
    }

    init_ext_ram();

    println!("Memory init done!");
    println!("Memory test start!");

    memtest();

    loop {
        defmt::info!("tick");

        Timer::after_millis(1000).await;

        println!("Hello, world! xxx");
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
