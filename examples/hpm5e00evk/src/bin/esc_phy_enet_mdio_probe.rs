#![no_main]
#![no_std]

use defmt_rtt as _;
use hpm_hal as hal;
use hal::gpio::Flex;
use panic_halt as _;

const RESULT_MAGIC: u32 = 0x454e_4d44;
const RESULT_PASS: u32 = 0x5041_5353;
const RESULT_FAIL: u32 = 0x4641_494c;
const PHY_BMCR: u8 = 0;
const PHY_BMSR: u8 = 1;
const PHY_ID1: u8 = 2;
const PHY_ID2: u8 = 3;
const MDIO_RETRIES: usize = 100_000;

#[unsafe(no_mangle)]
#[unsafe(link_section = ".uninit.esc_phy_result")]
static mut ESC_PHY_RESULT: [u32; 64] = [0; 64];

#[hal::entry]
fn main() -> ! {
    clear_result();
    record(0, RESULT_MAGIC);
    record(1, 0x01);

    let p = hal::init(Default::default());
    record(1, 0x10);

    // Keep the ESC responsible for the board's 25 MHz PHY reference clock.
    hal::sysctl::clock_add_to_group(hal::pac::resources::ESC0, 0);
    let esc = hal::pac::ESC;
    esc.gpr_cfg0().modify(|w| w.set_clk100_en(true));
    esc.phy_cfg0().modify(|w| {
        w.set_port0_rmii_en(false);
        w.set_port1_rmii_en(false);
        w.set_port2_rmii_en(false);
        w.set_mac_speed(true);
        w.0 = (w.0 & !(0x1f << 24)) | (1 << 24);
    });
    esc.phy_cfg1().modify(|w| {
        w.set_refck_25m_inv(false);
        w.set_refck_25m_oe(true);
    });
    init_refclk_pin();
    riscv::asm::fence();
    record(2, esc.gpr_cfg0().read().0);
    record(3, esc.phy_cfg0().read().0);
    record(4, esc.phy_cfg1().read().0);
    record(1, 0x20);

    // Release the shared active-low PHY reset with the same conservative
    // timing already proven by the ESC MI diagnostic.
    let mut phy_reset = Flex::new(p.PB24);
    phy_reset.set_low();
    phy_reset.set_as_output(Default::default());
    spin(1_500_000);
    phy_reset.set_high();
    spin(6_000_000);
    record(5, u32::from(phy_reset.is_set_high()) | (u32::from(phy_reset.is_high()) << 1));
    record(1, 0x30);

    // PA30/PA31 are shared between ESC MI and ENET0 MDIO. Use the independent
    // ENET0 Clause-22 controller while all media-side MII pins remain untouched.
    hal::sysctl::clock_add_to_group(hal::pac::resources::ETH0, 0);
    init_enet_mdio_pins();
    riscv::asm::fence();
    record(6, read_pinmux_word());
    record(7, hal::pac::ENET0.gmii_addr().read().0);
    record(1, 0x40);

    scan_phy_addresses();
    record(48, snapshot(1));
    record(49, snapshot(2));
    record(50, hal::pac::SYSCTL.clock(hal::pac::clocks::ETH0).read().0);
    record(51, hal::pac::ENET0.gmii_addr().read().0);

    let meaningful = result(9);
    record(1, if meaningful != 0 { RESULT_PASS } else { RESULT_FAIL });

    // Leave an ongoing cross-controller signal on PA30/PA31 for counters and
    // optional scope inspection. Address 1 is the documented first PHY address.
    let mut reads = 0u32;
    loop {
        let value = enet_mdio_read(1, PHY_ID1).map(u32::from).unwrap_or_else(|e| e);
        reads = reads.wrapping_add(1);
        record(52, value);
        record(53, reads);
    }
}

fn scan_phy_addresses() {
    let mut complete = 0u32;
    let mut meaningful = 0u32;
    let mut zero = 0u32;
    let mut ones = 0u32;
    let mut error = 0u32;

    for address in 0u8..32 {
        let id1 = enet_mdio_read(address, PHY_ID1);
        let id2 = enet_mdio_read(address, PHY_ID2);
        match (id1, id2) {
            (Ok(id1), Ok(id2)) => {
                complete |= 1 << address;
                record(16 + usize::from(address), u32::from(id1) | (u32::from(id2) << 16));
                if id1 == 0 && id2 == 0 {
                    zero |= 1 << address;
                } else if id1 == 0xffff && id2 == 0xffff {
                    ones |= 1 << address;
                } else {
                    meaningful |= 1 << address;
                }
            }
            (id1, id2) => {
                error |= 1 << address;
                record(16 + usize::from(address), id1.err().or(id2.err()).unwrap_or(0xe000_0000));
            }
        }
    }

    record(8, complete);
    record(9, meaningful);
    record(10, zero);
    record(11, ones);
    record(12, error);
}

fn snapshot(address: u8) -> u32 {
    let bmcr = enet_mdio_read(address, PHY_BMCR).unwrap_or(0xffff);
    let _ = enet_mdio_read(address, PHY_BMSR);
    let bmsr = enet_mdio_read(address, PHY_BMSR).unwrap_or(0xffff);
    u32::from(bmcr) | (u32::from(bmsr) << 16)
}

fn enet_mdio_read(address: u8, register: u8) -> Result<u16, u32> {
    let enet = hal::pac::ENET0;
    for _ in 0..MDIO_RETRIES {
        if !enet.gmii_addr().read().gb() {
            enet.gmii_addr().write(|w| {
                w.set_pa(address);
                w.set_gr(register);
                // Matches the HPM SDK's HPM5E ENET driver: ETH CSR clock in
                // the 150-250 MHz range, yielding MDC = CSR clock / 102.
                w.set_cr(4);
                w.set_gw(false);
                w.set_gb(true);
            });
            riscv::asm::fence();
            for _ in 0..MDIO_RETRIES {
                if !enet.gmii_addr().read().gb() {
                    return Ok(enet.gmii_data().read().gd());
                }
            }
            return Err(0xe200_0000 | enet.gmii_addr().read().0);
        }
    }
    Err(0xe100_0000 | enet.gmii_addr().read().0)
}

fn init_refclk_pin() {
    use hal::pac::IOC;
    use hal::pac::iomux::*;
    use hal::pac::pins::*;

    IOC.pad(PA09).func_ctl().modify(|w| w.set_alt_select(IOC_PA09_FUNC_CTL_ESC0_REFCK));
}

fn init_enet_mdio_pins() {
    use hal::pac::IOC;
    use hal::pac::iomux::*;
    use hal::pac::pins::*;

    IOC.pad(PA30).func_ctl().modify(|w| w.set_alt_select(IOC_PA30_FUNC_CTL_ETH0_MDIO));
    IOC.pad(PA31).func_ctl().modify(|w| w.set_alt_select(IOC_PA31_FUNC_CTL_ETH0_MDC));
}

fn read_pinmux_word() -> u32 {
    use hal::pac::IOC;
    use hal::pac::pins::*;

    u32::from(IOC.pad(PA09).func_ctl().read().alt_select())
        | (u32::from(IOC.pad(PA30).func_ctl().read().alt_select()) << 8)
        | (u32::from(IOC.pad(PA31).func_ctl().read().alt_select()) << 16)
        | (u32::from(IOC.pad(PB24).func_ctl().read().alt_select()) << 24)
}

fn clear_result() {
    for index in 0..64 {
        record(index, 0);
    }
}

#[inline(never)]
fn record(index: usize, value: u32) {
    unsafe {
        core::ptr::addr_of_mut!(ESC_PHY_RESULT)
            .cast::<u32>()
            .add(index)
            .write_volatile(value);
    }
}

fn result(index: usize) -> u32 {
    unsafe {
        core::ptr::addr_of!(ESC_PHY_RESULT)
            .cast::<u32>()
            .add(index)
            .read_volatile()
    }
}

#[inline(never)]
fn spin(cycles: u32) {
    for _ in 0..cycles {
        riscv::asm::nop();
    }
}
