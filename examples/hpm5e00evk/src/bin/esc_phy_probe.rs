#![no_main]
#![no_std]

use defmt_rtt as _;
use hpm_hal as hal;
use hal::gpio::Flex;
use panic_halt as _;

const RESULT_MAGIC: u32 = 0x4550_4859;
const RESULT_PASS: u32 = 0x5041_5353;
const RESULT_FAIL: u32 = 0x4641_494c;

const PHY_PORT0: u8 = 0;
const PHY_PORT1: u8 = 1;
const PHY_BMCR: u8 = 0;
const PHY_BMSR: u8 = 1;
const PHY_ID1: u8 = 2;
const PHY_ID2: u8 = 3;
const BMSR_LINK: u16 = 1 << 2;
const JL1111_ID1: u16 = 0x937c;
const JL1111_ID2_OUI: u16 = 0x10;
const MDIO_RETRIES: usize = 1_000_000;

#[unsafe(no_mangle)]
#[unsafe(link_section = ".uninit.esc_phy_result")]
static mut ESC_PHY_RESULT: [u32; 64] = [0; 64];

static mut MDIO_ACTIVITY: u32 = 0;
static mut MII_PAD_PREV: u32 = 0;
static mut MII_PAD_TRANSITIONS: u32 = 0;

#[hal::entry]
fn main() -> ! {
    clear_result();
    record(0, RESULT_MAGIC);
    record(1, 0x01);

    let p = hal::init(Default::default());
    record(1, 0x10);
    record(19, hal::sysctl::clocks().cpu0.0);

    hal::sysctl::clock_add_to_group(hal::pac::resources::ESC0, 0);
    riscv::asm::fence();

    let esc = hal::pac::ESC;
    record(2, esc.gpr_cfg0().read().0);
    esc.gpr_cfg0().modify(|w| w.set_clk100_en(true));

    init_esc_two_port_pins();
    record(1, 0x20);

    // Two 100-Mbit MII ports with the first external PHY at address 1.
    // The generated PAC omits PHY_OFFSET_VAL, so preserve the other fields and
    // update the documented bits [28:24] directly.
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
    let (refclk_transitions, refclk_high) = sample_pa09_refclk();
    record(16, refclk_transitions);
    record(17, refclk_high);

    // Both populated PHY LINK outputs are unrouted. A set GPR link bit means
    // nLINK is inactive, so initialize all ports to link-down.
    esc.gpr_cfg2().modify(|w| {
        w.set_nmii_link0_from_io(false);
        w.set_nmii_link0_gpr(true);
        w.set_nmii_link1_from_io(false);
        w.set_nmii_link1_gpr(true);
        w.set_nmii_link2_from_io(false);
        w.set_nmii_link2_gpr(true);
    });
    riscv::asm::fence();
    record(3, esc.gpr_cfg0().read().0);
    record(4, esc.phy_cfg0().read().0);
    record(5, esc.phy_cfg1().read().0);
    record(6, esc.gpr_cfg2().read().0);
    record(1, 0x30);

    let eeprom_before = read16(0x0502);
    if service_verified_sii_reload().is_err() {
        record(23, u32::from(eeprom_before) | (u32::from(read16(0x0502)) << 16));
        fail(0xe100);
    }
    let eeprom_after = read16(0x0502);
    record(23, u32::from(eeprom_before) | (u32::from(eeprom_after) << 16));
    if read16(0x0110) & 1 == 0 {
        fail(0xe101);
    }
    record(1, 0x38);

    // Drive the shared active-low reset only after the 25 MHz reference clock
    // and MII pads are configured. These conservative delays exceed the SDK's
    // 1 ms hold and 5 ms post-release requirements at the verified CPU clocks.
    let mut phy_reset = Flex::new(p.PB24);
    phy_reset.set_low();
    phy_reset.set_as_output(Default::default());
    spin(1_500_000);
    phy_reset.set_high();
    spin(6_000_000);
    record(22, u32::from(phy_reset.is_set_high()) | (u32::from(phy_reset.is_high()) << 1));
    record(61, read_pinmux_word());
    record(1, 0x40);

    esc.miim_pdi_acc_stat().write(|w| w.set_acc(true));
    riscv::asm::fence();
    init_mii_pad_activity();
    record(
        7,
        u32::from(esc.mii_mng_cs().read().0)
            | (u32::from(esc.miim_ecat_acc_stat().read().0) << 16)
            | (u32::from(esc.miim_pdi_acc_stat().read().0) << 24),
    );

    let p0 = read_phy_snapshot(PHY_PORT0);
    record(8, u32::from(p0.bmcr) | (u32::from(p0.bmsr) << 16));
    record(9, u32::from(p0.id1) | (u32::from(p0.id2) << 16));
    record(14, p0.error);

    let p1 = read_phy_snapshot(PHY_PORT1);
    record(10, u32::from(p1.bmcr) | (u32::from(p1.bmsr) << 16));
    record(11, u32::from(p1.id1) | (u32::from(p1.id2) << 16));
    record(15, p1.error);
    scan_direct_phy_addresses();
    record(20, mdio_activity());
    record(18, mii_pad_transitions());
    record(1, 0x50);

    if p0.error != 0 || p1.error != 0 {
        mdio_scope_loop(0xe001);
    }
    if !jl1111_id_matches(p0.id1, p0.id2) {
        mdio_scope_loop(0xe010);
    }
    if !jl1111_id_matches(p1.id1, p1.id2) {
        mdio_scope_loop(0xe011);
    }

    update_gpr_links(p0.bmsr & BMSR_LINK != 0, p1.bmsr & BMSR_LINK != 0);
    record(12, link_word(p0.bmsr, p1.bmsr));
    record(13, u32::from(read16(0x0110)) | (u32::from(read8(0x0518)) << 16) | (u32::from(read8(0x0519)) << 24));
    record(18, esc.gpr_cfg2().read().0);
    record(1, RESULT_PASS);

    defmt::info!(
        "HPM5E two-port ESC PHY PASS: p0={=u16:#06x}/{=u16:#06x}, p1={=u16:#06x}/{=u16:#06x}",
        p0.id1,
        p0.id2,
        p1.id1,
        p1.id2
    );

    let mut polls = 0u32;
    let mut errors = 0u32;
    loop {
        let link0 = read_current_link(PHY_PORT0);
        let link1 = read_current_link(PHY_PORT1);
        match (link0, link1) {
            (Ok((bmsr0, up0)), Ok((bmsr1, up1))) => {
                update_gpr_links(up0, up1);
                polls = polls.wrapping_add(1);
                record(12, link_word(bmsr0, bmsr1));
                record(16, u32::from(bmsr0) | (u32::from(bmsr1) << 16));
                record(17, polls);
                record(18, esc.gpr_cfg2().read().0);
            }
            _ => {
                errors = errors.wrapping_add(1);
                record(20, errors);
            }
        }
        spin(300_000);
    }
}

#[derive(Clone, Copy)]
struct PhySnapshot {
    bmcr: u16,
    bmsr: u16,
    id1: u16,
    id2: u16,
    error: u32,
}

fn read_phy_snapshot(port: u8) -> PhySnapshot {
    let bmcr = mdio_read(port, PHY_BMCR);
    let _ = mdio_read(port, PHY_BMSR);
    let bmsr = mdio_read(port, PHY_BMSR);
    let id1 = mdio_read(port, PHY_ID1);
    let id2 = mdio_read(port, PHY_ID2);
    let error = bmcr.err().or(bmsr.err()).or(id1.err()).or(id2.err()).unwrap_or(0);
    PhySnapshot {
        bmcr: bmcr.unwrap_or(0xffff),
        bmsr: bmsr.unwrap_or(0xffff),
        id1: id1.unwrap_or(0xffff),
        id2: id2.unwrap_or(0xffff),
        error,
    }
}

fn read_current_link(port: u8) -> Result<(u16, bool), u32> {
    let _ = mdio_read(port, PHY_BMSR)?;
    let bmsr = mdio_read(port, PHY_BMSR)?;
    Ok((bmsr, bmsr & BMSR_LINK != 0))
}

fn mdio_read(port: u8, register: u8) -> Result<u16, u32> {
    mdio_read_at(port, register, false)
}

fn mdio_read_at(address: u8, register: u8, direct: bool) -> Result<u16, u32> {
    let esc = hal::pac::ESC;
    wait_mdio_idle()?;
    esc.phy_addr().write(|w| {
        w.set_addr(address);
        w.set_show(direct);
    });
    esc.phy_reg_addr().write(|w| w.set_addr(register));
    esc.mii_mng_cs().modify(|w| w.set_cmd(1));
    riscv::asm::fence();
    wait_mdio_complete()?;
    let status = esc.mii_mng_cs().read();
    if status.cmd_err() {
        let error = 0x1000_0000 | u32::from(status.0);
        clear_mdio_error();
        return Err(error);
    }
    if status.rd_err() {
        let error = 0x2000_0000 | u32::from(status.0);
        clear_mdio_error();
        return Err(error);
    }
    Ok(esc.phy_data().read().data())
}

fn wait_mdio_complete() -> Result<(), u32> {
    let esc = hal::pac::ESC;
    let mut activity = 0u32;
    for _ in 0..MDIO_RETRIES {
        let status = esc.mii_mng_cs().read();
        sample_mii_pads();
        if status.cmd() == 1 {
            activity |= 1;
        }
        if status.busy() {
            activity |= 1 << 1;
        }
        if !status.busy() && status.cmd() == 0 {
            note_mdio_activity(activity);
            return Ok(());
        }
    }
    note_mdio_activity(activity | (1 << 2));
    Err(0x3100_0000 | u32::from(esc.mii_mng_cs().read().0))
}

fn note_mdio_activity(activity: u32) {
    unsafe {
        let ptr = core::ptr::addr_of_mut!(MDIO_ACTIVITY);
        let old = ptr.read_volatile();
        let count = (old >> 16).wrapping_add(1);
        ptr.write_volatile((count << 16) | (old & 0xffff) | activity);
    }
}

fn mdio_activity() -> u32 {
    unsafe { core::ptr::addr_of!(MDIO_ACTIVITY).read_volatile() }
}

fn sample_pa09_refclk() -> (u32, u32) {
    let mut previous = (hal::pac::GPIO0.di(0).value().read().0 >> 9) & 1;
    let mut transitions = 0u32;
    let mut high = 0u32;
    for _ in 0..100_000 {
        let current = (hal::pac::GPIO0.di(0).value().read().0 >> 9) & 1;
        transitions += u32::from(current != previous);
        high += current;
        previous = current;
    }
    (transitions, high)
}

fn init_mii_pad_activity() {
    let levels = hal::pac::GPIO0.di(0).value().read().0;
    unsafe {
        core::ptr::addr_of_mut!(MII_PAD_PREV).write_volatile(levels);
        core::ptr::addr_of_mut!(MII_PAD_TRANSITIONS).write_volatile(0);
    }
}

fn sample_mii_pads() {
    const MDIO: u32 = 1 << 30;
    const MDC: u32 = 1 << 31;
    let levels = hal::pac::GPIO0.di(0).value().read().0;
    unsafe {
        let previous_ptr = core::ptr::addr_of_mut!(MII_PAD_PREV);
        let transitions_ptr = core::ptr::addr_of_mut!(MII_PAD_TRANSITIONS);
        let previous = previous_ptr.read_volatile();
        let mut transitions = transitions_ptr.read_volatile();
        if (levels ^ previous) & MDC != 0 {
            transitions = transitions.wrapping_add(1);
        }
        if (levels ^ previous) & MDIO != 0 {
            transitions = transitions.wrapping_add(1 << 16);
        }
        previous_ptr.write_volatile(levels);
        transitions_ptr.write_volatile(transitions);
    }
}

fn mii_pad_transitions() -> u32 {
    unsafe { core::ptr::addr_of!(MII_PAD_TRANSITIONS).read_volatile() }
}

fn clear_mdio_error() {
    unsafe { (0xf170_0511usize as *mut u8).write_volatile(0) };
    riscv::asm::fence();
}

fn scan_direct_phy_addresses() {
    let mut complete = 0u32;
    let mut meaningful = 0u32;
    let mut zero = 0u32;
    let mut ones = 0u32;
    let mut error = 0u32;

    for address in 0u8..32 {
        let id1 = mdio_read_at(address, PHY_ID1, true);
        let id2 = mdio_read_at(address, PHY_ID2, true);
        match (id1, id2) {
            (Ok(id1), Ok(id2)) => {
                complete |= 1 << address;
                record(24 + usize::from(address), u32::from(id1) | (u32::from(id2) << 16));
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
                record(24 + usize::from(address), id1.err().or(id2.err()).unwrap_or(0x4000_0000));
            }
        }
    }

    record(56, complete);
    record(57, meaningful);
    record(58, zero);
    record(59, ones);
    record(60, error);
}

fn wait_mdio_idle() -> Result<(), u32> {
    let esc = hal::pac::ESC;
    for _ in 0..MDIO_RETRIES {
        if !esc.mii_mng_cs().read().busy() {
            return Ok(());
        }
    }
    Err(0x3000_0000 | u32::from(esc.mii_mng_cs().read().0))
}

fn jl1111_id_matches(id1: u16, id2: u16) -> bool {
    id1 == JL1111_ID1 && ((id2 >> 10) & 0x3f) == JL1111_ID2_OUI
}

fn update_gpr_links(link0_up: bool, link1_up: bool) {
    hal::pac::ESC.gpr_cfg2().modify(|w| {
        w.set_nmii_link0_gpr(!link0_up);
        w.set_nmii_link1_gpr(!link1_up);
        w.set_nmii_link2_gpr(true);
    });
    riscv::asm::fence();
}

fn link_word(bmsr0: u16, bmsr1: u16) -> u32 {
    u32::from(bmsr0 & BMSR_LINK != 0)
        | (u32::from(bmsr1 & BMSR_LINK != 0) << 1)
        | (u32::from(bmsr0) << 8)
        | (u32::from(bmsr1) << 24)
}

fn init_esc_two_port_pins() {
    use hal::pac::IOC;
    use hal::pac::iomux::*;
    use hal::pac::pins::*;

    IOC.pad(PA09).func_ctl().modify(|w| w.set_alt_select(IOC_PA09_FUNC_CTL_ESC0_REFCK));
    IOC.pad(PA30).func_ctl().modify(|w| w.set_alt_select(IOC_PA30_FUNC_CTL_ESC0_MDIO));
    IOC.pad(PA31).func_ctl().modify(|w| w.set_alt_select(IOC_PA31_FUNC_CTL_ESC0_MDC));

    IOC.pad(PA16).func_ctl().modify(|w| w.set_alt_select(IOC_PA16_FUNC_CTL_ESC0_P0_RXDV));
    IOC.pad(PA17).func_ctl().modify(|w| w.set_alt_select(IOC_PA17_FUNC_CTL_ESC0_P0_RXD_0));
    IOC.pad(PA18).func_ctl().modify(|w| w.set_alt_select(IOC_PA18_FUNC_CTL_ESC0_P0_RXD_1));
    IOC.pad(PA19).func_ctl().modify(|w| w.set_alt_select(IOC_PA19_FUNC_CTL_ESC0_P0_RXD_2));
    IOC.pad(PA20).func_ctl().modify(|w| w.set_alt_select(IOC_PA20_FUNC_CTL_ESC0_P0_RXD_3));
    IOC.pad(PA21).func_ctl().modify(|w| w.set_alt_select(IOC_PA21_FUNC_CTL_ESC0_P0_RXCK));
    IOC.pad(PA23).func_ctl().modify(|w| w.set_alt_select(IOC_PA23_FUNC_CTL_ESC0_P0_RXER));
    IOC.pad(PA24).func_ctl().modify(|w| w.set_alt_select(IOC_PA24_FUNC_CTL_ESC0_P0_TXCK));
    IOC.pad(PA25).func_ctl().modify(|w| w.set_alt_select(IOC_PA25_FUNC_CTL_ESC0_P0_TXD_0));
    IOC.pad(PA26).func_ctl().modify(|w| w.set_alt_select(IOC_PA26_FUNC_CTL_ESC0_P0_TXD_1));
    IOC.pad(PA27).func_ctl().modify(|w| w.set_alt_select(IOC_PA27_FUNC_CTL_ESC0_P0_TXD_2));
    IOC.pad(PA28).func_ctl().modify(|w| w.set_alt_select(IOC_PA28_FUNC_CTL_ESC0_P0_TXD_3));
    IOC.pad(PA29).func_ctl().modify(|w| w.set_alt_select(IOC_PA29_FUNC_CTL_ESC0_P0_TXEN));

    IOC.pad(PF02).func_ctl().modify(|w| w.set_alt_select(IOC_PF02_FUNC_CTL_ESC0_P1_TXCK));
    IOC.pad(PF03).func_ctl().modify(|w| w.set_alt_select(IOC_PF03_FUNC_CTL_ESC0_P1_TXD_0));
    IOC.pad(PF04).func_ctl().modify(|w| w.set_alt_select(IOC_PF04_FUNC_CTL_ESC0_P1_TXD_1));
    IOC.pad(PF05).func_ctl().modify(|w| w.set_alt_select(IOC_PF05_FUNC_CTL_ESC0_P1_TXD_2));
    IOC.pad(PF06).func_ctl().modify(|w| w.set_alt_select(IOC_PF06_FUNC_CTL_ESC0_P1_TXD_3));
    IOC.pad(PF07).func_ctl().modify(|w| w.set_alt_select(IOC_PF07_FUNC_CTL_ESC0_P1_TXEN));
    IOC.pad(PF08).func_ctl().modify(|w| w.set_alt_select(IOC_PF08_FUNC_CTL_ESC0_P1_RXDV));
    IOC.pad(PF09).func_ctl().modify(|w| w.set_alt_select(IOC_PF09_FUNC_CTL_ESC0_P1_RXD_0));
    IOC.pad(PF10).func_ctl().modify(|w| w.set_alt_select(IOC_PF10_FUNC_CTL_ESC0_P1_RXD_1));
    IOC.pad(PF11).func_ctl().modify(|w| w.set_alt_select(IOC_PF11_FUNC_CTL_ESC0_P1_RXD_2));
    IOC.pad(PF12).func_ctl().modify(|w| w.set_alt_select(IOC_PF12_FUNC_CTL_ESC0_P1_RXD_3));
    IOC.pad(PF13).func_ctl().modify(|w| w.set_alt_select(IOC_PF13_FUNC_CTL_ESC0_P1_RXCK));
    IOC.pad(PF14).func_ctl().modify(|w| w.set_alt_select(IOC_PF14_FUNC_CTL_ESC0_P1_RXER));
}

fn read_pinmux_word() -> u32 {
    use hal::pac::IOC;
    use hal::pac::pins::*;

    u32::from(IOC.pad(PA09).func_ctl().read().alt_select())
        | (u32::from(IOC.pad(PA30).func_ctl().read().alt_select()) << 8)
        | (u32::from(IOC.pad(PA31).func_ctl().read().alt_select()) << 16)
        | (u32::from(IOC.pad(PB24).func_ctl().read().alt_select()) << 24)
}

fn service_verified_sii_reload() -> Result<(), ()> {
    const EEPROM_BUSY: u16 = 1 << 15;
    const EEPROM_RELOAD: u16 = 4;
    const EEPROM_COMMAND_SHIFT: u16 = 8;

    let status = read16(0x0502);
    if status & EEPROM_BUSY == 0 || ((status >> EEPROM_COMMAND_SHIFT) & 0x7) != EEPROM_RELOAD {
        return Err(());
    }

    // Verified against the complete 2 KiB SDK ecat_io SII image: station alias
    // 0x3412 plus the enhanced-link response bits derived from configuration
    // byte 1. This is the same response proven by esc_sii_probe.
    write32(0x0508, 0x000f_3412);
    write32(0x050c, 0);
    write16(0x0502, EEPROM_RELOAD << EEPROM_COMMAND_SHIFT);

    for _ in 0..100_000 {
        if read16(0x0502) & EEPROM_BUSY == 0 {
            return Ok(());
        }
    }
    Err(())
}

fn mdio_scope_loop(code: u32) -> ! {
    record(21, code);
    record(1, RESULT_FAIL);
    let mut reads = 0u32;
    loop {
        match mdio_read_at(1, PHY_ID1, true) {
            Ok(data) => record(12, u32::from(data)),
            Err(error) => record(12, error),
        }
        reads = reads.wrapping_add(1);
        record(13, reads);
    }
}

fn fail(code: u32) -> ! {
    record(21, code);
    record(1, RESULT_FAIL);
    loop {
        riscv::asm::nop();
    }
}

fn read8(offset: usize) -> u8 {
    unsafe { ((0xf170_0000usize + offset) as *const u8).read_volatile() }
}

fn read16(offset: usize) -> u16 {
    unsafe { ((0xf170_0000usize + offset) as *const u16).read_volatile() }
}

fn write16(offset: usize, value: u16) {
    unsafe { ((0xf170_0000usize + offset) as *mut u16).write_volatile(value) };
    riscv::asm::fence();
}

fn write32(offset: usize, value: u32) {
    unsafe { ((0xf170_0000usize + offset) as *mut u32).write_volatile(value) };
    riscv::asm::fence();
}

fn clear_result() {
    unsafe { core::ptr::addr_of_mut!(MDIO_ACTIVITY).write_volatile(0) };
    unsafe { core::ptr::addr_of_mut!(MII_PAD_TRANSITIONS).write_volatile(0) };
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

#[inline(never)]
fn spin(cycles: u32) {
    for _ in 0..cycles {
        riscv::asm::nop();
    }
}
