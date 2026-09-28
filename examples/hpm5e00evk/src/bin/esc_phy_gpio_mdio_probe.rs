#![no_main]
#![no_std]

use defmt_rtt as _;
use hpm_hal as hal;
use hal::gpio::{Flex, Pull, Speed};
use panic_halt as _;

const RESULT_MAGIC: u32 = 0x474d_4449;
const RESULT_PASS: u32 = 0x5041_5353;
const RESULT_FAIL: u32 = 0x4641_494c;
const PHY_BMCR: u8 = 0;
const PHY_BMSR: u8 = 1;
const PHY_ID1: u8 = 2;
const PHY_ID2: u8 = 3;

#[unsafe(no_mangle)]
#[unsafe(link_section = ".uninit.esc_phy_result")]
static mut ESC_PHY_RESULT: [u32; 96] = [0; 96];

#[derive(Clone, Copy)]
struct MdioRead {
    turnaround: u8,
    data: u16,
}

#[hal::entry]
fn main() -> ! {
    clear_result();
    record(0, RESULT_MAGIC);
    record(1, 0x01);

    let p = hal::init(Default::default());
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
    record(1, 0x10);

    let mut phy_reset = Flex::new(p.PB24);
    phy_reset.set_low();
    phy_reset.set_as_output(Speed::default());
    spin(1_500_000);
    phy_reset.set_high();
    spin(6_000_000);
    record(5, u32::from(phy_reset.is_set_high()) | (u32::from(phy_reset.is_high()) << 1));

    // Flex changes GPIO direction and pad electrical settings; it deliberately
    // preserves the existing alternate-function selection. PPOR retains the
    // prior image's IOC selection, so explicitly return both pads to ALT0.
    init_gpio_mdio_pins();
    let mut mdc = Flex::new(p.PA31);
    mdc.set_low();
    mdc.set_as_output(Speed::default());
    let mut mdio = Flex::new(p.PA30);
    mdio.set_as_input(Pull::Up);
    let mut idle_high = 0u32;
    for _ in 0..1024 {
        idle_high += u32::from(mdio.is_high());
    }
    mdio.set_low();
    mdio.set_as_output(Speed::default());
    delay_half_cycle();
    let driven_low = mdio.is_low();
    mdio.set_high();
    delay_half_cycle();
    let driven_high = mdio.is_high();
    record(6, idle_high);
    record(7, u32::from(driven_low) | (u32::from(driven_high) << 1));
    record(1, 0x20);

    scan_phy_addresses(&mut mdc, &mut mdio);
    let bmcr1 = clause22_read(&mut mdc, &mut mdio, 1, PHY_BMCR);
    let bmsr1 = clause22_read(&mut mdc, &mut mdio, 1, PHY_BMSR);
    let bmcr2 = clause22_read(&mut mdc, &mut mdio, 2, PHY_BMCR);
    let bmsr2 = clause22_read(&mut mdc, &mut mdio, 2, PHY_BMSR);
    record(80, u32::from(bmcr1.data) | (u32::from(bmsr1.data) << 16));
    record(81, u32::from(bmcr1.turnaround) | (u32::from(bmsr1.turnaround) << 8));
    record(82, u32::from(bmcr2.data) | (u32::from(bmsr2.data) << 16));
    record(83, u32::from(bmcr2.turnaround) | (u32::from(bmsr2.turnaround) << 8));
    record(84, read_pinmux_word());

    let ack = result(9);
    record(1, if ack != 0 { RESULT_PASS } else { RESULT_FAIL });

    let mut reads = 0u32;
    loop {
        let value = clause22_read(&mut mdc, &mut mdio, 1, PHY_ID1);
        reads = reads.wrapping_add(1);
        record(88, u32::from(value.data) | (u32::from(value.turnaround) << 16));
        record(89, reads);
    }
}

fn scan_phy_addresses(mdc: &mut Flex<'_>, mdio: &mut Flex<'_>) {
    let mut complete = 0u32;
    let mut ack = 0u32;
    let mut no_response = 0u32;
    let mut meaningful = 0u32;
    let mut zero = 0u32;
    let mut ones = 0u32;
    let mut other_turnaround = 0u32;

    for address in 0u8..32 {
        let id1 = clause22_read(mdc, mdio, address, PHY_ID1);
        let id2 = clause22_read(mdc, mdio, address, PHY_ID2);
        complete |= 1 << address;
        record(16 + usize::from(address), u32::from(id1.data) | (u32::from(id2.data) << 16));
        record(
            48 + usize::from(address),
            u32::from(id1.turnaround) | (u32::from(id2.turnaround) << 8),
        );

        if id1.turnaround == 0b10 && id2.turnaround == 0b10 {
            ack |= 1 << address;
        } else if id1.turnaround == 0b11 && id2.turnaround == 0b11 {
            no_response |= 1 << address;
        } else {
            other_turnaround |= 1 << address;
        }

        if id1.data == 0 && id2.data == 0 {
            zero |= 1 << address;
        } else if id1.data == 0xffff && id2.data == 0xffff {
            ones |= 1 << address;
        } else {
            meaningful |= 1 << address;
        }
    }

    record(8, complete);
    record(9, ack);
    record(10, no_response);
    record(11, meaningful);
    record(12, zero);
    record(13, ones);
    record(14, other_turnaround);
}

fn clause22_read(
    mdc: &mut Flex<'_>,
    mdio: &mut Flex<'_>,
    phy_address: u8,
    register: u8,
) -> MdioRead {
    mdc.set_low();
    mdio.set_high();
    mdio.set_as_output(Speed::default());

    for _ in 0..32 {
        write_bit(mdc, mdio, true);
    }
    // Start=01, operation=10 (read), then PHYAD and REGAD, MSB first.
    write_bit(mdc, mdio, false);
    write_bit(mdc, mdio, true);
    write_bit(mdc, mdio, true);
    write_bit(mdc, mdio, false);
    write_bits(mdc, mdio, phy_address, 5);
    write_bits(mdc, mdio, register, 5);

    // Read turnaround is Z0. The internal pull-up makes Z observable as 1;
    // a responding PHY must drive the second bit low.
    mdio.set_as_input(Pull::Up);
    let ta0 = read_bit(mdc, mdio);
    let ta1 = read_bit(mdc, mdio);
    let mut data = 0u16;
    for _ in 0..16 {
        data = (data << 1) | u16::from(read_bit(mdc, mdio));
    }
    mdc.set_low();

    MdioRead {
        turnaround: (u8::from(ta0) << 1) | u8::from(ta1),
        data,
    }
}

fn write_bits(mdc: &mut Flex<'_>, mdio: &mut Flex<'_>, value: u8, count: u8) {
    for bit in (0..count).rev() {
        write_bit(mdc, mdio, value & (1 << bit) != 0);
    }
}

fn write_bit(mdc: &mut Flex<'_>, mdio: &mut Flex<'_>, value: bool) {
    mdc.set_low();
    if value {
        mdio.set_high();
    } else {
        mdio.set_low();
    }
    delay_half_cycle();
    mdc.set_high();
    delay_half_cycle();
    mdc.set_low();
}

fn read_bit(mdc: &mut Flex<'_>, mdio: &Flex<'_>) -> bool {
    mdc.set_low();
    delay_half_cycle();
    mdc.set_high();
    delay_half_cycle();
    let value = mdio.is_high();
    mdc.set_low();
    value
}

#[inline(never)]
fn delay_half_cycle() {
    spin(500);
}

fn init_refclk_pin() {
    use hal::pac::IOC;
    use hal::pac::iomux::*;
    use hal::pac::pins::*;

    IOC.pad(PA09).func_ctl().modify(|w| w.set_alt_select(IOC_PA09_FUNC_CTL_ESC0_REFCK));
}

fn init_gpio_mdio_pins() {
    use hal::pac::IOC;
    use hal::pac::iomux::*;
    use hal::pac::pins::*;

    IOC.pad(PA30).func_ctl().modify(|w| w.set_alt_select(IOC_PA30_FUNC_CTL_GPIO_A_30));
    IOC.pad(PA31).func_ctl().modify(|w| w.set_alt_select(IOC_PA31_FUNC_CTL_GPIO_A_31));
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
    for index in 0..96 {
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
