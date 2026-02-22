#![macro_use]

use crate::pac;

pub(crate) fn configure_dmamux(mux_num: usize, request: u8) {
    defmt::info!("configure_dmamux: ch={}, request={}", mux_num, request);

    // Print DMAMUX base address and register address
    let dmamux_base = pac::DMAMUX.as_ptr() as u32;
    let ch_mux_regs = pac::DMAMUX.muxcfg(mux_num);
    let reg_addr = ch_mux_regs.as_ptr() as u32;
    defmt::info!("  DMAMUX base=0x{:08x}, muxcfg[{}] addr=0x{:08x}", dmamux_base, mux_num, reg_addr);

    // Read before write
    let before = ch_mux_regs.read();
    defmt::info!("  before: raw=0x{:08x}, enable={}, source={}", before.0, before.enable(), before.source());

    // Write the value
    let write_val = (1u32 << 31) | (request as u32);
    defmt::info!("  writing: 0x{:08x}", write_val);
    ch_mux_regs.write(|reg| {
        reg.set_enable(true);
        reg.set_source(request); // peripheral request number
    });

    // Verify
    let readback = ch_mux_regs.read();
    defmt::info!("  after: raw=0x{:08x}, enable={}, source={}", readback.0, readback.enable(), readback.source());
}
