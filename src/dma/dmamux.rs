#![macro_use]

use crate::pac;

pub(crate) fn configure_dmamux(mux_num: usize, request: u8) {
    pac::DMAMUX.muxcfg(mux_num).write(|reg| {
        reg.set_enable(true);
        reg.set_source(request);
    });
}
