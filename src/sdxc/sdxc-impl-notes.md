# SDXC Driver Implementation Notes

## Overview

HPM6300 series SDXC driver for hpm-hal, following Embassy-style API patterns.

## Key Implementation Details

### 1. CMD Pin Open-Drain Mode Issue

**Problem**: After CMD7 (SELECT_CARD), subsequent commands (CMD55) failed with timeout at 25MHz, while working at 400kHz.

**Root Cause**: CMD pin was configured with open-drain mode (`od=true`). Open-drain mode has slower rise times and cannot reliably drive the CMD line at higher frequencies.

**Solution**: Configure CMD pin with push-pull mode (`od=false`):
```rust
fn configure_cmd_pin<T: Instance>(pin: &impl CmdPin<T>) {
    pin.ioc_pad().pad_ctl().write(|w| {
        w.set_ds(7);      // Max drive strength
        w.set_pe(true);   // Pull enable
        w.set_ps(true);   // Pull up
        w.set_od(false);  // Push-pull mode (NOT open-drain)
    });
}
```

**Note**: SD spec mentions open-drain for multi-card initialization, but single-card implementations work fine with push-pull, which is required for high-speed operation.

### 2. Response Type Handling

**R1b Response**: Commands like CMD7 (SELECT_CARD), CMD12 (STOP_TRANSMISSION) use R1b response which includes a busy signal on DAT0.

```rust
fn needs_busy_response(cmd_index: u8) -> bool {
    matches!(cmd_index, 7 | 12 | 28 | 29 | 38)
}

// In cmd():
let resp_type_sel = if needs_busy_response(cmd.cmd) {
    3  // R1b: 48-bit with busy check
} else {
    get_resp_type_select(resp_len)
};
```

RESP_TYPE_SELECT values:
- 0: No response
- 1: R136 (136-bit, for CID/CSD)
- 2: R48 (48-bit, standard response)
- 3: R48b (48-bit with busy on DAT0)

### 3. ACMD41 OCR Busy Bit

**Problem**: ACMD41 loop only ran once instead of polling until card ready.

**Root Cause**: `sdio_host::OCR::is_busy()` returns `true` when bit 31 = 0 (card busy), `false` when bit 31 = 1 (card ready). The condition was inverted.

**Solution**:
```rust
// WRONG: if ocr.is_busy() { /* card ready */ }
// CORRECT:
if !ocr.is_busy() {
    // Card is ready (bit 31 = 1 means power up complete)
}
```

### 4. Error Checking Order in cmd()

**Problem**: Commands could fail even when successful because error bits from previous operations remained set.

**Solution**: Check `cmd_complete` FIRST before checking error bits:
```rust
if status.cmd_complete() {
    regs.int_stat().write(|w| w.set_cmd_complete(true));
    return Ok(());
}
// Only check errors if command hasn't completed
if status.cmd_tout_err() {
    return Err(Error::Timeout);
}
```

### 5. Clock Configuration

HPM6300 uses `MISC_CTRL0.freq_sel_sw` for clock divider (HPM-specific, not standard SDHCI):

```rust
pub fn set_clock(&mut self, freq: Hertz) {
    let divider = (self.kernel_clock.0 / freq.0).max(1) - 1;

    regs.sys_ctrl().modify(|w| w.set_sd_clk_en(false));
    regs.misc_ctrl0().modify(|w| {
        w.set_freq_sel_sw(divider as u16);
        w.set_freq_sel_sw_en(true);
    });
    // Enable internal clock, wait stable, enable PLL, enable SD clock
}
```

After clock change, send 74+ clock cycles to re-sync card:
```rust
self.wait_card_active();
```

### 6. Pin Configuration

All SD pins need:
- `loop_back(true)` in FUNC_CTL for proper signal routing
- High drive strength (`ds=7`)
- Pull-up enabled for CMD and DAT lines

```rust
pin.ioc_pad().func_ctl().write(|w| {
    w.set_alt_select(pin.alt_num());
    w.set_loop_back(true);  // Required!
});
```

## SD Card Initialization Sequence

1. Set clock to 400kHz (identification mode)
2. Send 74+ clock cycles (`wait_card_active`)
3. CMD0 - GO_IDLE_STATE
4. CMD8 - SEND_IF_COND (check voltage, pattern 0xAA)
5. ACMD41 loop - SD_SEND_OP_COND (poll until ready)
6. CMD2 - ALL_SEND_CID
7. CMD3 - SEND_RELATIVE_ADDR (get RCA)
8. CMD9 - SEND_CSD
9. CMD7 - SELECT_CARD (card enters transfer state)
10. Switch clock to target frequency (e.g., 25MHz)
11. ACMD6 - SET_BUS_WIDTH (optional, for 4-bit mode)

## Debugging Tips

1. Check `INT_STAT` register for error details:
   - bit 16: cmd_tout_err
   - bit 17: cmd_crc_err
   - bit 18: cmd_end_bit_err

2. Check `PSTATE` for line levels:
   - `cmd_line_lvl`: CMD line level
   - `dat_3_0`: DAT[3:0] line levels
   - `cmd_inhibit`: CMD line busy

3. Use `MISC_CTRL0.freq_sel_sw` to verify clock divider setting

## References

- SD Physical Layer Simplified Specification
- HPM6300 Reference Manual - SDXC Chapter
- HPM SDK: `hpm_sdxc_drv.c`, `hpm_sdxc_soc_drv.h`
