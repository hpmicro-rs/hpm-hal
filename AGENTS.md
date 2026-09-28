# Agent Memory: HPM5E31 Bring-up

## Current verified baseline (2026-07-17)

- `/Users/mono/Elec/HPM/handoff.md` is the current-state authority. Historical
  fault narratives later in this file remain useful as evidence and are
  superseded by this baseline where their status differs.
- HPM5E clock is configured through `sysctl::Config`: PLL0 VCO 960 MHz,
  CLK0 480 MHz, CLK1 400 MHz, DCDC 1275 mV.
- USB vendor bulk HPM Monitor V2 passed 1000/1000 after the implementation
  boundary cleanup; elapsed time was 15.081 s with zero protocol, panic, or
  exception errors.
- The final typed-PLIC `monitor_sensors` image passed 30/30 in 34.683 s. I2C
  cycle advanced 10 to 39, PMS7003 valid frames 14 to 54, and USB IRQ count
  769 to 2683.
- Core fixes stay in their owner layers: cache/IRQ in `hpm-riscv-rt`, clock in
  `sysctl/v6e.rs`, and qTD/W1C/generation/retirement in the USB driver.
- Qualification-only state is feature-gated: `usb-diagnostics` exposes USB
  snapshots and `runtime-diagnostics` retains early trap records. Both are
  enabled by the HPM5E example package and disabled in normal HAL builds.
- The board currently runs the post-cleanup `monitor_sensors` release image.

This repository contains active HPM5E31 custom-board bring-up work. Treat the points below as established baseline unless new evidence directly contradicts them.

## Probe and Board Baseline

- Board: custom HPM5E31 board under `examples/hpm5e00evk`.
- Probe: HSLink-Pro / CherryUSB CMSIS-DAP.
- Probe selector:

```sh
0d28:0204:214F44B4FEDADD10BB0B8044E000E80A
```

- Primary tool path:

```sh
/Users/mono/Elec/probe-rs/target/debug/probe-rs
```

- Use probe-rs as the primary and expected path for this board. HPM OpenOCD was useful during diagnosis, but the user explicitly wants this flow to continue with probe-rs.
- `cargo run` in `examples/hpm5e00evk` now uses the probe-rs runner from `.cargo/config.toml`.
- Use one probe process at a time. Check and clear stale probe processes before new debug/download work:

```sh
ps aux | rg 'probe-rs|openocd' || true
```

## Stable probe-rs Commands

Build examples from:

```sh
/Users/mono/Elec/HPM/hpm-hal/examples/hpm5e00evk
```

Stable download pattern:

```sh
/Users/mono/Elec/probe-rs/target/debug/probe-rs download \
  --chip HPM5E31 \
  --chip-description-path /Users/mono/Elec/HPM/hpm-hal/HPMicro.yaml \
  --protocol jtag \
  --probe 0d28:0204:214F44B4FEDADD10BB0B8044E000E80A \
  --speed 10 \
  --disable-progressbars \
  --disable-double-buffering \
  target/riscv32imafc-unknown-none-elf/debug/<bin>
```

Stable reset/read pattern:

```sh
/Users/mono/Elec/probe-rs/target/debug/probe-rs reset \
  --chip HPM5E31 \
  --chip-description-path /Users/mono/Elec/HPM/hpm-hal/HPMicro.yaml \
  --protocol jtag \
  --probe 0d28:0204:214F44B4FEDADD10BB0B8044E000E80A \
  --speed 10
```

`probe-rs reset` often prints a shutdown warning:

```text
Failed to deconfigure device during shutdown: HaltResume
```

This warning can occur after a successful reset. Use stage markers and behavior as the source of truth.

## Probe-rs Workflow Contract

Use `examples/hpm5e00evk/monitor.sh` as the reusable entry point for this board. It centralizes the probe path, chip description, probe selector, JTAG speed, timeout handling, and `uv`-based Python tooling.

Primary commands:

```sh
cd /Users/mono/Elec/HPM/hpm-hal/examples/hpm5e00evk
BIN=<example> ./monitor.sh build
BIN=<example> ./monitor.sh flash
./monitor.sh reset
BIN=<example> ./monitor.sh rtt
BIN=<example> ./monitor.sh smoke-bulk
./monitor.sh web
```

Workflow rules:

- Run one probe-rs process at a time.
- Use `./monitor.sh reset` after any halt/debug command before judging live target behavior.
- Treat `probe-rs debug` as a halting diagnostic tool. It leaves the core paused until an explicit reset.
- `./monitor.sh flash` is bounded by `FLASH_TIMEOUT` and attempts reset cleanup after timeout or failure.
- A `HaltResume` shutdown warning means probe-rs could not cleanly deconfigure the RISC-V debug module. Confirm the target state with stage markers.
- Manual BOOT0 or hardware reset is a recovery action for a wedged target/debug module state. It is outside the normal loop.
- A flash/download command that prints no progress within the timeout is a tool/target-state failure. Record it and continue with bounded recovery.

Useful process check:

```sh
pgrep -af 'probe-rs|monitor.sh|cargo build' || true
```

## Verified Runtime Baseline

PMA/PMP:

- For `hpm5e31`, `hpm-riscv-rt/pma-noncacheable` is currently disabled in `Cargo.toml`.
- With this disabled, RT reaches Rust `main()`.
- Build warnings showing unused PMP/PMA helpers are expected while this test configuration is active.

Main entry proof:

- `examples/hpm5e00evk/src/bin/main_marker.rs` is the minimal RT sanity check.
- Verified result after `download -> reset -> read`:

```text
MAIN_MARKER_STAGE = 0x00005100
__hpm_rt_stage    = 0x48504d53 0x00000190
trap record       = all zero
```

Meaning: the RT startup reaches Rust `main()` on HPM5E31.

## RTT and Embassy Timer Baseline

RTT/defmt has been verified with probe-rs. Do not restart the RTT investigation from scratch when switching examples.

Verified command:

```sh
/Users/mono/Elec/probe-rs/target/debug/probe-rs run \
  --chip HPM5E31 \
  --chip-description-path /Users/mono/Elec/HPM/hpm-hal/HPMicro.yaml \
  --protocol jtag \
  --probe 0d28:0204:214F44B4FEDADD10BB0B8044E000E80A \
  --speed 10 \
  --disable-progressbars \
  --disable-double-buffering \
  --no-location \
  target/riscv32imafc-unknown-none-elf/debug/rtt_defmt_probe
```

Verified RTT output:

```text
INFO  rtt defmt probe start
INFO  CPU0 clock: 600000000Hz
INFO  rtt defmt probe sample 0
INFO  rtt defmt probe sample 1
...
INFO  rtt defmt probe sample 15
INFO  rtt defmt probe done
```

Verified final stages:

```text
RTT_DEFMT_PROBE_STAGE = 0x00000200
__hpm_hal_init_stage  = 0x00000090
__hpm_rt_stage        = 0x48504d53 0x00000190
```

Meaning: RTT/defmt and the current embassy timer interrupt path pass in `rtt_defmt_probe`.

## Required Function Alignment

A real failure was found in the embassy timer path:

```text
mcause = 0
mepc   = 0x80003562
```

The faulting instruction was a `jalr` in `GptmrDriver::trigger_alarm()` to `Queue::next_expiration`.

Before the fix:

```text
0x8000adee Queue::next_expiration
```

The target was only 2-byte aligned. The example now adds:

```toml
"-C",
"llvm-args=-align-all-functions=2",
```

After rebuild:

```text
0x8000b3c8 Queue::next_expiration
```

This 4-byte function alignment is part of the current HPM5E31 baseline. Keep it while testing embassy timer paths and derived examples.

## Diagnostics and Stage Markers

Useful stage symbols in `rtt_defmt_probe`:

```text
RTT_DEFMT_PROBE_STAGE         = 0x00200078
__hpm_hal_init_stage          = 0x0020007c
__hpm_rt_stage                = 0x00200080
__hpm_preinit_trap_record     = 0x00200090
```

Read command:

```sh
/Users/mono/Elec/probe-rs/target/debug/probe-rs read \
  --chip HPM5E31 \
  --chip-description-path /Users/mono/Elec/HPM/hpm-hal/HPMicro.yaml \
  --protocol jtag \
  --probe 0d28:0204:214F44B4FEDADD10BB0B8044E000E80A \
  --speed 10 \
  b32 0x00200078 16
```

Stage meanings:

```text
RTT_DEFMT_PROBE_STAGE 0x10  entered main before Config::default()
RTT_DEFMT_PROBE_STAGE 0x18  Config::default() returned
RTT_DEFMT_PROBE_STAGE 0x20  hpm_hal::init() returned
RTT_DEFMT_PROBE_STAGE 0x100..0x10f RTT sample loop
RTT_DEFMT_PROBE_STAGE 0x200 completed RTT probe

__hpm_hal_init_stage 0x70  embassy timestamp path passed
__hpm_hal_init_stage 0x90  Peripherals::take() returned

__hpm_rt_stage magic 0x48504d53, stage 0x190 means RT reached main()
```

Trap record caveat:

- `__hpm_preinit_trap_record` lives in `.uninit`.
- After a successful run, stale values can remain there.
- Treat trap record as active only when it matches the current stage story or is freshly cleared by startup instrumentation.

## Workflow Rule for New Examples

For a new HPM5E31 example:

1. Keep probe-rs as the primary tool.
2. Build from `examples/hpm5e00evk`.
3. Preserve the 4-byte function alignment flag.
4. Use `rtt_defmt_probe` as the known-good RTT baseline.
5. Start debugging the new peripheral or example logic from its own failure point.
6. Revisit RT/RTT only when `rtt_defmt_probe` itself regresses.

## Sustainable Debug Boundary

Use this boundary to keep fixes durable across examples.

Baseline ownership:

- `main_marker` owns RT entry proof.
- `rtt_defmt_probe` owns RTT/defmt and embassy timer proof.
- `i2c_detect` owns I2C0 PC08/PC09 bus proof.
- `i2c_sensor_bme680` owns BME680 register and measurement proof.
- `monitor_usb_lite` owns the minimal USB monitor proof.
- `monitor_blinky` owns integrated monitor plus GPIO state proof.

Decision rule:

- If `main_marker` passes, treat RT entry as healthy for the current session.
- If `rtt_defmt_probe` passes, treat RTT/defmt and the embassy timer path as healthy for the current session.
- If `i2c_detect` passes, debug BME680 code at the sensor/driver layer.
- If `monitor_usb_lite` passes, debug integrated monitor failures inside endpoint, descriptor, symbol, or task composition code.
- Enter RT/startup code only when `main_marker` or `rtt_defmt_probe` regresses under the same probe-rs command set.

Baseline build gate:

```sh
cd /Users/mono/Elec/HPM/hpm-hal/examples/hpm5e00evk
./monitor.sh build-baseline
```

Hardware validation order:

```text
main_marker -> rtt_defmt_probe -> i2c_detect -> i2c_sensor_bme680 -> monitor_usb_lite -> monitor_blinky
```

Keep broad integration changes in a separate example or a clearly named branch until this order passes. When an integrated example fails while the earlier baselines pass, preserve the lower-layer conclusion and debug only the integration delta.

## Current USB Monitor State

Current focused example:

```text
examples/hpm5e00evk/src/bin/monitor_usb_lite.rs
```

The example owns minimal USB monitor transport. Keep USB diagnosis inside this example until it passes again.

Reusable findings:

- USB0 interrupt reached the handler in a prior run; `USB_DEBUG_IRQ_COUNT` increased.
- PLIC vector table placement at ILM address `0x00000000` is valid for this runtime layout.
- PLIC feature bit for vectored mode is bit 1. Priority slots use SDK-style indexing `(irq - 1)`.
- USB static descriptor/control buffers are explicitly 4-byte aligned in the current diagnostic version.
- `monitor_usb_lite` records `MON_LAST_ERROR` stages around USB setup and records panic line/column locally.
- `monitor_blinky` is the integrated BME680 + CDC monitor + vendor bulk monitor + GPIO blinky example.
- `monitor_blinky` descriptor/control/read buffers are aligned with `AlignedBuf`, and config/BOS descriptor buffers are 512 bytes.

Known stage map:

```text
0x200 entered main
0x210 hpm_hal::init returned
0x211..0x214 USB0 pinmux setup
0x220 UsbDriver::new returned
0x230 embassy_usb::Config prepared
0x240 before Builder::new
0x250 Builder::new returned
0x260 CdcAcmClass::new returned
0x270 builder.build returned
0x280 usb.run future entered
0x290 monitor future entered
0x2a0 waiting for host CDC connection
```

Current observed fault before the newest alignment/MSOS-buffer change:

```text
MON_LAST_ERROR = 0x260
mcause         = 1
mepc/mtval/ra  = 0x8023abb2
```

Meaning: execution reached `CdcAcmClass::new`, then faulted before `builder.build()` returned. `0x8023abb2` is outside the linked text range and indicates a corrupted return address or task frame. Use this as the current USB-layer fault signature.

Current `monitor_blinky` code validation:

```text
BIN=monitor_blinky ./monitor.sh build        passes
./monitor.sh build-baseline                  passes
```

Aligned `monitor_blinky` buffer addresses from ELF:

```text
CONFIG_DESC   0x01200000
CONTROL_BUF   0x01200204
USB_READ_BUF  0x01200248
BULK_READ_BUF 0x0120028c
BOS_DESC      0x01200ca0
EP_STATE      0x01201000
CDC_STATE     0x01204000
MSOS_DESC     0x01204024
```

Latest board validation attempt:

```text
BIN=monitor_blinky FLASH_TIMEOUT=20s ./monitor.sh flash
probe-rs download failed or timed out after 20s
preinit trap magic = 0x48504d54
mcause             = 2
mepc               = 0x80033f0c
mtval              = 0xffffffff
```

`0x80033f0c` maps to `_hpm_start_rust` in the current `monitor_blinky` ELF. The target is currently in an early RT trap/debug-module recovery state, so the integrated image has code-level validation only until the board is recovered through BOOT0 or hardware reset.

Follow-up flash-content check:

```text
flash read 0x80003000: matches ELF .text entry bytes
flash read 0x80033f0c: 0xffffffff repeated
flash read 0x8005c140: 0xffffffff repeated
```

Interpretation: the reset command runs, then the CPU immediately executes a partially erased/programmed flash image and traps on erased flash content. The user-visible behavior looks like reset recovery failure because the application image cannot boot after reset. The immediate cause is an incomplete/failed probe-rs download that left later flash regions erased.

Flash algorithm finding:

```text
ROM_API_TABLE_ROOT->version       = 0x56010200
xpi_nor_driver_if->version        = 0x56010300
HPM5E31 HPMicro.yaml flash algo   = flash-algo-hpm5300
SDK HPM5E00EVK NOR cfg option     = { 0xfcf90002, 0x00000005, 0x1000, 0x0 }
SDK HPM5301EVK lite NOR cfg option= { 0xfcf90002, 0x00000005, 0x1000, 0x0 }
```

The board-level NOR option matches HPM5301, so the flash chip geometry/config is shared. The algorithm risk is the HPM5E ROM/XPI path. The official SDK `samples/openocd_algo/src/openocd_flash_algo.c` enables hybrid XPI when `xpi_nor_driver_if->version >= 0x56010300`, then passes absolute flash addresses to erase/program/read. The current HPM5E31 probe-rs target still binds `flash-algo-hpm5300`, while the local Rust `flash-algo` implementation subtracts `FLASH_BASE` before erase/program. Treat HPM5E31 as requiring a dedicated flash algorithm with hybrid XPI enablement and HPM5E address semantics.

Verified HPM5E31 flash algo state:

```text
Flash algo repo: /Users/mono/Elec/HPM/flash-algo
Branch: hpm5e31-flash-algo
Target YAML: /Users/mono/Elec/HPM/flash-algo/hpm5e31-flash-algo.yaml
Probe-rs command path: use --chip HPM5E31 --chip-description-path above
```

The dedicated HPM5E31 flash algorithm works with hybrid XPI enabled and absolute flash addresses. A full `monitor_blinky` download using 4 KiB erase sectors completed in `125.38429s`; a later 64 KiB erase-block variant completed in `115.885025s`.

64 KiB erase-block notes:

```text
algorithm sector size: 0x10000
erase implementation: ROM xpi_nor_driver_if->erase_block
observed erase calls for monitor_blinky: 7 blocks, 0x80000000 through 0x80060000
observed erase time: about 0.32-0.34s per 64 KiB block
```

Meaning: larger erase blocks reduce erase time substantially, but total download time is still dominated by 10 kHz JTAG page download/program operations. Set `FLASH_TIMEOUT` to at least `180s` for `monitor_blinky` on this probe speed. A `--log-file` TRACE run can create multi-million-line logs and slow page downloads heavily, so use it only for diagnosis.

Probe speed testing on HSLink-Pro / CherryUSB CMSIS-DAP:

```text
SPEED=10     monitor_blinky flash completed in 115.885025s
SPEED=100    monitor_blinky flash completed in 111.26274s
SPEED=500    monitor_blinky flash completed in 41.658863s
SPEED=1000   monitor_blinky flash completed in 35.91141s
SPEED=2000   monitor_blinky flash completed in 34.92204s once, then a later attach hit DMI timeout
```

Default `examples/hpm5e00evk/monitor.sh` speed is `500kHz` with `FLASH_TIMEOUT=120s`. This is the current conservative default because it gives a large speedup and completed a full flash. `1000kHz` is a useful manual override for faster local testing. After a DMI timeout, lower speeds may also fail until the target or probe is manually reset.

Post-reset confirmation: after a manual target reset, the default `monitor.sh flash` path (`SPEED=500`, `FLASH_TIMEOUT=120s`) completed `monitor_blinky` in `42.03419s`. Flash readback at `0x80033f0c` returned programmed words, confirming the image was not left partially erased.

## Current i2c_detect State

Current verified `i2c_detect` result:

```text
INFO  i2c detect start
INFO  bus: I2C0, scl: PC08, sda: PC09
INFO  cpu0: 600000000Hz
INFO  scan 0x10..0x1f
INFO  scan 0x20..0x2f
INFO  scan 0x30..0x3f
INFO  scan 0x40..0x4f
INFO  scan 0x50..0x5f
INFO  scan 0x60..0x6f
INFO  scan 0x70..0x7f
INFO  found 0x77
INFO  i2c detect done, found 1 device(s)
```

Observed state:

```text
_SEGGER_RTT header present at 0x00200048
RTT write offset = 0x52
__hpm_rt_stage = 0x48504d53 0x00000190
__hpm_hal_init_stage = 0x00000090
```

Meaning: the I2C0 blocking scan on PC08/PC09 completed and found one device at address `0x77`.

Operational note: in the verified run, `probe-rs run` downloaded and started the image but did not print live RTT lines. A later `probe-rs attach` decoded the RTT buffer successfully.

Important diagnostic: `hpm5e31` previously enabled `hpm-riscv-rt/ignore-illegal-mtval-zero`; that path can hide `mcause=2, mtval=0` by returning from the trap. Keep it disabled during fault localization so the exception record can capture `mepc/mtval`.

## Current BME680 Sensor State

Verified example:

```text
examples/hpm5e00evk/src/bin/i2c_sensor_bme680.rs
```

Dependency:

```text
bosch-bme680 = "1.0.4"
```

Verified bus/device:

```text
I2C0, SCL PC08, SDA PC09, address 0x77
chip id: 0x61
```

Representative RTT output:

```text
INFO  i2c bme680 start
INFO  chip id: 0x61
INFO  bme680 init ok
INFO  sample 1
INFO  temperature: 29.59202 C
INFO  humidity: 60.93953 %
INFO  pressure: 99578.94 Pa
INFO  gas resistance: 47459.688 ohm
```

Longer run note: later samples stabilized around 30-33 C, 58-61 %RH, about 99580-100008 Pa, with gas resistance warming upward over time.

Integrated `monitor_blinky` BME680 readback after default `500kHz` flash and manual reset:

```text
__hpm_hal_init_stage  = 0x00000090
__hpm_rt_stage        = 0x48504d53 0x00000190
MON_BME680_CHIP_ID    = 0x61
MON_BME680_STATUS     = 5
MON_BME680_LAST_ERROR = 0
MON_BME680_SAMPLE_COUNT = 1
temperature           = 27.56 C
humidity              = 58.11 %RH
pressure              = 100160 Pa
gas resistance        = 9164 ohm
gas valid             = 1
```

Current transport note: `monitor_blinky` BME680 values are readable through DLM with probe-rs. USB bulk monitor still did not enumerate as `1209:5e31` in this run.

Detailed history is in:

```text
docs/hpm5e31-bringup-milestones.md
```
