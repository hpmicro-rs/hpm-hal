# HPM5E31 bring-up milestones

Date: 2026-07-17

Board: custom HPM5E31 board
Probe: HSLink-Pro, CMSIS-DAP JTAG
Primary flashing path: local `probe-rs` fork at `/Users/mono/Elec/probe-rs`
HAL example path: `/Users/mono/Elec/HPM/hpm-hal/examples/hpm5e00evk`

Open engineering work, dependency upgrades, and upstream tracking are recorded
in `docs/hpm5e31-backlog.md`.

## Baseline correction

HPM5E support is new in this bring-up. Treat HPM5E11, HPM5E31, and HPM5E3Y as unverified targets until they pass the validation ladder on hardware.

The established implementation baseline is the existing HPM53, HPM67, and HPM6E families. Use those families for HAL/RT conventions such as PLIC priority indexing, vector table layout, resource clock setup patterns, and example structure.

Important current correction:

```text
PLIC priority indexing follows the existing HAL convention:
PLIC.priority(interrupt_number)
```

Keep the temporary `interrupt_number - 1` priority indexing out of the tree. Enable bits and priority writes must address the same external interrupt number.

HPM5E31 currently keeps these RT features enabled:

```text
hpm-riscv-rt/ignore-illegal-mtval-zero
hpm-riscv-rt/pma-noncacheable
```

The PMA/noncacheable setup is part of the expected runtime invariant for USB and DMA-style buffers. Future startup experiments that disable it are temporary experiments and must return to this invariant before integration work.

## Current status

The bring-up has multiple independent layers. The flashing transport layer is partially stabilized. Runtime startup and RTT/defmt have reached `main()` in staged tests. The current blockers are HPM5E-specific GPTMR interrupt validation, USB monitor integration, and target debug/flash algorithm recovery in wedged states.

Current best-known probe-rs invocation:

```sh
/Users/mono/Elec/probe-rs/target/debug/probe-rs run \
  --chip HPM5E31 \
  --chip-description-path /Users/mono/Elec/HPM/hpm-hal/HPMicro.yaml \
  --protocol jtag \
  --probe 0d28:0204:214F44B4FEDADD10BB0B8044E000E80A \
  --speed 10 \
  --connect-under-reset \
  --disable-progressbars \
  --disable-double-buffering \
  --no-location \
  target/riscv32imafc-unknown-none-elf/debug/rtt_defmt_probe
```

## Milestone 1: probe-rs DMI status diagnosis

Observed failure:

```text
INVALID DMI OP status
Timeout during DMI access
The page write of the page at address 0x800xxxxx failed
```

Confirmed cause:

`INVALID DMI OP status` came from the local probe-rs fork printing a single message for every non-OK RISC-V DMI op status. The actual statuses include `Reserved`, `OperationFailed`, and `RequestInProgress`.

Patch location:

```text
/Users/mono/Elec/probe-rs/probe-rs/src/architecture/riscv/dtm/jtag_dtm.rs
```

Applied behavior:

- Replace the raw `println!("INVALID DMI OP status")` with structured `tracing::debug!(?status, ...)`.
- Treat `Reserved` as a real DTM operation failure instead of `unimplemented!`.
- Increase DM register read timeout from 100 ms to 1 s.
- Set minimum DMI idle cycles to 8, max backoff to 64.

Evidence:

After increasing DM read timeout, `probe-rs download` completed successfully:

```text
Finished in 35.269024s
```

## Milestone 2: CMSIS-DAP JTAG sequence batching

Observed failure:

```text
Failed to verify flash algorithm. Data mismatch at address 0x0000028c
The RAM contents did not match the expected contents after loading the flash algorithm.
```

Confirmed cause:

The local fork batched too many CMSIS-DAP JTAG sequences. This produced intermittent RAM readback corruption while loading the flash algorithm.

Patch location:

```text
/Users/mono/Elec/probe-rs/probe-rs/src/probe/cmsisdap/mod.rs
```

Applied behavior:

- Add `PROBE_RS_CMSIS_DAP_JTAG_SEQUENCE_BATCH`.
- Set default batch size to `2`.

Evidence:

Batch scan result after raising DM read timeout:

```text
batch 1: page write DMI timeout
batch 2: Finished in 37.90925s
batch 3: intermittent RAM readback mismatch
```

Current default is `2`.

## Milestone 3: RTT retention in runtime linker script

Observed issue:

RTT control block could be missing or not discoverable after link-time section placement.

Patch location:

```text
/Users/mono/Elec/HPM/hpm-riscv-rt/hpm-link.x
/Users/mono/Elec/HPM/hpm-riscv-rt/link.x
```

Applied behavior:

```ld
EXTERN(_SEGGER_RTT);
PROVIDE(_SEGGER_RTT = 0);
KEEP(*(.uninit .uninit.*));
```

Evidence from ELF:

```text
00200048 D _SEGGER_RTT
```

Evidence from target RAM in one successful startup state:

```text
47474553 52205245 00005454 ...
```

This is the little-endian `SEGGER RTT` header.

## Milestone 4: RT startup trap recorder

Reason:

RTT output can fail before the first defmt frame. A fixed RAM trap recorder gives direct cause data through `probe-rs read`.

Patch location:

```text
/Users/mono/Elec/HPM/hpm-riscv-rt/src/asm.rs
```

Symbol:

```text
__hpm_preinit_trap_record
```

Recorded words:

```text
magic, mcause, mepc, mtval, ra, sp, mtvec, mstatus
```

Magic:

```text
0x48504d54
```

Useful read command:

```sh
/Users/mono/Elec/probe-rs/target/debug/probe-rs read \
  --chip HPM5E31 \
  --chip-description-path /Users/mono/Elec/HPM/hpm-hal/HPMicro.yaml \
  --protocol jtag \
  --probe 0d28:0204:214F44B4FEDADD10BB0B8044E000E80A \
  --speed 10 \
  b32 <__hpm_preinit_trap_record_addr> 8
```

## Milestone 5: RT startup stage recorder

Reason:

The pre-init trap recorder showed faults inside runtime startup. A stage recorder identifies the last completed startup phase.

Patch location:

```text
/Users/mono/Elec/HPM/hpm-riscv-rt/src/lib.rs
```

Symbol:

```text
__hpm_rt_stage
```

Stage values:

```text
0x100: entered _hpm_start_rust
0x110: mstatus FS set
0x120: I-cache enable returned
0x130: D-cache enable returned
0x140: D-cache invalidate returned
0x150: PMA setup returned
0x160: noncacheable sections initialized
0x170: before _setup_interrupts
0x180: after _setup_interrupts
0x190: before main()
```

Confirmed stage:

```text
48504d53 00000190 00000000 00000000
```

Meaning: runtime reached the call to `main()` in one test pass.

## Milestone 6: vector table / interrupt setup check

Observed ELF state:

```text
00000000 R __INTERRUPTS
00000000 R __vector_ram_start__
00000140 R __vector_ram_end__
```

Interpretation:

The generated interrupt vector table is linked at address `0x00000000` in ILM. This placement also appears in HPM5300 and HPM6E builds, so address zero is a valid vector table location for this runtime layout.

Applied correction:

The temporary guard that skipped `_setup_interrupts()` when `__INTERRUPTS == 0` was removed. The runtime now treats the ILM zero vector table as valid.

## Milestone 7: PLIC API issue

Observed trap recorder:

```text
magic    = 0x48504d54
mcause   = 0x00000001
mepc     = 0xe4000000
mtval    = 0xe4000000
ra       = 0xe4000000
mtvec    = 0x800030f8
mstatus  = 0x00003800
```

Meaning:

The core took an instruction access fault while trying to execute from the PLIC base address `0xe4000000`.

Temporary mitigation:

Runtime `_setup_interrupts()` was changed to use direct volatile MMIO helpers for PLIC threshold, claim, interrupt enable, and feature registers.

Required proper fix:

Audit `andes-riscv::plic::PlicExt` usage and decide whether runtime should keep direct MMIO for startup code.

## Milestone 8: HAL BitIter debug overflow

Observed stack evidence:

```text
panic_const_shl_overflow
hpm-hal/src/internal/mod.rs
self.0 &= !(1 << b)
```

Patch location:

```text
/Users/mono/Elec/HPM/hpm-hal/src/internal/mod.rs
```

Applied behavior:

Use `checked_shl` and terminate the iterator if the bit index is outside `0..32`.

Purpose:

Avoid debug-build shift overflow while diagnosing early startup and interrupt/DMA flag paths.

## Milestone 9: I2C detect known-good result

Example:

```text
examples/hpm5e00evk/src/bin/i2c_detect.rs
```

Known-good prior output:

```text
INFO  i2c detect start
INFO  bus: I2C0, scl: PC08, sda: PC09
INFO  cpu0: 600000000Hz
INFO  found 0x77
INFO  i2c detect done, found 1 device(s)
```

Conclusion:

The BME680 is visible on I2C address `0x77` when the runtime and probe session are healthy.

## Milestone 10: BME680 examples

Current files:

```text
examples/hpm5e00evk/src/bin/bme680_chipid.rs
examples/hpm5e00evk/src/bin/bme680_probe.rs
```

Current status:

The examples build. BME680 chip-id and measurement output should be validated after `rtt_defmt_probe` and `i2c_detect` pass in the same probe/flash session.

Next validation order:

1. `rtt_defmt_probe`: prove runtime reaches `main()` and defmt frames are emitted.
2. `i2c_detect`: reconfirm `0x77`.
3. `bme680_chipid`: read register `0xd0`, expected chip id `0x61`.
4. `bme680_probe`: read raw pressure, temperature, humidity, and gas ADC fields.

## Milestone 11: PMA/PMP disable experiment

Temporary experiment:

```text
hpm5e31 feature temporarily removed hpm-riscv-rt/pma-noncacheable
```

Experiment evidence:

Build warnings show the PMA/PMP helper functions are unused:

```text
PMP_RWX_NAPOT is never used
PMA_NAPOT_MEM_NC_BUF is never used
set_pmp_addr0/set_pmp_addr1/write_pmpcfg0/write_pmacfg0 are never used
```

Runtime evidence from OpenOCD SBA reads:

```text
__hpm_rt_stage magic = 0x48504d53
__hpm_rt_stage stage = 0x190
preinit trap magic   = 0x00000000
```

Meaning:

With PMA/PMP disabled in that experiment, runtime reached the final stage before calling `main()` and no pre-init trap was recorded.

Current decision:

The experiment is closed. HPM5E31 keeps `hpm-riscv-rt/pma-noncacheable` enabled so `.noncacheable` USB descriptors, endpoint state, and future DMA-style buffers keep the intended memory attributes.

Current `Cargo.toml` feature state:

```text
hpm5e31 = [
    "hpm-metapac/hpm5e31",
    "hpm-riscv-rt/ignore-illegal-mtval-zero",
    "hpm-riscv-rt/pma-noncacheable",
    "mcan",
]
```

## Milestone 12: current debug/flash state

Probe state:

```text
JTAG tap found: 0x1000563d
hart 0: XLEN=32, misa=0x4094112d
dmstatus observed: 0x00400ca2 and 0x004003a2
```

HPM OpenOCD can enumerate the TAP and examine the hart in some sessions. `probe-rs` can enter the flashing path, then fails in the flash algorithm:

```text
The execution of 'init' failed with code 2147483648
```

HPM OpenOCD flash driver reaches the same layer:

```text
PC ended up at 0x0 instead of 0x4
Failed to execute run algorithm: -4
auto_probe failed
```

Recovery notes:

- Use only one probe process at a time.
- HPM OpenOCD can read Debug Module/SBA data with `hpm5e00.cpu0 configure -defer-examine`.
- Board reset through the probe is unreliable on this custom board. `connect-under-reset` reports that the target is not responding to the reset sequence.
- The next clean validation should start from a manual board reset, then immediately program `main_marker`.

Minimal marker example:

```text
examples/hpm5e00evk/src/bin/main_marker.rs
```

Symbols:

```text
MAIN_MARKER_STAGE            = 0x00200018
__hpm_rt_stage               = 0x00200020
__hpm_preinit_trap_record    = 0x00200030
```

## Milestone 13: main entry and RTT/timer exception

Probe-rs-only validation after manual board reset:

```text
probe-rs download ... target/riscv32imafc-unknown-none-elf/debug/main_marker
probe-rs reset ...
probe-rs read ... b32 0x00200018 14
```

Observed:

```text
MAIN_MARKER_STAGE = 0x00005100
__hpm_rt_stage    = 0x48504d53 0x00000190
trap record       = all zero
```

Meaning:

With PMA/PMP disabled, the runtime reaches Rust `main()`.

RTT probe before function alignment:

```text
RTT output:
INFO  rtt defmt probe start
INFO  CPU0 clock: 600000000Hz
INFO  rtt defmt probe sample 0

stage:
RTT_DEFMT_PROBE_STAGE = 0x00000100
__hpm_hal_init_stage  = 0x00000070
```

Debug backtrace:

```text
DefaultExceptionHandler
_start_rust_CORE_LOCAL
```

Recorded default exception:

```text
magic      = 0x48504d45
mcause     = 0x00000000
mepc       = 0x80003562
mtval      = 0x00000000
trap_frame = 0x0021fe00
sp         = 0x0021fdb0
```

Address mapping:

```text
0x80003562 is the JALR in GptmrDriver::trigger_alarm()
target symbol before fix:
0x8000adee Queue::next_expiration
```

Conclusion:

The timer/embassy path faulted on an indirect call to a 2-byte-aligned function entry. The current example now adds:

```text
-C llvm-args=-align-all-functions=2
```

After rebuilding:

```text
0x8000b3c8 Queue::next_expiration
```

The indirect call target is now 4-byte aligned.

Probe-rs RTT validation after rebuilding:

```text
probe-rs run ... --disable-double-buffering --no-location \
  target/riscv32imafc-unknown-none-elf/debug/rtt_defmt_probe
```

Observed RTT output:

```text
INFO  rtt defmt probe start
INFO  CPU0 clock: 600000000Hz
INFO  rtt defmt probe sample 0
...
INFO  rtt defmt probe sample 15
INFO  rtt defmt probe done
```

Final stage:

```text
RTT_DEFMT_PROBE_STAGE = 0x00000200
__hpm_hal_init_stage  = 0x00000090
__hpm_rt_stage        = 0x48504d53 0x00000190
```

Meaning:

RTT/defmt and the current embassy timer interrupt path pass in this probe example.

## Current blockers

1. `.uninit` trap record can contain stale data after a successful run and should be cleared at startup for cleaner diagnostics.
2. probe-rs `download -> reset -> read` is reliable enough for staged validation.
3. probe-rs `run` can print RTT for this example; `attach` still needs separate verification.
4. probe-controlled reset reports `HaltResume` during shutdown, but the reset action itself can succeed.

## Milestone 14: sustainable validation gate

Purpose:

Keep new HPM5E31 work on a fixed validation ladder so one failing integrated example does not reopen every lower layer.

Baseline bins:

```text
main_marker
rtt_defmt_probe
i2c_detect
i2c_sensor_bme680
monitor_usb_lite
monitor_blinky
```

Build gate:

```sh
cd /Users/mono/Elec/HPM/hpm-hal/examples/hpm5e00evk
./monitor.sh build-baseline
```

Layer ownership:

```text
main_marker          RT entry
rtt_defmt_probe     RTT/defmt and embassy timer
i2c_detect          I2C0 PC08/PC09 bus
i2c_sensor_bme680   BME680 sensor access and measurements
monitor_usb_lite    minimal USB monitor transport
monitor_blinky      integrated GPIO monitor state
```

Decision boundary:

Only `main_marker` or `rtt_defmt_probe` regression justifies startup/RT investigation. When these pass, failures in BME680, USB monitor, or web bridge stay in their owning layer.

Current monitor integration note:

The attempted integrated `monitor_blinky` extension added BME680 monitor symbols, sensor fixed-point fields, and USB bulk/web plumbing. Runtime validation showed the integrated image can fail before the BME680 task starts and before monitor USB enumeration. Since standalone BME680 and RTT baselines have already passed, the next durable step is to validate `monitor_usb_lite`, then add one integration feature at a time.

## Milestone 15: probe-rs workflow hardening and USB monitor fault signature

Date: 2026-07-12

Reason:

The board entered a state where `probe-rs download --connect-under-reset` printed no progress and waited indefinitely. This created an operational dependency on manual reset. The reusable workflow now bounds flash/reset operations and records the target state through fixed RAM markers.

Script updates:

```text
examples/hpm5e00evk/monitor.sh
```

Reusable behavior:

- Default `PROBE_RS` is `/Users/mono/Elec/probe-rs/target/debug/probe-rs`.
- Default `FLASH_TOOL` is `probe-rs`.
- `flash` runs `probe-rs download` with `FLASH_TIMEOUT`, then calls `reset_target` after a successful download.
- Failed or timed-out `flash` attempts reset cleanup and returns failure.
- `reset` runs probe-rs reset with `PROBE_RS_TIMEOUT`.
- `build-baseline` builds the known validation ladder.
- `rtt` uses `probe-rs run` with `--no-location`.
- Python monitor tooling runs through `uv run --with pyserial --with pyusb python`.

Current command entry points:

```sh
cd /Users/mono/Elec/HPM/hpm-hal/examples/hpm5e00evk
BIN=monitor_usb_lite ./monitor.sh build
BIN=monitor_usb_lite ./monitor.sh flash
./monitor.sh reset
BIN=rtt_defmt_probe ./monitor.sh rtt
BIN=monitor_usb_lite ./monitor.sh smoke-bulk
./monitor.sh web
```

Observed bounded-flash result in the bad target state:

```text
Flashing target/riscv32imafc-unknown-none-elf/debug/monitor_usb_lite
Probe: 0d28:0204:214F44B4FEDADD10BB0B8044E000E80A, speed: 10kHz
probe-rs download failed or timed out after 15s; attempting reset cleanup
WARN probe_rs::session: Failed to deconfigure device during shutdown: HaltResume
```

Interpretation:

The script layer now converts the hang into a bounded failure. The remaining stability issue is in the probe-rs/debug-module recovery path for this target state.

Reset command result in the same state:

```text
./monitor.sh reset
WARN probe_rs::session: Failed to deconfigure device during shutdown: HaltResume
```

Memory read still works after the reset warning, so this warning alone is not a reliable failure signal. Stage markers remain the decision source.

USB monitor diagnostic updates:

```text
examples/hpm5e00evk/src/bin/monitor_usb_lite.rs
```

Applied reusable diagnostics:

- Removed `panic_halt` in this example and added a local `#[panic_handler]`.
- Added `MON_PANIC_LINE` and `MON_PANIC_COLUMN`.
- Added 4-byte-aligned wrapper buffers for config, BOS, MSOS, and control descriptors.
- Added a real MSOS descriptor buffer instead of an empty slice.

Current USB setup stage map:

```text
0x200 entered main
0x210 hpm_hal::init returned
0x211..0x214 USB0 direct pinmux writes
0x220 UsbDriver::new returned
0x230 embassy_usb::Config prepared
0x240 before Builder::new
0x250 Builder::new returned
0x260 CdcAcmClass::new returned
0x270 builder.build returned
0x280 usb.run future entered
0x290 monitor future entered
0x2a0 waiting for CDC connection
```

Fault observed before adding the aligned descriptor/MSOS diagnostic update:

```text
MON_LAST_ERROR = 0x00000260
mcause         = 0x00000001
mepc           = 0x8023abb2
mtval          = 0x8023abb2
ra             = 0x8023abb2
trap_frame     = 0x0021e8d0
```

Address mapping:

```text
0x8023abb2 -> no ELF symbol, outside the linked text range
```

Interpretation:

The image reached `CdcAcmClass::new` and faulted before `builder.build()` returned. The fault signature is an instruction access fault through a corrupted return address or task frame. Continue USB work from `monitor_usb_lite`; keep the RT and RTT baselines closed unless `main_marker` or `rtt_defmt_probe` regresses.

State after the timed-out flash/reset attempt:

```text
preinit trap magic = 0x48504d54
mcause             = 0x00000002
mepc               = 0x80029870
mtval              = 0xffffffff
mtvec              = 0x800030f8
```

Interpretation:

The target moved into an early RT trap after the failed download/reset sequence. Use BOOT0 or hardware reset as the recovery mechanism for this wedged state, then retry the bounded `monitor.sh` flow.

## Milestone 16: HPM5E31 flash algorithm mismatch risk

Current target and ROM state:

```text
HPM5E31 HPMicro.yaml flash algo   = flash-algo-hpm5300
ROM_API_TABLE_ROOT->version       = 0x56010200
xpi_nor_driver_if->version        = 0x56010300
```

SDK comparison:

```text
boards/hpm5e00evk/board.c        { 0xfcf90002, 0x00000005, 0x1000, 0x0 }
boards/hpm5301evklite/board.c    { 0xfcf90002, 0x00000005, 0x1000, 0x0 }
```

The board-level NOR option is shared with HPM5301. The HPM5E-specific requirement is in the ROM/XPI programming path: SDK `samples/openocd_algo/src/openocd_flash_algo.c` checks `xpi_nor_driver_if->version >= 0x56010300`, calls `enable_hybrid_xpi(xpi_base)`, and converts erase/program/read offsets into absolute flash addresses by adding `flash_base`.

Current local Rust `flash-algo` state:

```text
/Users/mono/Elec/HPM/flash-algo
Cargo.toml has an unresolved merge conflict
src/main.rs currently subtracts FLASH_BASE before erase/program
HPM5E-specific romapi module is absent
```

Working hypothesis: the partial flash image is explained by using the HPM5300-style algorithm on HPM5E31. The first flash region can program, while later sectors remain erased and reset reaches `mtval = 0xffffffff`. The next implementation step is to create an HPM5E31 flash algorithm that follows the SDK hybrid XPI behavior, bind it as `flash-algo-hpm5e31` in `HPMicro.yaml`, then recover with BOOT0 and run full program/readback verification.

## Milestone 16 follow-up plan (historical)

This plan records the state at Milestone 16. Milestone 17 below supersedes its
USB, runtime, clock, and integrated-validation items.

1. Keep all validation on probe-rs for this board.
2. Use `monitor.sh` as the primary validation loop so flash/reset/RTT/web paths share the same probe settings.
3. Keep the 4-byte function alignment flag while testing embassy timer paths.
4. Clear the `.uninit` trap record at startup to avoid stale diagnostics.
5. Keep the trap and stage recorders until `rtt_defmt_probe`, `i2c_detect`, and `i2c_sensor_bme680` pass sequentially.
6. Continue USB monitor validation in `monitor_usb_lite` from the `0x260 -> builder.build()` fault boundary.
7. Commit probe-rs DMI/CMSIS-DAP fixes separately from hpm-riscv-rt startup fixes.

## Milestone 17: HPM5E clock and USB monitor stability closure (2026-07-16)

HPM5E clock configuration now follows the official C SDK board setup. Register
readback after initialization confirms:

```text
PLL0 MFI/MFN       = 40/0
PLL0 DIV0/DIV1     = 5/7
PLL0 VCO           = 960 MHz
PLL0 CLK0          = 480 MHz
PLL0 CLK1          = 400 MHz
DCDC voltage       = 1275 mV
```

The stable initialization sequence raises DCDC and waits for readiness, moves
the CPU to the 24 MHz oscillator, programs PLL0 and both dividers while they are
disabled, then selects PLL0 as the CPU source. This matches the SDK's divider
enable semantics and removes the previous 600 MHz/1200 mV operating point.

Four independent USB/runtime defects are closed:

1. Runtime executes `fence.i` immediately after enabling L1I, synchronizing
   instruction fetch after flash programming and reset.
2. The external interrupt wrapper executes from ILM `.fast.text` and preserves
   the HPM/Andes interrupt context used by the C SDK.
3. USB qTD publication and retirement use full-width descriptor writes,
   `fence iorw`, ISR-owned W1C completion, and generation-based wakeups.
4. SETUP handling keeps the global USB interrupt enable active. EP0 is woken
   directly, so a control-transfer direction mismatch cannot hide EP1 OUT
   completion and deadlock the monitor transport.

Standalone `monitor_usb_lite` release validation after a full verified flash and
PPOR reset:

```text
10/10 requests             PASS
1000/1000 requests         PASS in 15.093 s
timeout/CRC/header errors  0
exception mcause/mepc      0/0
post-PPOR 100/100          PASS in 1.511 s
```

The integrated `monitor_sensors` release image runs three I2C sensor tasks,
PMS7003 UART receive, and USB monitor concurrently. A 30-cycle host stress run
completed in 33.963 s with every request successful. During the run I2C cycles
advanced from 25 to 54, valid PMS frames from 35 to 74, and USB interrupt count
from 777 to 2691. Monitor, UART, and exception error records remained clear.

The board is left running the verified `monitor_sensors` image. This closes the
USB monitor stability milestone and establishes the asynchronous sensor example
as the hardware regression baseline.

## Milestone 18: Durable repair boundaries (2026-07-17)

The stable fixes were separated from bring-up instrumentation:

- Runtime cache and interrupt invariants remain in `hpm-riscv-rt`.
- PLIC setup uses the typed `andes-riscv` register API and direct writes to the
  side-effectful CLAIM/COMPLETE register.
- HPM5E clock policy and safe PLL switching remain in `sysctl/v6e.rs`.
- USB DCD ownership, descriptor ordering, W1C completion, generations, and
  retirement remain in the shared USB driver.
- USB snapshots are available through the opt-in `usb-diagnostics` feature.
- Retained early-trap recording is available through the opt-in
  `runtime-diagnostics` feature.
- Protocol, CSR, panic, exception, and sensor traces remain in qualification
  examples.

HPM5E `Config::default()` now carries the SDK-qualified PLL0 parameters:

```text
VCO frequency = 960 MHz
postdiv codes = [5, 7]
CLK0/CLK1     = 480/400 MHz
```

The safe HPM5E PLL transition consumes these parameters and calculates the
runtime clock table from the configured VCO and divider codes. Normal HAL builds
compile without USB snapshot I/O or retained trap recording.

Validation after cleanup:

```text
HPM5E production feature check    PASS
HPM6E80 cross-family check        PASS
monitor_usb_lite flash/verify     PASS
monitor_usb_lite stress           1000/1000 in 15.081 s
monitor_sensors flash/verify      PASS
monitor_sensors stress            30/30 in 34.683 s
I2C cycle                         10 -> 39
PMS valid frames                  14 -> 54
USB IRQ                           769 -> 2683
protocol/panic/exception errors   0
```

The board is left running the post-cleanup `monitor_sensors` release image.
