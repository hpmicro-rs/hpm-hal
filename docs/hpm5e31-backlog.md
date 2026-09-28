# HPM5E31 bring-up backlog

Updated: 2026-07-19

This document tracks unresolved engineering work discovered during HPM5E31
bring-up and upstream research. Verified results belong in
`hpm5e31-bringup-milestones.md`; this file records remaining actions, upstream
changes, migration work, risks, and acceptance criteria.

Priority meanings:

- P0: blocks a supported build or the validation ladder.
- P1: required before the HPM5E implementation is considered stable.
- P2: maintenance, diagnostics, or follow-up alignment.

## P0: verify every flash download by readback or runtime proof

### Finding

The first 2026-07-13 download of the BH1750 release example reported success,
while target flash contained valid ELF words only through `0x800038a7` and
returned `0xffffffff` from `0x800038a8` onward. The CPU trapped when it reached
the missing code. A second identical `probe-rs run` programmed the complete
image and produced valid RTT output.

The HPM5E31 target description was still bound to `flash-algo-hpm5300`. The
dedicated HPM5E31 algorithm uses hybrid XPI, absolute flash addresses, and a
64-KiB erase block. It is now present in the local target description.

Large downloads also exposed an independent probe-rs transport limit. A
system-bus write queued every word in one DMI command batch and consistently
timed out after about 24 KiB. Bounded 256-word batches with an `SBCS` status
check completed a 64-KiB RAM load and ROM program operation. A fresh probe
attach read back matching image header and entry words. Same-session verify
still reports a mismatch, so first-address reporting and cache/XPI
reinitialization remain part of qualification.

A 2026-07-13 no-reset qualification used
`PROBE_RS_FLASH_SKIP_RESET=1`. probe-rs halted the existing target state,
entered the HPM5E31 algorithm for erase and program, and completed the full
download in 6.78 seconds. The algorithm now masks machine interrupts, disables
and invalidates L1 caches, resets the XPI command engine, runs ROM auto-config,
and issues memory/instruction fences on exit. This records one successful
no-reset attempt while DMI halt/register/resume access remained operational;
repeat qualification and state restoration remain open.

A 2026-07-14 controlled retry after a user-performed physical reset built the
`pms7003_uart3` release+LTO ELF and ran exactly one `monitor.sh flash` operation.
Download returned `Finished in 6.838366s`; the scripted reset reported
`HaltResume`. Runtime inspection recorded an instruction-access-fault with
`mepc=mtval=0x00203474`, while the ELF exception table at `0x80005d90` contains
`0x800052a0`. A later read-only debugger session read all 16 mapped table words
from `0x80005d90..0x80005dcc`; every word matched the ELF. This reproduces the
post-flash execution failure with correct mapped Flash content and narrows the
remaining defect to CPU/XPI/cache/reset state restoration. The operation was
stopped after the single download; UART and RTT validation were intentionally
skipped.

The direct reset-path defect was then found in the local probe-rs fork:
`probe-rs/src/vendor/hpmicro/` existed as untracked source, while
`probe-rs/src/vendor/mod.rs` neither declared nor registered the HPMicro vendor.
The module had never compiled; registering it initially exposed a private
`read_csr` call in its diagnostic-only `on_connect()` implementation. Every
earlier HPM5E operation therefore selected `DefaultRiscvSequence` and performed
only a generic hart reset.

The vendor is now registered only for `HPM5E*` targets, preserving the existing
paths for HPM53/HPM67/HPM6E. Its reset sequence mirrors the SDK OpenOCD
`hpm5e00.cfg::reset_soc` operation by writing `24000000` to
`PPOR.SOFTWARE_RESET` at `0xF410001C`, waiting for the 24 MHz countdown and then
halting the restarted hart. A reset of the already-programmed PMS7003 image
cleared all trap words and reached runtime stage `0x190` without another flash
download. RTT reached UART3 initialization. This validates PPOR recovery for one
image; alternating-image stress qualification remains open.

Deep transport research on 2026-07-14 found that the local fork is based on
probe-rs v0.24.0 while upstream has reached v0.31.0. Upstream commit
`bd10f986fd49622415196ab8d772de22bd5e726c` (2026-02-24, PR #3870) fixes a
RISC-V system-bus race and states that the old code could return arbitrary data
or make writes have no effect. The patch clears and checks sticky
`SBCS.sbbusyerror` and replays the affected operation. It has been backported to
the local single-read, multiple-read, and multiple-write SBA paths. The RISC-V
unit tests pass 4/4.

The RISC-V Debug Specification also requires the debugger to wait for
`SBCS.sbbusy=0` before writing SBCS. Current upstream code documents this rule
but still writes the new SBCS configuration directly. The local fork now adds
an idle barrier before configuration and after each transfer batch, with a
bounded timeout. The local HPM SDK OpenOCD `sba_read_mem`/`sba_write_mem`
helpers now apply the same barrier and report sticky busy/bus errors.

The HPM reset path contained a second independent defect. The HPM5E PPOR C
sample enables `RESET_ENABLE.bit31` before writing the software-reset counter.
The SDK OpenOCD `hpm5e00.cfg::reset_soc` and the first local probe-rs HPM
sequence wrote only `PPOR.SOFTWARE_RESET=24000000`. On the current target this
left the counter access permanently busy. Cross-tool evidence from the same
CherryUSB CMSIS-DAP 2.1.1 probe was:

```text
JTAG TAP ID   = 0x1000563d
dmstatus      = 0x00400ca2 (hart running; halt request not accepted)
SBCS          = 0x20640407 (sbbusy=1, sbbusyerror=1)
SBADDRESS0    = 0xF410001C (PPOR.SOFTWARE_RESET)
```

JTAG, DTM, and Debug Module reads remained available while all SBA traffic and
software PPOR recovery were blocked. An effective board reset or power cycle is
required once for this pre-fix state. The corrected sequence enables the
software reset source, selects cold reset, sets reset-halt catch, triggers the
counter, waits for a halted hart, and clears reset catch.

The HPM5E flash algorithm now uses a 4-KiB program page while retaining the
64-KiB erase block. probe-rs reads every RAM page buffer back before the ROM
algorithm consumes it and performs bounded retransmission. This caught word
corruption in target RAM before NOR programming. After the upstream
busy-error patch, one mismatch still appeared with a 256-word batch and passed
after retransmission. A second independent confirmation read has been added to
classify the remaining event as read-side transient or stable RAM corruption.

### Actions

- [ ] Finish full-range program/readback qualification of the HPM5E31 flash
      algorithm.
- [ ] Qualify the 256-word probe-rs system-bus write batching change across
      ILM, DLM, and XPI downloads.
- [x] Backport upstream probe-rs PR #3870 `sbbusyerror` replay handling.
- [x] Add an `sbbusy=0` barrier around local probe-rs and HPM OpenOCD SBA
      transactions.
- [x] Enable the HPM5E PPOR software-reset source and use reset-halt catch.
- [ ] After one physical reset/power cycle, verify that corrected PPOR reset
      leaves SBCS idle across repeated flash/reset transactions.
- [ ] Classify every page-buffer mismatch with an independent confirmation
      read before retransmission.
- [ ] Reset/reinitialize XPI before verify and record the first expected/actual
      byte mismatch.
- [ ] Promote the no-reset path from an environment-gated diagnostic after
      repeated erase/program/readback qualification.
- [ ] Distinguish SBA-only attach from full core-control attach in probe-rs
      diagnostics; flash algorithms require halt, DPC/register writes, and
      resume in addition to memory reads.
- [ ] Add an optional post-download ELF segment verification step to
      `monitor.sh`.
- [ ] Treat `Finished` as transport completion; require runtime magic, marker,
      RTT, or flash readback as application proof.

### Acceptance criteria

- Repeated downloads of small and large debug/release ELFs pass full programmed
  range readback.
- `monitor.sh` reports the first mismatching address and fails the operation.
- Ten alternating small/large images complete algorithm execution, full-range
  verify, PPOR reset-halt, runtime proof, and leave `SBCS.sbbusy=0`.

## P0: make unoptimized initialization reliable

### Finding

The HPM5E `sysctl::init()` path failed in the default Cargo dev profile. The
fault record contained an instruction access fault with `ra` and `mepc` in the
DLM stack region. The same firmware passed with `opt-level=1`.

Measured code-generation difference:

```text
dev opt-level=0: sysctl::init stack frame = 928 bytes before recorder change
dev opt-level=1: sysctl::init has no large stack frame
```

The dense `mark_hal_init_stage()` calls were implemented as `inline(always)`
inline assembly. This makes the unoptimized caller retain many operands and
temporaries. Changing the recorder to an out-of-line leaf function reduced the
O0 frame to 720 bytes; hardware validation of that O0 build is still pending.

### Upstream comparison

- Embassy's official examples document `cargo run --release --bin ...` as the
  normal hardware workflow and retain debug information in release profiles.
- Espressif's official Rust template sets the dev profile to `opt-level = "s"`
  with the explanation that unoptimized Rust is too slow.
- Embassy vendor HAL initialization paths keep core initialization compact;
  diagnostics live in defmt, panic/backtrace support, or dedicated examples.

Sources:

- https://github.com/embassy-rs/embassy
- https://github.com/embassy-rs/embassy/blob/main/examples/nrf52840/Cargo.toml
- https://github.com/embassy-rs/embassy/blob/main/examples/rp/Cargo.toml
- https://github.com/esp-rs/esp-template/blob/main/Cargo.toml

### Actions

- [x] Flash and run the out-of-line recorder build at `opt-level=0`.
- [x] Keep the stage recorder out of line and document its code-generation
      requirement.
- [ ] Reduce permanent HAL stages to coarse phase boundaries; keep fine-grained
      SYSCTL diagnosis in `sysctl_marker`.
- [ ] Add a dedicated `on-target` Cargo profile with optimization and debug
      information, then make `monitor.sh` select it explicitly.
- [ ] Keep an O0 build in CI to ensure compilation and inspect frame sizes;
      reserve hardware qualification for the supported on-target profile.

### Acceptance criteria

- `main_marker`, `rtt_defmt_probe`, `i2c_detect`, and
  `i2c_sensor_bme680` pass using the supported on-target profile.
- The O0 `i2c_sensor_bme680` image reaches at least stage `0x05`, or the O0
  profile is explicitly documented as compile/debug-only with a reproducible
  diagnostic.
- Trap record remains clear during a 30-second BME680 RTT run.

2026-07-13 result: the O0 image now completes `hal::init()`
(`__hpm_hal_init_stage=0x90`) with a clear trap record. It reaches BME680 stage
`2`; the first defmt/I2C-construction interval still needs O0-specific
diagnosis. The release image completes continuous BME680 sampling.

## P0: unify the RISC-V dependency and critical-section provider

### Finding

The dependency graph currently contains two `riscv` versions that both enable
`critical-section-single-hart`:

```text
hpm-hal       -> riscv 0.15
hpm5e example -> riscv 0.15
hpm-riscv-rt  -> riscv 0.16
```

Release LTO reports duplicate global definitions:

```text
_critical_section_1_0_acquire: symbol multiply defined
```

Embassy HALs commonly expose an explicit critical-section implementation
feature, ensuring the final application selects one provider.

Source:

- https://github.com/rust-embedded/critical-section
- https://github.com/embassy-rs/embassy/blob/main/examples/rp/Cargo.toml

### Actions

- [x] Upgrade `hpm-hal` and HPM examples to the same `riscv` release used by
      `hpm-riscv-rt`.
- [x] Define ownership of `critical-section-single-hart` in one crate instance.
- [ ] Consider an explicit `critical-section-impl` HAL feature following the
      Embassy RP pattern.
- [ ] Run `cargo tree -d` and record all remaining duplicated architecture
      crates.

### Acceptance criteria

- Release+LTO links without duplicate `_critical_section_1_0_*` symbols.
- Exactly one critical-section implementation is present in the final ELF.
- GPIO, Embassy timer, I2C, USB, and interrupt tests pass after the migration.

2026-07-13 result:

```text
hpm-hal/example riscv: 0.15.0 -> 0.16.1
hpm-hal andes-riscv:   0.1.x  -> 0.3.0
release + fat LTO:     passed
```

`riscv-rt 0.16.0` still brings `riscv 0.15.0` through its upstream dependency.
That instance does not own the HPM application's
`critical-section-single-hart` feature. Track its removal as an upstream
dependency deduplication item.

## P1: define supported Cargo profiles for hardware

### Finding

The current scripts implicitly use Cargo's default dev profile. Embedded Rust
projects often run optimized firmware while retaining debug symbols. Embassy's
official workflow uses `--release`; the Espressif template optimizes both dev
and release builds.

### Proposed profiles

```toml
[profile.dev]
debug = 2

[profile.on-target]
inherits = "dev"
opt-level = 1
debug = 2
incremental = false

[profile.release]
opt-level = "z"
debug = 2
codegen-units = 1
lto = "fat"
```

The final settings should follow measured flash time, code size, RTT behavior,
and debugging quality rather than copying another vendor unchanged.

### Actions

- [ ] Add the custom profile after resolving the critical-section LTO failure.
- [ ] Teach `monitor.sh` to derive the ELF path from `PROFILE`.
- [ ] Build the full validation ladder under dev, on-target, and release.
- [ ] Record ELF size and maximum observed stack use for each profile.

### Acceptance criteria

- `monitor.sh` prints the selected profile and exact ELF before flashing.
- The default hardware workflow uses the qualified on-target profile.
- Debug symbols remain available to probe-rs/defmt in qualified builds.

## P1: make stack sizing explicit and measurable

### Finding

`hpm-riscv-rt` declares `_stack_size = 0x4000`, while its `.stack` section grows
from the end of BSS/heap to the top of the complete DLM region. In the current
BME680 image this produces a 128,780-byte stack region:

```text
_estack = 0x002008f4
_sstack = 0x00220000
```

The HPM C SDK allocates a fixed 16 KiB stack and a fixed 16 KiB heap by default.
The current Rust layout provides ample capacity, but `_stack_size` does not
control the linked section and there is no guard/canary.

### Actions

- [ ] Decide whether `_stack_size` is a fixed reservation or a minimum.
- [ ] Make the linker script and assertion implement that definition.
- [ ] Add stack fill/canary support for bring-up builds.
- [ ] Record high-water usage after RTT, I2C, USB CDC, vendor bulk, and Web
      bridge validation.
- [ ] Preserve 16-byte stack alignment at entry and interrupt boundaries.

### Acceptance criteria

- Linker map reports an intentional stack size.
- Startup detects canary corruption and records a distinct trap/diagnostic code.
- Maximum measured use has documented headroom for nested interrupts.

## Complete: align HPM5E clock configuration with the C SDK

### Finding

The C SDK board setup initializes PLL0 to 960 MHz, applies post-divider 2.0,
and selects CPU divider 1, giving a 480 MHz CPU clock. The old HAL reused the
HPM6E defaults and the board readback confirmed 600 MHz at 1200 mV.

During USB interrupt pressure, that uncontrolled 600 MHz/1200 mV state produced
XPI execution-flow faults whose PCs acquired extra high address bits. The HAL
now establishes the official HPM5E operating point independently of ROM state.

### Actions

- [x] Choose 480 MHz as the supported custom-board CPU frequency.
- [x] Raise DCDC to 1275 mV before selecting the fast clock.
- [x] Switch CPU to 24 MHz while programming PLL0 VCO=960 MHz, DIV0=5 and DIV1=7.
- [x] Set HPM5E `CLOCKS` to the programmed 480/400 MHz outputs.
- [x] Validate register readback, USB monitor timing and PPOR cold start.

### Acceptance criteria

- HAL-reported CPU/AHB clocks match register read-back and the chosen board
  configuration.
- Timing measurements agree within documented tolerance.

2026-07-16 readback and validation:

```text
PLL0 MFI=40, MFN=0, DIV0=5, DIV1=7
PLL0_CLK0=480 MHz, PLL0_CLK1=400 MHz
DCDC=1275 mV
USB monitor=1000/1000; PPOR then 100/100; exception=0
```

## P1: Embassy dependency upgrade audit

### Finding

The HPM5E example currently uses:

```text
embassy-executor 0.9.1
embassy-sync     0.7.2 (through hpm-hal)
embassy-time     0.5.0
embassy-usb      0.5.1
```

Embassy upstream `main` examples observed during this investigation use newer
lines including executor 0.10, sync 0.8, time 0.5.1, and USB 0.6. Treat these as
upgrade candidates; upstream `main` is evidence for an audit, not an automatic
version bump.

### Actions

- [x] Check current crates.io releases and Embassy upstream manifests.
- [x] Review executor platform feature-name changes.
- [x] Upgrade sync, executor, and USB to the selected compatible releases.
- [x] Re-run the full HPM5E compile validation ladder.
- [ ] Review timer queue, raw waker, USB driver, and critical-section API changes.

### Acceptance criteria

- Dependency versions are documented and intentionally pinned.
- No duplicated Embassy core crates remain unless justified.
- Timer and USB behavior passes on hardware after each accepted upgrade.

2026-07-13 selected versions and results:

```text
embassy-executor           0.9.1 -> 0.10.0
embassy-executor-macros    0.7.0 -> 0.8.0
embassy-sync               0.7.2 -> 0.8.0
embassy-usb                0.5.1 -> 0.6.0
embassy-net-driver-channel 0.3.2 -> 0.4.0
usbd-hid                   0.8.2 -> 0.10.0
```

- The executor feature migrated from `arch-riscv32` to `platform-riscv32`.
- `monitor_blinky` migrated to the executor 0.10 fallible task-token API.
- The full dev validation ladder compiles.
- BME680 release+LTO runs on hardware and produced 22 continuous RTT samples.
- `monitor_usb_lite` release compiles. Its first post-upgrade download did not
  reach runtime magic after reset, so USB enumeration remains pending target
  recovery and a clean startup proof.
- `monitor_usb_lite` now contains a Full-Speed vendor bulk baseline with
  64-byte endpoints and a WinUSB MS OS descriptor.
- The HPM5E31 core is `RV32-IMAFDCPB`; the example target enables `+d`, so the
  `riscv-interrupt-m` USB IRQ wrapper legitimately uses `fsd`/`fld` context
  saves.
- probe-rs reset currently exits with `HaltResume`. An in-session reset places
  PC at `0x80003000`; the detach path can leave the target halted or at the
  early trap address. USB enumeration remains blocked until cold-reset/run is
  stable.

## P2: target-feature and function-alignment qualification

### Finding

The HPM5E examples currently enable:

```text
zba, zbb, zbc, zbs, d, zfh
```

and force 4-byte function alignment with:

```text
-C llvm-args=-align-all-functions=2
```

The alignment flag fixed a real indirect-call failure in the Embassy timer
path. Rust warns that `d` and `zfh` target features are unstable. The exact
HPM5E core ISA and LLVM support need a documented compatibility matrix.

### Actions

- [ ] Read `misa` and relevant Andes feature CSRs on HPM5E31.
- [ ] Compare the result with the C SDK `-march` string.
- [ ] Test each extra target feature independently.
- [ ] Retain 4-byte function alignment until indirect calls pass without it
      across debug and optimized profiles.
- [ ] Track Rust/LLVM upstream changes affecting RISC-V target features and
      compressed indirect-call targets.

### Acceptance criteria

- The selected target-feature set matches hardware and compiler support.
- Every enabled extension has an on-target instruction test or SDK evidence.
- Function-alignment policy is documented with reproducible evidence.

## P2: upstream monitoring

Track these upstream projects before dependency refreshes:

- Embassy releases and migration notes:
  https://github.com/embassy-rs/embassy/releases
- Rust RISC-V target/codegen issues:
  https://github.com/rust-lang/rust/issues?q=is%3Aissue%20state%3Aopen%20RISC-V
- LLVM RISC-V backend issues:
  https://github.com/llvm/llvm-project/issues?q=is%3Aissue%20state%3Aopen%20RISC-V
- riscv and riscv-rt releases:
  https://github.com/rust-embedded/riscv/releases
- critical-section releases:
  https://github.com/rust-embedded/critical-section/releases
- probe-rs RISC-V changes:
  https://github.com/probe-rs/probe-rs/releases

For every adopted update, record:

1. old and new version/commit;
2. relevant upstream issue or release note;
3. local files changed;
4. build matrix result;
5. on-target validation result;
6. rollback point.

### 2026-07-14 dependency divergence audit

Remote HEADs were queried directly on 2026-07-14. The commit counts below use
`remote_default_branch...local_HEAD`; `behind` is remote-only history and
`ahead` is local-only committed history. Working-tree changes are listed
separately because they are not represented by the commit counts.

| Repository / comparison | Behind | Ahead | Working tree |
| --- | ---: | ---: | --- |
| `hpm-hal` vs `hpmicro/hpm-hal:master` | 0 | 2 | 18 tracked, 2512 untracked |
| `hpm-hal` vs `hpmicro-rs/hpm-hal:master` | 58 | 2 | same working tree |
| `hpm-data` vs `hpmicro/hpm-data:main` | 0 | 1 | 14 untracked generated files |
| `hpm-riscv-rt` vs `hpmicro-rs/hpm-riscv-rt:main` | 0 | 1 | 4 tracked |
| `andes-riscv` checkout vs `hpmicro-rs/andes-riscv:main` | 5 | 0 | clean |
| `flash-algo` vs `hpmicro-rs/flash-algo:main` | 1 | 0 | 8 staged, 2 unstaged, 9 untracked |
| `hpm_sdk` vs `hpmicro/hpm_sdk:main` | 0 | 0 | 3 tracked, 1901 untracked sample files |
| `probe-rs` vs `hpmicro-rs/probe-rs:hpmicro` | 0 | 0 | 8 tracked, 36 untracked |
| `probe-rs` vs `probe-rs/probe-rs:master` | 970 | 10 | same working tree |
| local Embassy reference vs `embassy-rs/embassy:main` | 1659 | 0 | 1 untracked |
| local RISC-V reference vs `rust-embedded/riscv:master` | 21 | 0 | clean |

Important interpretation:

- `hpm-hal` currently follows the older `hpmicro/hpm-hal` line plus two local
  HPM5E commits. The more active `hpmicro-rs/hpm-hal` line has 58 remote-only
  commits, including dependency, peripheral, and cache-management changes.
- The active PAC and runtime are local path dependencies. Their HPM5E support
  lives in one local commit each plus uncommitted runtime/HAL changes.
- The local `andes-riscv` checkout is not used by the current HPM5E Cargo
  graph; Cargo resolves crates.io `andes-riscv 0.3.0`, which matches its current
  main manifest version.
- The local probe-rs fork is version `0.24.0`. Official master is version
  `0.31.0` and has 970 remote-only commits. The current HPM reset, SBA, and
  page-buffer verification work exists as uncommitted changes on the old HPM
  fork, so a direct upgrade requires a structured rebase/port and hardware
  regression matrix.
- The current example lockfile has no git-source packages. Embassy direct
  versions (`executor 0.10.0`, `sync 0.8.0`, `time 0.5.1`, `usb 0.6.0`, and
  `embedded-hal 0.6.0`) match the versions declared on current Embassy main.
  `embassy-hal-internal` resolves as both `0.3.0` (direct HPM HAL dependency)
  and `0.4.0` (through `embassy-embedded-hal`), while current Embassy main
  declares `0.5.0`.
- The active `riscv 0.16.1` matches current rust-embedded/riscv master. The
  generated `hpm-metapac` still enables `riscv-rt 0.16.0`, which brings
  `riscv 0.15.0`; this duplicate RISC-V dependency line remains an explicit
  migration item.

Recommended integration order:

1. Preserve the verified HPM5E baseline in focused commits across `hpm-data`,
   `hpm-riscv-rt`, `flash-algo`, `hpm-hal`, and probe-rs.
2. Port probe-rs HPM support onto official `0.31.x` in a separate branch and
   run cross-family RISC-V plus alternating-image HPM5E hardware tests.
3. Rebase `hpm-hal` onto the 58-commit `hpmicro-rs/master` delta, resolving
   Embassy/internal and cache-management changes as independent commits.
4. Collapse the Embassy internal and RISC-V runtime duplicate versions after
   the hardware baseline passes on the rebased branches.

## Hardware inventory: I2C0 sensors

2026-07-13 on-target scan of I2C0 (`PC08=SCL`, `PC09=SDA`) found three
7-bit addresses: `0x23`, `0x39`, and `0x77`.

- `0x23`: BH1750 ambient-light sensor module.
- `0x39`: APDS-9930 ambient-light and proximity sensor. The read-only
  identification registers returned `REV=0x02` at command `0x91` and
  `ID=0x39` at command `0x92`, matching the APDS-9930 datasheet.
- `0x77`: BME680-compatible address; retain chip-ID probing when this device
  is used.

The reusable read-only identification example is
`examples/hpm5e00evk/src/bin/i2c_probe_0x39.rs`.

### Shared-bus validation

2026-07-13 added `embassy-embedded-hal 0.6.0` and the
`i2c_shared_sensors` example. It follows Embassy's blocking shared-I2C pattern:

```text
I2C0 -> Mutex<CriticalSectionRawMutex, RefCell<I2c>>
     -> I2cDevice(BH1750, 0x23)
     -> I2cDevice(APDS-9930, 0x39)
     -> I2cDevice(BME680, 0x77)
```

The release+LTO image ran for 28 consecutive one-second cycles. Every cycle
read all three device proxies in order without NACK, timeout, or mutex errors.
Representative values were:

```text
BH1750:     63.33-64.16 lux
APDS-9930: clear 524-537, IR 135-139, proximity 34-60
BME680:     27.46-28.08 C, 60.30-61.32 %RH, 99373-99377 Pa
```

The BME680 gas resistance rose from about 2.8 kohm to 18.3 kohm during heater
warm-up. Treat early gas samples as warm-up data when adding application-level
air-quality processing.

## UART3 PMS7003 bring-up

2026-07-13 pinmux verification against both `hpm-data` and the HPM SDK found
the HPM5E31 UART3 group-B pins below:

```text
PC14 = UART3_RXD, ALT 2
PC15 = UART3_TXD, ALT 2
```

PC10 provides UART2 DE/RTS functions and has no UART RX function on HPM5E31.

2026-07-14 rewrote `pms7003_uart3` from the known-working local CH32 reference:

```text
/Users/mono/Elec/WCH/ch32-hal/examples/ch32x035/src/bin/uart_pms7003_async.rs
```

The implementation was compared with three local references: the CH32 async
DMA example, the known-working HPM5300 blocking UART sensor thread, and the
stream synchronization logic in `old-embedded-drivers::pmsx003`. The temporary
common-UART blocking error-path modification was reverted.

Release+LTO builds offline. One controlled download completed in 9.068205 s
and reached the application automatically. A 12-second RTT attachment showed
only:

```text
INFO PMS7003 UART3 RX: PC14, 9600 8N1
```

No 32-byte DMA read completed and no UART error was reported. The current
boundary is the absence of a complete PMS7003 frame at PC14.

The active-mode example now uses HPM UART `read_until_idle()` with a 5-second
timeout. The idle boundary recovers alignment after a partial first burst.
On target it repeatedly reported `no frame in 5s`; it observed no partial
burst, UART error, or valid frame.

Added a separate electrical diagnostic example:

```text
examples/hpm5e00evk/src/bin/pms7003_pc14_signal.rs
```

It samples PC14 as a GPIO input for two seconds without a pull and two seconds
with the internal pull-up. Every run sampled about 3.8 million low levels,
zero high levels, and zero edges in both configurations. PC14 is therefore
held low by the external sensor connection; this boundary precedes UART, DMA,
and frame parsing.

The PMS7003 V2.5 manual defines pins 1/2 as 5 V, 3/4 as ground, pin 5 as
active-low RESET, pin 9 as 3.3 V TTL TX, and pin 10 as SET where high/floating
means normal operation and low means sleep. Power-up defaults to active mode,
with output intervals from roughly 0.2 to 2.3 seconds. Validate those physical
conditions before changing UART code.

The target is a custom user board where PC14 has no other onboard connection.
The official `hpm5e00evk` PPI/SRAM pin assignments are valid only as SoC
pinmux reference data and must not be used to infer this board's netlist. The
earlier SRAM-conflict hypothesis is invalidated. Continue with the custom
schematic/netlist and a PC14 waveform measurement.

After the user corrected the wiring, PC14 measured about 97% high with
236--360 edges per two-second window. This confirms idle-high UART bursts now
reach the MCU.

With valid input present, the async DMA/IRQ version reproducibly enters the
default exception handler after its initialization log. The trap record is:

```text
mcause = 5
mepc   = 0x80004a36  (GPTMR Driver::now + 2)
mtval  = 0xf404003a
```

A PPOR cold reset reproduces the same record. The instruction at `mepc` does
not semantically access `mtval`. The later verified flash mismatch proves that
instructions may differ from the ELF, so the async fault remains unattributed.
Re-test it only after `download --verify` passes, then separate an UART
interrupt/context defect from corrupted instruction fetch.

The HPM5300-style blocking UART path, combined with the local PMSx003 prefix
synchronizer, then produced 33 consecutive checksum-valid frames:

```text
PM1.0 = 3--5 ug/m3
PM2.5 = 4--7 ug/m3
PM10  = 4--7 ug/m3
```

The working implementation is now the formal `pms7003_uart3.rs` example and
builds in release+LTO. Keep `pms7003_pc14_signal.rs` as a separate electrical
diagnostic. An equivalent temporary blocking reference image produced the 33
valid frames above and completed the sensor functional validation.

The subsequent download of the formally named image returned `Finished` in
9.081483 s but produced no RTT output. Its trap record contained an illegal
instruction at `0x80003b2e`: `mtval=0x888092d7`, while the ELF instruction is
`0x882282e7`. A mapped read at `0x80003b28` returned 16 bytes that did not
match the ELF. A final bounded `download --verify` stopped at its first
mismatch:

```text
address  = 0x800031f1
expected = 0x52
actual   = 0x15
```

The target is now in an unqualified post-verification-failure state. Stop
peripheral flashing and return to the P0 flash program/readback transaction.
The sensor result remains valid because it was independently established by
the earlier checksum-valid blocking run.

### 2026-07-14 verified PMS7003 rerun

After the physical board reset cleared the stuck SBA state, the repaired
probe-rs flow programmed `pms7003_uart3` with CMSIS-DAP JTAG sequence batch 1,
32-word SBA chunks, mandatory 4 KiB page-buffer readback/retry, and complete
Flash verification. Three stable RAM write corruptions were independently
confirmed and recovered before the ROM program call. This proves that batch 1
reduces sequence aggregation while page-buffer verification remains required.

The post-download PPOR reset completed with a clean shutdown after backporting
the current upstream RISC-V cleanup shape: clear DCSR while halted, resume the
hart, clear DMI transport errors, then set `dmcontrol.dmactive=0`.

A pair of read-only RTT attachments observed 100 checksum-valid frames and
zero invalid frames. Representative atmospheric values were:

```text
PM1.0 = 3--5 ug/m3
PM2.5 = 5--10 ug/m3
PM10  = 5--10 ug/m3
version = 147, error = 0
```

The formal blocking example now provides the full CF=1 values, atmospheric
values, six particle-count bins per 0.1 L, version, error code, readiness, and
valid/invalid counters every two seconds. Use `BIN=pms7003_uart3 ./monitor.sh
attach` for a read-only attachment that preserves the current Flash image.

### Historical: 2026-07-15 USB bulk / HPM monitor investigation

`monitor_usb_lite` enumerated as `1209:5e31` and completed full HPM monitor
CONNECT/GETINFO/GETVALUE cycles over vendor bulk. Sustained cycles were unstable
at this investigation stage; the 2026-07-16 closure below supersedes this state.

Important corrected interpretation: a dTD token such as `0x001e8000` means 30
bytes remain, IOC (bit 15) is set, and ACTIVE (bit 7) is clear. Earlier notes
that described this token as ACTIVE are invalid. The short 34-byte OUT packet
has completed even when `ENDPTCOMPLETE` is late or absent.

Observed failure chain:

- USB UI/SRI can re-enter hundreds of times for one logical transfer. Runs
  measured roughly 400--1000 USB IRQs before parser data or stack loads became
  corrupt.
- Faults include instruction/load faults where the saved memory word is valid
  through JTAG while the CPU observes an address with extra DLM-related bits,
  for example `0x800084d2 -> 0x803084d2`, and parser loads from `0x00404000`.
- The official SDK interrupt context saves `mepc`, `mstatus`, MXSTATUS when
  supported, and MCCTLBEGINADDR/MCCTLDATA. The runtime wrapper now explicitly
  saves these CSRs. This removed some early return corruption but did not make
  an interrupt storm safe.
- The qTD slot-rotation experiment was reverted. The official HPM SDK uses the
  first per-endpoint qTD slot for short transfers, so the Rust driver follows
  the same fixed-slot layout.
- A 32-slot payload-buffer experiment did not improve stability and was
  reverted.

The latest non-cacheable snapshot narrowed the corruption to descriptor word
2. The IN payload buffer contained the correct monitor frame at `0x01233090`,
while the qTD buffer pointer changed from the expected `0x0123304a` form to
`0x0000304a`; the host then received deterministic instruction bytes from the
truncated address. HPM5E configures this SRAM window as
`MEM_TYPE_MEM_NON_CACHE_BUF`, and the old Rust path initialized qTD buffer word
2 through separate page-pointer and current-offset read-modify-write cycles.
The C SDK assigns the complete 32-bit pointer once.

The pending hardware-validation build now constructs every qTD token, next
pointer, buffer pointer and expected-length word with one full volatile store,
then publishes the chain with `fence iorw, iorw`. It also waits for qTD,
`ENDPTPRIME`, and `ENDPTSTAT` retirement before slot reuse. The monitor bulk
task no longer masks machine interrupts while parsing GETVALUE and calculating
CRC, keeping USB IRQ service available during protocol work.

Current direction: treat ENDPTCOMPLETE as a wake hint, bind completion to the
current pending qTD generation, and bound UI handling without relying on a
posted SRI W1C immediately becoming visible. Add per-cause UI/SRI counters
before the next hardware iteration. Run 10-cycle smoke first, then 100 and
1000 cycles. Do not report USB stability until all three pass with zero trap,
panic, CRC, command-order, or timeout failures.

### 2026-07-15 integrated four-sensor monitor result

Added `monitor_sensors.rs`, combining the verified Embassy shared-I2C-device
path for BH1750/APDS-9930/BME680 with a cooperative nonblocking byte poll of
the verified PMS7003 UART3 blocking receiver. All values and health counters
are exposed as stable `MON_*` symbols through HPM Monitor V2 GETVALUE. Added
`tools/monitor_sensor_stress.py` for ELF symbol lookup, unit-aware reporting,
refresh-counter checks, and USB/exception validation.

The release+LTO image built and passed complete flash readback plus automatic
reset. Its first full monitor query returned:

```text
BH1750       21.66 lux
APDS-9930    clear=67 IR=22 proximity=60 raw
BME680       27.34 C, 48.80 %RH, 99201 Pa
PMS7003      5/7/9 ug/m3 atmospheric, 21 valid frames
USB IRQ      303
```

The second protocol cycle timed out on bulk OUT while sensor counters kept
advancing (`I2C 15 -> 79`, `PMS valid 21 -> 110`) with a clear exception
record. This isolates the remaining instability to USB transport.

Two hardening fixes followed:

- GETVALUE independently bounds `payload_len` and requires the exact
  `12 + count * 12` layout, returning `RESULT_PAYLOAD_ERR` for corrupted input.
- The runtime CORE_LOCAL wrapper now saves/restores `mepc` and `mstatus` in
  the unused tail of its 144-byte trap frame. Before this fix, USB pressure
  produced `mcause=1`, `mepc/mtval=0x802070ba` outside the ELF text range.

After both fixes, the target retained `exception=0`, `panic=0`, I2C ready mask
`0x7`, I2C cycle 41, and PMS valid frame 58 after the host observed a bad
monitor response header. The remaining P0 is USB response-buffer/completion
integrity. The 10-cycle USB stability gate still fails.

### 2026-07-16 USB bulk stability closure

The remaining failures had two independent platform causes and one endpoint
completion deadlock:

- `hpm-riscv-rt` enabled L1I without a following `fence.i`. Adding the HPM
  instruction-cache synchronization after enable stopped the repeated XPI PC
  high-bit jumps observed after flash programming.
- HPM5E shared HPM6E's 600 MHz/1200 mV defaults. The new HPM5E path establishes
  the SDK-qualified 480 MHz/1275 mV state through a 24 MHz transition and
  explicit PLL0/post-divider programming.
- The USB ISR disabled global `USBINTR.UE` whenever SETUPSTAT was pending. A new
  SETUP could arrive while the control future waited on the other EP0 direction;
  EP1 OUT then reached `ENDPTCOMPLETE=0x2` with no software completion wake.
  Keeping UE enabled preserves all endpoint completion delivery.

Hardware acceptance results for `monitor_usb_lite` release/LTO:

```text
flash download --verify: passed
PPOR boot:              passed
protocol stress:        1000/1000 in 15.093 s
second PPOR stress:     100/100 in 1.511 s
timeouts/CRC/header:    0
panic/exception:        0
```

The standalone USB monitor P0 is complete. The integrated four-sensor monitor
was rebuilt on this driver/runtime baseline, passed full flash verify and PPOR,
then completed 30/30 monitor cycles in 33.963 s. I2C cycle advanced 25 to 54,
PMS valid frames advanced 35 to 74, USB IRQ advanced 777 to 2691, and all four
sensor refresh checks passed. The integrated USB/sensor P0 is complete.

### Open P1: integrated monitor stops after long unattended runtime

On 2026-07-18, the `monitor_sensors` image reached approximately 8 h 46 min
of runtime and then presented a preserved stalled-state snapshot. A new host
CONNECT/GETVALUE session timed out before receiving any bytes. Read-only JTAG
inspection of the current ELF symbols recorded:

```text
I2C cycle:          26384
BH1750 samples:     26383
APDS-9930 samples:  26383
BME680 samples:     26381
PMS valid frames:   36859
PMS last update:    31568416 ms
USB IRQ:            2712
USB stage:          0x740 (UI interrupt handling)
USBSTS/USBINTR:     0x81 / 0x143
panic/exception:    0
```

A later JTAG counter read returned the same I2C cycle, confirming that the
sensor executor had also stopped advancing. The last sensor values remained
coherent in DLM. PPOR cold reset restored operation immediately; three USB
monitor cycles passed in 2.499 s and all four sensor refresh checks advanced.

The short 1000-cycle standalone USB test and 30-cycle integrated test remain
valid coverage for active traffic. This new result adds an unattended-runtime
qualification dimension and currently leaves the stopping mechanism
unclassified.

Actions:

- [ ] Reproduce with timestamped heartbeat, executor-loop and per-task progress
      counters sampled throughout an overnight run.
- [ ] Run separate overnight cases with continuous host polling and with an
      idle host, then compare the first stopped counter and USB register state.
- [ ] Capture endpoint prime/status/complete, pending qTD generation and timer
      interrupt state in the preserved stalled state.
- [ ] Add a long-duration acceptance test that performs a fresh monitor session
      after at least 12 hours and verifies continued sensor-counter progress.

### 2026-07-17 implementation-boundary cleanup complete

- [x] Removed HAL and runtime startup stage markers from normal execution.
- [x] Moved USB snapshots and qTD trace into the opt-in `usb-diagnostics`
  feature.
- [x] Made the early runtime trap recorder opt-in through
  `runtime-diagnostics`, with successful-startup clearing.
- [x] Moved HPM5E PLL defaults into `sysctl::Config` and made the safe PLL
  transition consume that configuration.
- [x] Kept CPU-only completion state in normal BSS and DMA-owned structures in
  `.noncacheable`.
- [x] Passed HPM5E production-feature compilation, HPM6E80 cross-family
  compilation, standalone USB 1000/1000 hardware stress, and integrated sensor
  30/30 hardware stress.

The remaining work is test automation and reviewable commit separation. The
runtime, SYSCTL, and USB fixes now live at their owning abstraction layers.

### Open: cross-family and feature-matrix build gaps found during cleanup

- [x] `ewdg` now uses `core::time::Duration` and accepts compatible duration
  types through `Into<Duration>`. The minimal `rt,hpm5301`, `rt,hpm5e31`,
  `rt,embassy,hpm5e31`, `rt,hpm6280`, `rt,hpm6880`, and `rt,hpm6e80` feature
  combinations compile without requiring the Embassy time driver.
- [x] `hpm6e50` ADC2/ADC3 analog-clock patches now use generated
  `peri_adc2`/`peri_adc3` capability cfgs. HPM6E50 and HPM6E80 both compile.
- [x] `hpm6e80` production-style build passes and provides the current
      cross-family regression evidence for the shared v6e/USB changes.

### Open P1: ADC16 common Embassy API parity (2026-07-19)

Embassy's maintained HALs converge on four practical ADC layers:

- blocking single-channel sampling;
- interrupt-driven async single-channel sampling;
- finite multi-channel sequence/batch sampling;
- timer-triggered continuous sampling into a ring or double buffer.

Representative implementations are MSPM0's blocking/async channel and
12-entry sequence API, RP's DMA single/multi-channel batch API, STM32's
configured sequences and ring-buffered ADC, and nRF's timer/task-triggered
double-buffer sampler. MCXA additionally models LPADC commands, chained
triggers, comparison, FIFO, and trigger priority.

Before this work, `hpm-hal::adc` provided initialization/calibration, blocking
one-shot, periodic register sampling, resolution/clock selection, per-channel
sample cycles, and type-erased channels. Its `State::waker` and instance
interrupt were unused. Sequence and preemption appeared only in the module
documentation. Configuration failures and
conversion conflicts panicked, and calibration/one-shot paths contained
unbounded polling.

HPM5E31 ADC16 supplies the foundation for the common layers: a 16-entry
sequence queue, software/TRGM triggers, continuous/restart modes, completion
interrupts, watchdog events, and an ADC-owned sequence DMA engine. The HPM5E31
DMA result is a packed 32-bit word containing value, channel, sequence number,
and cycle bit. DMA buffers require system-address translation, alignment, and
non-cacheable memory handling.

Implementation scope:

- [x] P0: introduce typed errors and bounded waits for invalid configuration,
      calibration, conflict, timeout, DMA abort, and FIFO full.
- [x] P0: add `Blocking`/`Async` construction and interrupt-driven async
      one-shot sampling with cancellation cleanup.
- [x] P0: add a safe finite `read_sequence` API using ADC16 internal sequence
      DMA and decoded sample access.
- [ ] P1: add timer/TRGM-triggered continuous sampling through an async
      ring/double-buffer reader with explicit overrun reporting.
- [x] Add a hardware example for async one-shot and two-channel sequence DMA.
- [ ] Add a hardware example for timer-triggered continuous capture.

HPM5E31 hardware evidence on 2026-07-19:

- `adc_async` release built for `riscv32imac-unknown-none-elf`; ADC-owned DMA
  storage linked at `.noncacheable.bss` `0x0123_0000`.
- Final download and full verify completed in 20.9 seconds, followed by successful
  PPOR.
- Async PF27/ADC0_IN0 one-shot returned `20287`.
- The two-entry sequence completed 29 observed iterations successfully;
  packed metadata consistently decoded as `(sequence=0, channel=0)` for PF27
  and `(sequence=1, channel=5)` for PF28.
- Observed PF27 values were `19898..20596`; floating PF28 values were
  `17567..18140`. PF28 data serves only as queue/DMA validation evidence.

Tooling finding: `examples/hpm5e00evk/monitor.sh` still defaults to the older
`riscv32imafc` debug path and a 120-second flash timeout. The maintained
handoff baseline uses `riscv32imac` release and a 180-second bounded download.
The stale script path caused one pre-programming file lookup failure and one
bounded download timeout during this validation. Align the script defaults
with the handoff command before using `monitor.sh flash` as the release
qualification path.

HPM-specific preemption queues, differential dual-ADC pairing, complex trigger
routing, and watchdog streaming stay in the low-level PAC/extension layer for
this phase. They can receive typed high-level APIs when a concrete application
establishes the required semantics.

### HPM5E31 remaining driver qualification matrix (2026-07-19)

Representative hardware evidence currently covers SYSCTL/clock startup, GPIO
output, GPTMR blocking timing, I2C0 async shared-bus transactions, UART3 async
RX, SPI1 async full-duplex through HDMA, USB0 device control/vendor bulk, and
ADC0 blocking/async/finite sequence operation. Flash download/verify and PPOR
belong to the probe/runtime qualification path and are also complete.

Public drivers that still need HPM5E31 hardware qualification:

- [ ] P0 EWDG0: validate start/feed/stop/status, then run a bounded intentional
      timeout-reset case and prove PPOR-style recovery. The HPM5E build issue is
      fixed locally; durable evidence currently stops at compilation.
- [ ] P0 MBX0A/MBX0B: run word and FIFO loopback through blocking and async
      APIs, covering both interrupts and repeated queue wrap through the
      internal mailbox connection.
- [ ] P0 MCAN0: run controller internal loopback through the `mcan` dependency,
      covering message RAM at `0xF020_0000`, nominal timing, TX/RX interrupts,
      and repeated frames. External-transceiver qualification can follow.
- [ ] P1 XDMA: rerun and record the existing `dma_memcpy` HDMA/XDMA comparison;
      add cancellation and linked/ring-buffer coverage after the basic result.
- [ ] P1 GPIO input/interrupt: self-wire one output to one input and validate
      level reads plus rising/falling async waits. GPIO output latch is already
      qualified.
- [ ] P1 GPTMR advanced paths: validate interrupt-driven delay, input capture,
      compare output, DMA request, and synchronization. Blocking delay timing
      is already qualified against the 24 MHz MCHTMR.
- [ ] P1 UART TX/full-duplex and SPI blocking/explicit-CS paths: current
      representative evidence covers UART3 RX and SPI1 async DMA full-duplex.
- [ ] P1 ADC1 instance and ADC timer/TRGM continuous ring or double-buffer
      capture. ADC0 finite sequence operation is qualified.

Drivers that need implementation work before HPM5E31 qualification:

- [x] P0 PWMV2 implementation and GPIO output-buffer fix were recovered by
      rebuilding the HPM5E work on current `hpmicro-rs/hpm-hal` master.
- [ ] Qualify PWM0 P6/Ch6 on PD06 and P7/Ch7 on PD07, then extend coverage to
      complementary output, dead time, and fault handling.
- [ ] P1 QEIV2: the current public module mainly configures pins/resources and
      exposes raw registers. Define counter/phase/index/filter semantics for
      `qei_v6e`, then drive it from GPIO/PWM/TRGM and verify direction/count.
- [ ] P1 TRGM: the current public API exposes the register block. Add typed
      source/destination routing and filtering primitives required by ADC
      continuous capture, GPTMR capture, PWM, and QEI validation.

HPM5E31 peripherals currently represented only by PAC access include ENET0,
ESC, ACMP0, TSNS, SDM0, PEWDG, CRC, and OTP. Treat each as a separate
driver-support project when it enters product scope. The HPM5E31 peripheral set
excludes DAC, RTC, RNG, and FEMC, so this board's validation matrix ends at the
listed devices.

### 2026-07-19 upstream-baseline migration review

- [x] Removed the abandoned `hpmicro/hpm-hal` remote. `hpm-hal` now has one
      remote: `origin = git@github.com:hpmicro-rs/hpm-hal.git`.
- [x] Created `hpm5e-support-v2` from current `origin/master` (`574b1e7`) and
      replayed the five HPM5E commits in order with semantic conflict review.
- [x] Kept upstream PWMV2, SPI, audio, generated build configuration, and
      dependency updates.
- [x] Kept the board-qualified HPM5E PLL0/DCDC path, PLIC direct completion
      writes, MCHTMR volatile 64-bit access, USB qTD/W1C state machine, monitor
      examples, and ADC async sequence API.
- [x] Split shared SYSCTL code into `v6e_hpm5e.rs` and `v6e_hpm6e.rs`. HPM5E
      owns its single-core 480 MHz path; HPM6E retains upstream preset/audio and
      dual-core resources.
- [x] Fixed two current-upstream regressions found by the family matrix: HPM68
      DAO/PDM clock integration and HPM6E50 ADC3 capability gating.
- [x] Cross-build representatives pass for HPM5301, HPM6750, HPM6880,
      HPM6E50, HPM6E80, and HPM5E31. All HPM5E board bins compile.
- [ ] Publish `hpm-data` commit `3fb4f3c` and `hpm-riscv-rt` commits `2600bc2`
      and `afdc417` to reachable branches/tags, then replace HAL path
      dependencies with immutable git revisions or released versions.
- [ ] Move the reviewed branch from the temporary worktree into the primary
      checkout after preserving the old branch's local documentation and
      bring-up files.
