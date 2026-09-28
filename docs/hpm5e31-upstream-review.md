# HPM5E31 upstream migration review

Updated: 2026-07-19

## Baseline

- Effective remote: `git@github.com:hpmicro-rs/hpm-hal.git`
- Base: `origin/master` at `574b1e7`
- Reviewed branch: `hpm5e-support-v2`
- Original branch: `hpm5e-support` at `a4eb03a`
- Common ancestor: `620f58d`
- Original divergence: 5 local commits and 58 upstream commits

The migration was performed in a separate worktree so the primary checkout's
local documentation and bring-up programs remained intact.

## Commit replay

1. `3e9702e feat: add HPM5E monitor tooling`
   - Merged target metadata, runtime integration, HPM5E examples and USB DCD.
   - Retained upstream PWMV2, SPI, audio drivers and build generation.
   - Removed default-path DMAMUX diagnostic logging.
   - Added family-safe ACMP resource patches.
2. `1bbbce7 fix: handle multi-qtd usb transfer lengths`
   - Applied cleanly and passed HPM5E cross-build.
3. `9e07d2b fix: harden HPM5E runtime and USB transfers`
   - Kept direct PLIC CLAIM/COMPLETE writes and volatile MCHTMR access.
   - Kept USB W1C ownership, full-word qTD publication, generation tracking,
     cancellation cleanup and opt-in diagnostics.
   - Split HPM5E and HPM6E SYSCTL implementations by family.
4. `2d190a0 feat: add qualified HPM5E sensor monitor`
   - Applied cleanly; retains async USB task and independently yielding sensor
     tasks for three I2C sensors plus PMS7003 UART.
5. `a4eb03a feat(adc): add async sequence sampling`
   - Applied cleanly. The sequence DMA setup and packed result decoding match
     the local HPM SDK ADC16 implementation.

Additional reviewed changes:

- Added DMA v1/v2 `Transfer::new_memcpy`, completed XDMA reset/IRQ setup and
  retained non-cacheable buffer requirements.
- Changed EWDG duration input to `core::time::Duration`, removing its hard
  dependency on Embassy time.
- Retained four qualification examples: ADC/ML8511, SPI async DMA loopback,
  GPTMR/GPIO timing, and HDMA/XDMA memcpy.
- Classified one-device probes, stage markers and wiring diagnostics as local
  bring-up assets.

## Overlap decisions

| Area | Selected implementation | Reason |
| --- | --- | --- |
| SYSCTL HPM5E | Local qualified family file | Matches SDK 960 MHz VCO, 480 MHz CPU and 1275 mV DCDC |
| SYSCTL HPM6E | Current upstream family file | Preserves dual-core resources, presets and audio clocks |
| PWMV2 | Current upstream | Complete modular implementation and GPIO output fix |
| SPI | Current upstream plus HPM5E cfg | Preserves current fixes and HPM5E register compatibility |
| DMA clocks | Current upstream ownership | SYSCTL opens clocks; DMA initializes controller and IRQ |
| PLIC completion | Local hardened write | CLAIM/COMPLETE has read side effects |
| MCHTMR | Local hardened access | Stable 64-bit read and safe compare write sequence |
| USB | Local qualified state machine | Passed 1000-cycle standalone and integrated stress tests |
| ADC | Local async extension | Matches SDK sequence DMA and passed HPM5E hardware test |

## Upstream defects discovered

- HPM68 DAO lacked a `ClockPeripheral` mapping to `resources::DAO0`.
- The DMA-v2 PDM path called the HPM6E I2S clock helper without a family cfg.
- HPM6E ADC2/ADC3 analog patches used a broad family cfg; HPM6E50 has no ADC3.
- DMAMUX emitted unconditional `defmt::info!` calls in a default HAL build.

These are fixed on the reviewed branch. They are independent, reviewable
cross-family fixes suitable for upstream submission.

## Validation matrix

The following commands pass for `riscv32imafc-unknown-none-elf`:

- `cargo check --features hpm5301`
- `cargo check --features hpm6750`
- `cargo check --features hpm6880`
- `cargo check --features hpm6e50`
- `cargo check --features hpm6e80`
- `cargo check --features hpm5e31`
- `cargo check --bins` in `examples/hpm5e00evk`

Warnings inherited from current upstream remain in ENET, PDM, SDXC and a few
dead-code paths. The reviewed changes introduce no build errors in this matrix.

## Dependency publication gate

- `hpm-data` requires local commit `3fb4f3c` for HPM5E family, SVD and pinmux
  generation. Its remote main branch still lacks HPM5E.
- `hpm-riscv-rt` requires `2600bc2` and `afdc417` for PMP/PMA, full trap
  context, cache synchronization, PLIC semantics and opt-in diagnostics.
- HAL currently uses local path dependencies. Publish these commits first,
  then pin HAL to immutable git revisions or released versions.

## Remaining acceptance work

- Publish and pin dependency commits.
- Move the reviewed branch into the primary checkout while preserving local
  documentation and probe programs.
- Run release builds, full flash verify, PPOR, USB 1000-cycle stress,
  integrated sensor stress and the long-duration idle/active comparison.
- Qualify PWMV2 output on PD06/PD07 and complete the remaining peripheral
  hardware matrix tracked in `hpm5e31-backlog.md`.
