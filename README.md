# hpm-hal

A Rust HAL implementation for the HPMicro's RISC-V MCUs.
The PAC(Peripheral Access Crate) is based on [hpm-data].

This crate is a working-in-progress and not ready for use.

## Project status

### MCU Family Support

| MCU Family | PAC | Demo | Embassy | SYSCTL | GPIO | UART | I2C | SPI | DMA | TMR | ADC | USB |
|------------|-----|------|---------|--------|------|------|-----|-----|-----|-----|-----|-----|
| HPM6700    | ✓   | ✓    | ✓       | ✓      | ✓+   | ✓+   | ✓   | ✓   | ✓+  |     |     |     |
| HPM6300    | ✓   | ✓    | ✓       | ✓      |      |      |     |     |     |     |     |     |
| HPM6200    | ✓   |      |         |        |      |      |     |     |     |     |     |     |
| HPM5300    | ✓   | ✓    | ✓       | ✓      | ✓+   | ✓+   | ✓   | ✓   | ✓+  |     |     |     |
| HPM6800    | ✓   |      |         |        |      |      |     |     |     |     |     |     |
| HPM6E00    | ✓   | ✓    | ✓       | ✓      | ✓+   | ✓+   | ?   | ✓   | ✓+  |     |     |     |

- ✓: Implemented
- ?: Requires demo verification
- !: Partially implemented
- Blank: Not implemented
- +: Async support

### TODO

- Peripherals:
  - [x] Basic rt code: linker, startup
    - [x] vectored interrupt handling
    - [x] L1C support
    - [ ] PMP for noncacheable memory
    - [ ] CPU1 support - how to?
  - [x] Embassy time driver using MCHTMR
  - [x] SYSCTL init
    - [x] Resource group handling
  - [x] PLL setting
  - [x] GPIO, Flex, Input, Output, Async
  - [x] DMA, both HDMA and XDMA
    - [x] DMA v2
    - [x] DMA v1
  - [x] UART
    - [x] Blocking driver
    - [x] Async driver
    - [ ] Ring buffer based async
  - [x] I2C
    - [x] Blocking driver
    - [ ] Async driver
  - [x] MBX
    - [x] blocking and async, message mode and fifo mode
    - [ ] DMA driver?
  - [x] FEMC
    - [x] SDRAM init
  - [x] SPI driver
    - [x] QSPI driver
    - [x] Blocking
    - [x] Async using DMA
  - [x] RTC, with alarm driver and optional chrono datetime
- Long term Plans
  - [ ] andes-riscv for specific CSRs
  - [ ] hpm-riscv-rt for customized runtime (riscv-rt is not fit)

### Toolchain Support

- [probe-rs]
  - [x] [HPM5300 series flash algorithm support](https://github.com/probe-rs/probe-rs/pull/2575)
  - [ ] [#2578 JTag support for DAPLink](https://github.com/probe-rs/probe-rs/pull/2578)
- [ ] probe-rs for HPM6750 is not working, use OpenOCD instead

## Usage

The best reference is the examples in the `examples` directory and Github actions workflow.

### Requirements

- A probe(debugger), optional if you are using official HPMicro's development board
  - FT2232-based (official HPMicro's development board uses this chip)
  - JLink
  - DAPLink-based probe
- A flash tool for your probe, choose one from:
  - [probe-rs]
  - [HPM OpenOCD]
  - JLink
  - HPMIcro Manufacturing Tool
- A RISC-V GCC toolchain if you perfer to use OpenOCD(only GDB is needed)
- A Rust toolchain
  - `rustup default nightly-2024-06-12` (locked because of bug [rust-embedded/riscv#196](https://github.com/rust-embedded/riscv/issues/196))
  - `rustup target add riscv32imafc-unknown-none-elf`

### Guide

#### Step 0. Prerequisites

- Install Rust: <https://rustup.rs/>
- Download HPM SDK: <https://github.com/hpmicro/hpm_sdk>
  - Set `HPM_SDK_BASE` environment variable to the SDK path
- Choose one debugger:
  - OpenOCD: HPM's fork <https://github.com/hpmicro/riscv-openocd>
  - probe-rs: <https://github.com/probe-rs/probe-rs>
    - The `HPMicro.yaml` flash algorithm is provided in top level of this repo
    - If you are using DAPLink probe, you need to use the version from PR [#2578 JTag support for DAPLink](https://github.com/probe-rs/probe-rs/pull/2578)

#### Step 1. Prepare Rust Toolchain

```bash
rustup default nightly-2024-06-12
rustup target add riscv32imafc-unknown-none-elf
```

#### Step 2. Clone this repo

```bash
git clone https://github.com/hpmicro-rs/hpm-hal.git

# Or if you are using SSH

git clone git@github.com:hpmicro-rs/hpm-hal.git

# Or if you are using GitHub CLI

gh repo clone hpmicro-rs/hpm-hal
```

#### Step 3. Run Examples

1. Edit `examples/YOUR_BOARD/.cargo/config.toml` to set the correct flash/run command for your probe.

2. (Optional) Edit and run `run-openocd.sh` if using OpenOCD.

3. Connect your probe to the target board.

4. Run an example:

```bash
cd examples/hpm5300evk
cargo run --release --bin blinky
```

## License

Embassy is licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or
  <http://www.apache.org/licenses/LICENSE-2.0>)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or <http://opensource.org/licenses/MIT>)

at your option.

## Contributing

This crate is under active development. Before starting your work, it's better to create a "Work in Progress" (WIP) pull request describing your work to avoid conflicts.

[hpm-data]: https://github.com/andelf/hpm-data
[HPM OpenOCD]: https://github.com/hpmicro/riscv-openocd
