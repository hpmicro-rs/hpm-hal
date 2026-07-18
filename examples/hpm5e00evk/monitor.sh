#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"

PROBE_RS="${PROBE_RS:-/Users/mono/Elec/probe-rs/target/debug/probe-rs}"
CHIP_DESC="${CHIP_DESC:-/Users/mono/Elec/HPM/hpm-hal/HPMicro.yaml}"
PROBE="${PROBE:-0d28:0204:214F44B4FEDADD10BB0B8044E000E80A}"
SPEED="${SPEED:-500}"
BIN="${BIN:-blinky}"
ELF="${ELF:-target/riscv32imafc-unknown-none-elf/debug/$BIN}"
FLASH_TOOL="${FLASH_TOOL:-probe-rs}"
OPENOCD="${OPENOCD:-/Users/mono/opt/riscv-openocd-hpm/bin/openocd}"
HPM_SDK_BASE="${HPM_SDK_BASE:-/Users/mono/Elec/HPM/hpm_sdk}"
OPENOCD_SPEED="${OPENOCD_SPEED:-100}"
PROBE_RS_LOG="${PROBE_RS_LOG:-/tmp/probe-rs-hpm5e-download.log}"
PROBE_RS_TIMEOUT="${PROBE_RS_TIMEOUT:-8s}"
FLASH_TIMEOUT="${FLASH_TIMEOUT:-120s}"
RTT_TIMEOUT="${RTT_TIMEOUT:-180s}"
RISCV_SYSBUS_WRITE_CHUNK_WORDS="${RISCV_SYSBUS_WRITE_CHUNK_WORDS:-32}"
CMSIS_DAP_JTAG_SEQUENCE_BATCH="${CMSIS_DAP_JTAG_SEQUENCE_BATCH:-1}"
PY_RUN="${PY_RUN:-uv run --with pyserial --with pyusb python}"

cmd="${1:-flash}"

usage() {
    cat <<'EOF'
Usage:
  ./monitor.sh build
  ./monitor.sh build-baseline
  ./monitor.sh flash
  ./monitor.sh reset
  ./monitor.sh attach
  ./monitor.sh rtt
  ./monitor.sh probe-debug
  ./monitor.sh smoke [serial-port]
  ./monitor.sh smoke-bulk
  ./monitor.sh web
  ./monitor.sh all [serial-port]

Defaults:
  BIN: blinky
  FLASH_TOOL: probe-rs
  flash: builds are expected at target/riscv32imafc-unknown-none-elf/debug/$BIN
  smoke: scans /dev/cu.usbmodem* for the target USB CDC monitor
  smoke-bulk: probes 1209:5e31 via vendor bulk endpoints
  web: starts the monitor web service on http://0.0.0.0:8765
  reset: resets the target with probe-rs after flashing or debugging

Examples:
  ./monitor.sh all
  BIN=monitor_usb_lite ./monitor.sh all
  FLASH_TOOL=probe-rs ./monitor.sh flash
  BIN=rtt_defmt_probe ./monitor.sh rtt

BOOT0 recovery:
  Hold BOOT0, run ./monitor.sh flash, release BOOT0 after output starts.
EOF
}

build() {
    cargo build --bin "$BIN"
}

build_baseline() {
    local bins=(
        main_marker
        rtt_defmt_probe
        i2c_detect
        i2c_shared_sensors
        i2c_sensor_bh1750
        i2c_sensor_bme680
        monitor_usb_lite
        monitor_blinky
    )

    for bin in "${bins[@]}"; do
        echo "Building baseline bin: $bin"
        cargo build --bin "$bin"
    done
}

timeout_cmd() {
    command -v timeout || command -v gtimeout || true
}

reset_target() {
    local timeout_bin
    timeout_bin="$(timeout_cmd)"
    local reset_cmd=(
        "$PROBE_RS" reset
        --chip HPM5E31
        --chip-description-path "$CHIP_DESC"
        --protocol jtag
        --probe "$PROBE"
        --speed "$SPEED"
    )

    if [[ "$timeout_bin" == "" ]]; then
        "${reset_cmd[@]}"
    else
        "$timeout_bin" "$PROBE_RS_TIMEOUT" "${reset_cmd[@]}"
    fi
}

flash() {
    echo "Flashing $ELF"
    export PROBE_RS_RISCV_SYSBUS_WRITE_CHUNK_WORDS="$RISCV_SYSBUS_WRITE_CHUNK_WORDS"
    export PROBE_RS_CMSIS_DAP_JTAG_SEQUENCE_BATCH="$CMSIS_DAP_JTAG_SEQUENCE_BATCH"
    case "$FLASH_TOOL" in
        openocd)
            "$OPENOCD" \
                -s "$HPM_SDK_BASE/boards/openocd" \
                -c "set HPM_SDK_BASE $HPM_SDK_BASE; set BOARD hpm5e00evk; set PROBE cmsis_dap" \
                -f hpm5e00_all_in_one.cfg \
                -c "cmsis-dap backend usb_bulk" \
                -c "adapter speed $OPENOCD_SPEED" \
                -c "init; reset halt; program $ELF reset exit"
            ;;
        probe-rs)
            echo "Probe: $PROBE, speed: ${SPEED}kHz"
            echo "BOOT0 recovery: hold BOOT0 before this step, release it after probe-rs starts printing."
            timeout_bin="$(timeout_cmd)"
            if [[ "$timeout_bin" == "" ]]; then
                echo "timeout/gtimeout is required for bounded probe-rs flash"
                return 2
            fi
            if ! "$timeout_bin" "$FLASH_TIMEOUT" "$PROBE_RS" download \
                --chip HPM5E31 \
                --chip-description-path "$CHIP_DESC" \
                --protocol jtag \
                --probe "$PROBE" \
                --speed "$SPEED" \
                --connect-under-reset \
                --disable-progressbars \
                --disable-double-buffering \
                --verify \
                "$ELF"; then
                echo "probe-rs download or verification failed; target state preserved for diagnosis"
                return 1
            fi
            echo "Resetting target after download"
            reset_target
            ;;
        *)
            echo "Unknown FLASH_TOOL: $FLASH_TOOL"
            exit 2
            ;;
    esac
}

smoke_one() {
    local port="$1"
    echo "Trying $port"
    $PY_RUN tools/monitor_smoke.py --port "$port" --elf "$ELF" --timeout 2
}

smoke() {
    if [[ "${1:-}" != "" ]]; then
        smoke_one "$1"
        return
    fi

    local ok=1
    local ports=(/dev/cu.usbmodem*)
    if [[ ! -e "${ports[0]}" ]]; then
        echo "No /dev/cu.usbmodem* ports found"
        return 1
    fi

    for port in "${ports[@]}"; do
        if [[ -e "$port" ]] && smoke_one "$port"; then
            ok=0
            break
        fi
    done
    return "$ok"
}

smoke_bulk() {
    $PY_RUN tools/monitor_smoke.py --transport bulk --elf "$ELF" --timeout 2
}

probe_debug() {
    local timeout_bin
    timeout_bin="$(timeout_cmd)"

    rm -f "$PROBE_RS_LOG"
    echo "Writing probe-rs log to $PROBE_RS_LOG"

    if [[ "$timeout_bin" == "" ]]; then
        echo "timeout/gtimeout is required for probe-debug"
        return 2
    fi

    "$timeout_bin" "$PROBE_RS_TIMEOUT" "$PROBE_RS" download \
        --chip HPM5E31 \
        --chip-description-path "$CHIP_DESC" \
        --protocol jtag \
        --probe "$PROBE" \
        --speed "$SPEED" \
        --disable-progressbars \
        --disable-double-buffering \
        --log-file "$PROBE_RS_LOG" \
        "$ELF" || true

    rg 'Using sequence|Setting scan chain|Configuring JTAG|Found IDCODE|Detected IR|Found .*TAP|JtagChainItem|Dtmcs|DTM|DMI|DM register|Timeout|RequestInProgress|Reserved|flash|download|ERROR|WARN|write_register_batch' "$PROBE_RS_LOG" | tail -n 160 || true
}

probe_rtt() {
    local timeout_bin
    timeout_bin="$(timeout_cmd)"

    if [[ "$timeout_bin" == "" ]]; then
        echo "timeout/gtimeout is required for rtt"
        return 2
    fi

    build
    echo "Running $ELF with probe-rs RTT/defmt for $RTT_TIMEOUT"
    "$timeout_bin" "$RTT_TIMEOUT" "$PROBE_RS" run \
        --chip HPM5E31 \
        --chip-description-path "$CHIP_DESC" \
        --protocol jtag \
        --probe "$PROBE" \
        --speed "$SPEED" \
        --disable-progressbars \
        --disable-double-buffering \
        --no-location \
        "$ELF"
}

attach_rtt() {
    local timeout_bin
    timeout_bin="$(timeout_cmd)"

    if [[ "$timeout_bin" == "" ]]; then
        echo "timeout/gtimeout is required for attach"
        return 2
    fi

    echo "Attaching to $ELF RTT/defmt for $RTT_TIMEOUT without flashing"
    "$timeout_bin" -s INT "$RTT_TIMEOUT" "$PROBE_RS" attach \
        --chip HPM5E31 \
        --chip-description-path "$CHIP_DESC" \
        --protocol jtag \
        --probe "$PROBE" \
        --speed "$SPEED" \
        --no-location \
        "$ELF" || [[ $? -eq 124 ]]
}

web() {
    $PY_RUN tools/monitor_web/server.py --host "${WEB_HOST:-0.0.0.0}" --port "${WEB_PORT:-8765}"
}

case "$cmd" in
    build)
        build
        ;;
    build-baseline)
        build_baseline
        ;;
    flash)
        flash
        ;;
    reset)
        reset_target
        ;;
    smoke)
        smoke "${2:-}"
        ;;
    smoke-bulk)
        smoke_bulk
        ;;
    web)
        web
        ;;
    probe-debug)
        probe_debug
        ;;
    rtt)
        probe_rtt
        ;;
    attach)
        attach_rtt
        ;;
    all)
        build
        flash
        smoke "${2:-}"
        ;;
    help|-h|--help)
        usage
        ;;
    *)
        usage
        exit 2
        ;;
esac
