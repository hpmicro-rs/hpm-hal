#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"

PROBE_RS="${PROBE_RS:-/tmp/probe-rs-hpm-debug-src/target/release/probe-rs}"
CHIP_DESC="${CHIP_DESC:-/Users/mono/Elec/HPM/hpm-hal/HPMicro.yaml}"
PROBE="${PROBE:-0d28:0204:214F44B4FEDADD10BB0B8044E000E80A}"
SPEED="${SPEED:-10}"
BIN="${BIN:-blinky}"
ELF="${ELF:-target/riscv32imafc-unknown-none-elf/debug/$BIN}"
FLASH_TOOL="${FLASH_TOOL:-openocd}"
OPENOCD="${OPENOCD:-/Users/mono/opt/riscv-openocd-hpm/bin/openocd}"
HPM_SDK_BASE="${HPM_SDK_BASE:-/Users/mono/Elec/HPM/hpm_sdk}"
OPENOCD_SPEED="${OPENOCD_SPEED:-100}"
PROBE_RS_LOG="${PROBE_RS_LOG:-/tmp/probe-rs-hpm5e-download.log}"
PROBE_RS_TIMEOUT="${PROBE_RS_TIMEOUT:-8s}"
RTT_TIMEOUT="${RTT_TIMEOUT:-90s}"

cmd="${1:-flash}"

usage() {
    cat <<'EOF'
Usage:
  ./monitor.sh build
  ./monitor.sh flash
  ./monitor.sh rtt
  ./monitor.sh probe-debug
  ./monitor.sh smoke [serial-port]
  ./monitor.sh web
  ./monitor.sh all [serial-port]

Defaults:
  BIN: blinky
  FLASH_TOOL: openocd
  flash: builds are expected at target/riscv32imafc-unknown-none-elf/debug/$BIN
  smoke: scans /dev/cu.usbmodem* for the target USB CDC monitor
  web: starts the monitor web service on http://0.0.0.0:8765

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

flash() {
    echo "Flashing $ELF"
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
            "$PROBE_RS" download \
                --chip HPM5E31 \
                --chip-description-path "$CHIP_DESC" \
                --protocol jtag \
                --probe "$PROBE" \
                --speed "$SPEED" \
                --connect-under-reset \
                --disable-progressbars \
                --disable-double-buffering \
                "$ELF"
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
    python3 tools/monitor_smoke.py --port "$port" --timeout 2
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

probe_debug() {
    local timeout_cmd
    timeout_cmd="$(command -v timeout || command -v gtimeout || true)"

    rm -f "$PROBE_RS_LOG"
    echo "Writing probe-rs log to $PROBE_RS_LOG"

    if [[ "$timeout_cmd" == "" ]]; then
        echo "timeout/gtimeout is required for probe-debug"
        return 2
    fi

    "$timeout_cmd" "$PROBE_RS_TIMEOUT" "$PROBE_RS" download \
        --chip HPM5E31 \
        --chip-description-path "$CHIP_DESC" \
        --protocol jtag \
        --probe "$PROBE" \
        --speed 100 \
        --disable-progressbars \
        --disable-double-buffering \
        --log-file "$PROBE_RS_LOG" \
        "$ELF" || true

    rg 'Using sequence|Setting scan chain|Configuring JTAG|Found IDCODE|Detected IR|Found .*TAP|JtagChainItem|Dtmcs|DTM|DMI|DM register|Timeout|RequestInProgress|Reserved|flash|download|ERROR|WARN|write_register_batch' "$PROBE_RS_LOG" | tail -n 160 || true
}

probe_rtt() {
    local timeout_cmd
    timeout_cmd="$(command -v timeout || command -v gtimeout || true)"

    if [[ "$timeout_cmd" == "" ]]; then
        echo "timeout/gtimeout is required for rtt"
        return 2
    fi

    build
    echo "Running $ELF with probe-rs RTT/defmt for $RTT_TIMEOUT"
    "$timeout_cmd" "$RTT_TIMEOUT" "$PROBE_RS" run \
        --chip HPM5E31 \
        --chip-description-path "$CHIP_DESC" \
        --protocol jtag \
        --probe "$PROBE" \
        --speed 100 \
        --disable-progressbars \
        --disable-double-buffering \
        "$ELF"
}

web() {
    python3 tools/monitor_web/server.py --host "${WEB_HOST:-0.0.0.0}" --port "${WEB_PORT:-8765}"
}

case "$cmd" in
    build)
        build
        ;;
    flash)
        flash
        ;;
    smoke)
        smoke "${2:-}"
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
