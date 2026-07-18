#!/usr/bin/env python3
"""Read all sensor values through HPM Monitor V2 and verify periodic refresh."""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

from monitor_smoke import BulkMonitorTransport, find_symbols, run_monitor_query


SENSOR_NAMES = [
    "MON_SENSOR_CYCLE_COUNT",
    "MON_SENSOR_READY_MASK",
    "MON_SENSOR_LAST_ERROR",
    "MON_BH1750_SAMPLE_COUNT",
    "MON_BH1750_RAW",
    "MON_BH1750_LUX_X100",
    "MON_APDS9930_SAMPLE_COUNT",
    "MON_APDS9930_ID",
    "MON_APDS9930_STATUS",
    "MON_APDS9930_CLEAR_RAW",
    "MON_APDS9930_IR_RAW",
    "MON_APDS9930_PROXIMITY_RAW",
    "MON_BME680_SAMPLE_COUNT",
    "MON_BME680_TEMP_C_X100",
    "MON_BME680_HUMIDITY_RH_X100",
    "MON_BME680_PRESSURE_PA",
    "MON_BME680_GAS_OHM",
    "MON_BME680_GAS_VALID",
    "MON_PMS7003_STATUS",
    "MON_PMS7003_VALID_FRAMES",
    "MON_PMS7003_INVALID_FRAMES",
    "MON_PMS7003_UART_ERRORS",
    "MON_PMS7003_PM1_0_UG_M3",
    "MON_PMS7003_PM2_5_UG_M3",
    "MON_PMS7003_PM10_UG_M3",
    "MON_PMS7003_VERSION",
    "MON_PMS7003_SENSOR_ERROR",
    "MON_PMS7003_LAST_UPDATE_MS",
    "MON_USB_IRQ_COUNT",
    "EXCEPTION_MCAUSE",
    "EXCEPTION_MEPC",
]


def format_row(iteration: int, values: dict[str, int]) -> str:
    return (
        f"sample {iteration}: i2c_cycle={values['MON_SENSOR_CYCLE_COUNT']} "
        f"bh1750={values['MON_BH1750_LUX_X100'] / 100:.2f}lux "
        f"apds(clear/ir/prox)={values['MON_APDS9930_CLEAR_RAW']}/"
        f"{values['MON_APDS9930_IR_RAW']}/{values['MON_APDS9930_PROXIMITY_RAW']} "
        f"bme680={values['MON_BME680_TEMP_C_X100'] / 100:.2f}C,"
        f"{values['MON_BME680_HUMIDITY_RH_X100'] / 100:.2f}%RH,"
        f"{values['MON_BME680_PRESSURE_PA']}Pa "
        f"pms={values['MON_PMS7003_PM1_0_UG_M3']}/"
        f"{values['MON_PMS7003_PM2_5_UG_M3']}/"
        f"{values['MON_PMS7003_PM10_UG_M3']}ug/m3 "
        f"frames={values['MON_PMS7003_VALID_FRAMES']} "
        f"usb_irq={values['MON_USB_IRQ_COUNT']}"
    )


def validate(first: dict[str, int], last: dict[str, int]) -> list[str]:
    failures: list[str] = []
    if last["MON_SENSOR_READY_MASK"] & 0x7 != 0x7:
        failures.append(f"I2C ready mask is 0x{last['MON_SENSOR_READY_MASK']:x}, expected bits 0..2")
    if last["MON_SENSOR_LAST_ERROR"] != 0:
        failures.append(f"I2C sensor error 0x{last['MON_SENSOR_LAST_ERROR']:08x}")
    if last["MON_APDS9930_ID"] != 0x39:
        failures.append(f"APDS-9930 ID is 0x{last['MON_APDS9930_ID']:02x}")
    for counter in (
        "MON_BH1750_SAMPLE_COUNT",
        "MON_APDS9930_SAMPLE_COUNT",
        "MON_BME680_SAMPLE_COUNT",
        "MON_PMS7003_VALID_FRAMES",
    ):
        if last[counter] <= first[counter]:
            failures.append(f"{counter} did not advance: {first[counter]} -> {last[counter]}")
    if last["MON_PMS7003_STATUS"] != 2:
        failures.append(f"PMS7003 status is 0x{last['MON_PMS7003_STATUS']:08x}")
    if last["MON_PMS7003_UART_ERRORS"] != first["MON_PMS7003_UART_ERRORS"]:
        failures.append(
            "PMS7003 UART errors advanced: "
            f"{first['MON_PMS7003_UART_ERRORS']} -> {last['MON_PMS7003_UART_ERRORS']}"
        )
    if last["MON_PMS7003_INVALID_FRAMES"] != first["MON_PMS7003_INVALID_FRAMES"]:
        failures.append(
            "PMS7003 invalid frames advanced: "
            f"{first['MON_PMS7003_INVALID_FRAMES']} -> {last['MON_PMS7003_INVALID_FRAMES']}"
        )
    if last["EXCEPTION_MCAUSE"] != 0:
        failures.append(
            f"exception record mcause=0x{last['EXCEPTION_MCAUSE']:08x} "
            f"mepc=0x{last['EXCEPTION_MEPC']:08x}"
        )
    return failures


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--elf", type=Path, required=True)
    parser.add_argument("--vid", type=lambda value: int(value, 0), default=0x1209)
    parser.add_argument("--pid", type=lambda value: int(value, 0), default=0x5E31)
    parser.add_argument("--timeout", type=float, default=3.0)
    parser.add_argument("--iterations", type=int, default=10)
    parser.add_argument("--interval", type=float, default=1.0)
    parser.add_argument("--warmup", type=float, default=3.0)
    parser.add_argument("--chunk-values", type=int, default=1)
    args = parser.parse_args()

    if args.iterations < 2:
        raise SystemExit("--iterations must be at least 2")
    if args.chunk_values < 1:
        raise SystemExit("--chunk-values must be at least 1")
    symbols = find_symbols(args.elf, SENSOR_NAMES)
    missing = [name for name in SENSOR_NAMES if name not in symbols]
    if missing:
        raise SystemExit(f"missing monitor symbols: {', '.join(missing)}")

    if args.warmup > 0:
        time.sleep(args.warmup)

    snapshots: list[dict[str, int]] = []
    started = time.monotonic()
    try:
        with BulkMonitorTransport(args.vid, args.pid, args.timeout) as transport:
            for iteration in range(1, args.iterations + 1):
                _info, values = run_monitor_query(
                    transport,
                    SENSOR_NAMES,
                    symbols,
                    args.timeout,
                    chunk_limit=args.chunk_values,
                )
                snapshots.append(values)
                print(format_row(iteration, values), flush=True)
                if iteration < args.iterations and args.interval > 0:
                    time.sleep(args.interval)
    except Exception as error:
        print(
            f"FAIL: monitor transport stopped after {len(snapshots)} complete cycle(s): {error}",
            file=sys.stderr,
        )
        return 1

    failures = validate(snapshots[0], snapshots[-1])
    elapsed = time.monotonic() - started
    if failures:
        for failure in failures:
            print(f"FAIL: {failure}", file=sys.stderr)
        return 1
    print(f"PASS: {args.iterations} monitor cycles in {elapsed:.3f}s; all four sensors refreshed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
