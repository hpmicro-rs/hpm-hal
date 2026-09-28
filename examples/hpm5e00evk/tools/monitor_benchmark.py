#!/usr/bin/env python3
"""Benchmark HPM Monitor V2 GETVALUE throughput over USB bulk."""

from __future__ import annotations

import argparse
import json
import struct
import time
from pathlib import Path

from monitor_smoke import (
    CMD_CONNECT,
    CMD_GETINFO,
    CMD_GETVALUE,
    PROFILE_MIN_SIZE,
    BulkMonitorTransport,
    find_symbols,
    make_getvalue_payload,
    make_packet,
    parse_getvalue_payload,
    read_packet,
    require_success,
    write_transport_frame,
)
from monitor_web.server import DEFAULT_SYMBOLS, read_monitor


def command(transport, index: int, cmd: int, timeout: float, payload: bytes = b"") -> dict:
    write_transport_frame(transport, make_packet(index, cmd, payload))
    response = read_packet(transport, timeout)
    require_success(response, cmd)
    if int(response["index"]) != index:
        raise ValueError(f"unexpected packet index {response['index']}, expected {index}")
    return response


def connect(transport, timeout: float) -> tuple[dict[str, int], int]:
    command(transport, 1, CMD_CONNECT, timeout)
    response = command(transport, 2, CMD_GETINFO, timeout)
    vid, pid, version, maxsize, memsize, max_count, threshold = struct.unpack(
        "<HHQIIHH", bytes(response["payload"])
    )
    return (
        {
            "vid": vid,
            "pid": pid,
            "version": version,
            "maxsize": maxsize,
            "memsize": memsize,
            "max_count": max_count,
            "threshold": threshold,
        },
        3,
    )


def run_case(
    transport,
    names: list[str],
    symbols: dict[str, int],
    max_count: int,
    timeout: float,
    iterations: int,
    warmup: int,
    packet_index: int,
) -> tuple[dict[str, int | float | str], int]:
    chunks = [names[start : start + max_count] for start in range(0, len(names), max_count)]
    app_bytes_per_snapshot = sum(2 * (PROFILE_MIN_SIZE + 12 + 12 * len(chunk)) for chunk in chunks)
    total_runs = warmup + iterations
    started = 0.0

    for iteration in range(total_runs):
        if iteration == warmup:
            started = time.perf_counter()
        for chunk in chunks:
            payload = make_getvalue_payload(chunk, symbols)
            response = command(transport, packet_index, CMD_GETVALUE, timeout, payload)
            parse_getvalue_payload(chunk, bytes(response["payload"]))
            packet_index = 3 + ((packet_index - 2) % 0x7FFC)

    elapsed = time.perf_counter() - started
    transactions = iterations * len(chunks)
    values = iterations * len(names)
    app_bytes = iterations * app_bytes_per_snapshot
    return (
        {
            "name": "scalar" if len(names) == 1 else ("packet_max" if len(chunks) == 1 else "web_snapshot"),
            "symbols_per_snapshot": len(names),
            "transactions_per_snapshot": len(chunks),
            "iterations": iterations,
            "elapsed_s": round(elapsed, 6),
            "snapshots_per_s": round(iterations / elapsed, 3),
            "transactions_per_s": round(transactions / elapsed, 3),
            "values_per_s": round(values / elapsed, 3),
            "application_bytes_per_snapshot": app_bytes_per_snapshot,
            "application_bytes_per_s": round(app_bytes / elapsed, 3),
        },
        packet_index,
    )


def run_web_bridge_case(args, symbol_count: int) -> dict[str, int | float | str]:
    warmup = min(args.warmup, 3)
    started = 0.0
    last_duration_ms = 0.0
    for iteration in range(warmup + args.cold_iterations):
        if iteration == warmup:
            started = time.perf_counter()
        result = read_monitor("bulk", None, args.elf, DEFAULT_SYMBOLS, args.timeout, args.vid, args.pid)
        last_duration_ms = float(result["duration_ms"])

    elapsed = time.perf_counter() - started
    # CONNECT (68 B duplex), GETINFO (92 B duplex), then GETVALUE chunks.
    getvalue_bytes = sum(
        2 * (PROFILE_MIN_SIZE + 12 + 12 * min(32, symbol_count - start))
        for start in range(0, symbol_count, 32)
    )
    app_bytes_per_snapshot = 68 + 92 + getvalue_bytes
    return {
        "name": "web_bridge_cold_session",
        "symbols_per_snapshot": symbol_count,
        "transactions_per_snapshot": 4,
        "iterations": args.cold_iterations,
        "elapsed_s": round(elapsed, 6),
        "snapshots_per_s": round(args.cold_iterations / elapsed, 3),
        "transactions_per_s": round(args.cold_iterations * 4 / elapsed, 3),
        "values_per_s": round(args.cold_iterations * symbol_count / elapsed, 3),
        "application_bytes_per_snapshot": app_bytes_per_snapshot,
        "application_bytes_per_s": round(args.cold_iterations * app_bytes_per_snapshot / elapsed, 3),
        "last_handler_duration_ms": last_duration_ms,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--elf", type=Path, required=True)
    parser.add_argument("--vid", type=lambda value: int(value, 0), default=0x1209)
    parser.add_argument("--pid", type=lambda value: int(value, 0), default=0x5E31)
    parser.add_argument("--timeout", type=float, default=2.0)
    parser.add_argument("--iterations", type=int, default=1000)
    parser.add_argument("--warmup", type=int, default=50)
    parser.add_argument("--cold-iterations", type=int, default=50)
    parser.add_argument("--json", type=Path)
    args = parser.parse_args()

    if args.iterations < 1 or args.cold_iterations < 1 or args.warmup < 0:
        parser.error("iteration counts must be positive and warmup must be non-negative")

    symbols = find_symbols(args.elf, DEFAULT_SYMBOLS)
    names = [name for name in DEFAULT_SYMBOLS if name in symbols]
    if not names:
        parser.error("no web monitor symbols found in ELF")

    with BulkMonitorTransport(args.vid, args.pid, args.timeout) as transport:
        info, packet_index = connect(transport, args.timeout)
        if info["max_count"] < 1:
            raise RuntimeError("device reported max_count=0")
        cases = [names[:1], names[: info["max_count"]], names]
        results = []
        for case_names in cases:
            result, packet_index = run_case(
                transport,
                case_names,
                symbols,
                info["max_count"],
                args.timeout,
                args.iterations,
                args.warmup,
                packet_index,
            )
            results.append(result)
            print(
                f"{result['name']}: {result['snapshots_per_s']:.3f} snapshots/s, "
                f"{result['transactions_per_s']:.3f} transactions/s, "
                f"{result['values_per_s']:.3f} values/s, "
                f"{result['application_bytes_per_s'] / 1000:.3f} kB/s",
                flush=True,
            )

    cold_result = run_web_bridge_case(args, len(names))
    results.append(cold_result)
    print(
        f"{cold_result['name']}: {cold_result['snapshots_per_s']:.3f} snapshots/s, "
        f"{cold_result['values_per_s']:.3f} values/s, "
        f"{cold_result['application_bytes_per_s'] / 1000:.3f} kB/s",
        flush=True,
    )

    report = {
        "elf": str(args.elf.resolve()),
        "info": info,
        "available_web_symbols": len(names),
        "warmup": args.warmup,
        "cold_iterations": args.cold_iterations,
        "results": results,
    }
    if args.json:
        args.json.parent.mkdir(parents=True, exist_ok=True)
        args.json.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
