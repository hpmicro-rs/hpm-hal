#!/usr/bin/env python3
"""Local web bridge for the HPM monitor protocol."""

from __future__ import annotations

import argparse
import json
import mimetypes
import sys
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

ROOT = Path(__file__).resolve().parents[2]
WEB_ROOT = Path(__file__).resolve().parent / "static"
DEFAULT_ELF = ROOT / "target/riscv32imafc-unknown-none-elf/debug/monitor_blinky"

sys.path.insert(0, str(ROOT / "tools"))
import monitor_smoke as monitor  # noqa: E402


DEFAULT_SYMBOLS = [
    "MON_LOOP_COUNT",
    "MON_LED_STATE",
    "MON_LAST_ERROR",
    "MON_RX_PACKETS",
    "MON_BLINK_TICKS",
    "MON_BLINK_PERIOD_MS",
    "MON_GPIO_PD06_LEVEL",
    "MON_GPIO_PD07_LEVEL",
    "MON_UPTIME_US",
    "MON_CSR_MCAUSE",
    "MON_CSR_MEPC",
    "MON_CSR_MTVAL",
    "MON_CSR_MSTATUS",
    "MON_CSR_MIE",
    "MON_CSR_MIP",
    "MON_CSR_MDCAUSE",
    "MON_CSR_PMPCFG0",
    "MON_CSR_PMPADDR0",
    "MON_CSR_PMPADDR1",
    "MON_CSR_PMACFG0",
    "MON_CSR_PMAADDR1",
    "MON_USB_STAGE",
    "MON_USB_USBSTS",
    "MON_USB_USBINTR",
    "MON_USB_PORTSC1",
    "MON_USB_OTGSC",
    "MON_USB_PHY_CTRL0",
    "MON_USB_USBCMD",
    "MON_USB_USBMODE",
    "MON_USB_IRQ_COUNT",
    "MON_USB_QTD_ADDR",
    "MON_USB_QTD_DATA_PTR",
    "MON_USB_QTD_LEN",
    "MON_USB_QTD_SYS_ADDR",
    "EXCEPTION_MCAUSE",
    "EXCEPTION_MEPC",
]


def available_ports() -> list[str]:
    ports = sorted(monitor.glob.glob("/dev/cu.usbmodem*") + monitor.glob.glob("/dev/cu.usbserial*"))
    return [port for port in ports if Path(port).exists()]


def parse_symbols(raw: str | None) -> list[str]:
    if not raw:
        return DEFAULT_SYMBOLS
    return [item.strip() for item in raw.split(",") if item.strip()]


def read_monitor(port: str, elf: Path, names: list[str], timeout: float) -> dict:
    import serial

    symbols = monitor.find_symbols(elf, names)
    names = [name for name in names if name in symbols]
    if not names:
        raise RuntimeError("no requested monitor symbols found in ELF")

    with serial.Serial(port, 115200, timeout=0.02) as ser:
        ser.reset_input_buffer()
        monitor.write_serial_frame(ser, monitor.make_packet(1, monitor.CMD_CONNECT))
        monitor.require_success(monitor.read_packet(ser, timeout), monitor.CMD_CONNECT)

        monitor.write_serial_frame(ser, monitor.make_packet(2, monitor.CMD_GETINFO))
        info_resp = monitor.read_packet(ser, timeout)
        monitor.require_success(info_resp, monitor.CMD_GETINFO)
        vid, pid, version, maxsize, memsize, max_count, threshold = monitor.struct.unpack(
            "<HHQIIHH", bytes(info_resp["payload"])
        )

        monitor.write_serial_frame(
            ser,
            monitor.make_packet(3, monitor.CMD_GETVALUE, monitor.make_getvalue_payload(names, symbols)),
        )
        values_resp = monitor.read_packet(ser, timeout)
        monitor.require_success(values_resp, monitor.CMD_GETVALUE)
        values = monitor.parse_getvalue_payload(names, bytes(values_resp["payload"]))

    return {
        "info": {
            "vid": vid,
            "pid": pid,
            "version": version,
            "maxsize": maxsize,
            "memsize": memsize,
            "max_count": max_count,
            "threshold": threshold,
        },
        "values": values,
        "symbols": {name: symbols[name] for name in names},
    }


class Handler(BaseHTTPRequestHandler):
    server_version = "HpmMonitorWeb/0.1"

    def do_GET(self) -> None:
        parsed = urlparse(self.path)
        if parsed.path == "/api/ports":
            self.write_json({"ports": available_ports(), "default_elf": str(DEFAULT_ELF)})
            return
        if parsed.path == "/api/read":
            query = parse_qs(parsed.query)
            port = query.get("port", [None])[0] or (available_ports()[0] if available_ports() else None)
            if port is None:
                self.write_json({"error": "no serial ports found"}, status=404)
                return
            elf = Path(query.get("elf", [str(DEFAULT_ELF)])[0]).expanduser()
            names = parse_symbols(query.get("symbols", [None])[0])
            timeout = float(query.get("timeout", ["2.0"])[0])
            try:
                self.write_json(read_monitor(port, elf, names, timeout) | {"port": port, "elf": str(elf)})
            except Exception as exc:
                self.write_json({"error": str(exc), "port": port, "elf": str(elf)}, status=500)
            return
        if parsed.path == "/favicon.ico":
            self.send_response(204)
            self.end_headers()
            return
        self.serve_static(parsed.path)

    def serve_static(self, path: str) -> None:
        if path == "/":
            path = "/index.html"
        target = (WEB_ROOT / path.lstrip("/")).resolve()
        if WEB_ROOT not in target.parents and target != WEB_ROOT:
            self.send_error(403)
            return
        if not target.is_file():
            self.send_error(404)
            return
        content_type = mimetypes.guess_type(target.name)[0] or "application/octet-stream"
        body = target.read_bytes()
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def write_json(self, payload: dict, status: int = 200) -> None:
        body = json.dumps(payload, ensure_ascii=False).encode()
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, fmt: str, *args) -> None:
        print(f"{self.address_string()} - {fmt % args}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8765)
    args = parser.parse_args()
    server = ThreadingHTTPServer((args.host, args.port), Handler)
    print(f"hpm monitor web: http://{args.host}:{args.port}")
    server.serve_forever()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
