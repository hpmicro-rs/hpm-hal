#!/usr/bin/env python3
"""Local web bridge for the HPM monitor protocol."""

from __future__ import annotations

import argparse
import json
import mimetypes
import sys
import time
from contextlib import contextmanager
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from threading import Condition
from urllib.parse import parse_qs, urlparse

ROOT = Path(__file__).resolve().parents[2]
WEB_ROOT = Path(__file__).resolve().parent / "static"
DEFAULT_ELF = ROOT / "target/riscv32imac-unknown-none-elf/release/monitor_sensors"

sys.path.insert(0, str(ROOT / "tools"))
import monitor_smoke as monitor  # noqa: E402


DEFAULT_SYMBOLS = [
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
    "MON_LOOP_COUNT",
    "MON_LED_STATE",
    "MON_LAST_ERROR",
    "MON_RX_PACKETS",
    "MON_PANIC_LINE",
    "MON_PANIC_COLUMN",
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


class BackendUsbPaused(RuntimeError):
    pass


class UsbOwnershipConflict(RuntimeError):
    pass


class UsbOwnership:
    """Serialize backend bulk access and lease the device to one WebUSB page."""

    def __init__(self, lease_seconds: float = 10.0):
        self.lease_seconds = lease_seconds
        self.condition = Condition()
        self.mode = "backend"
        self.owner: str | None = None
        self.lease_deadline = 0.0
        self.backend_active = False
        self.backend_reset_required = False

    def _expire_locked(self) -> None:
        if self.mode == "webusb" and time.monotonic() >= self.lease_deadline:
            self.mode = "backend"
            self.owner = None
            self.lease_deadline = 0.0
            self.backend_reset_required = True
            self.condition.notify_all()

    def _snapshot_locked(self) -> dict:
        self._expire_locked()
        remaining = max(0.0, self.lease_deadline - time.monotonic()) if self.mode == "webusb" else 0.0
        return {
            "mode": self.mode,
            "backend_active": self.backend_active,
            "backend_reset_required": self.backend_reset_required,
            "lease_remaining_ms": round(remaining * 1000),
        }

    def snapshot(self) -> dict:
        with self.condition:
            return self._snapshot_locked()

    def set_webusb(self, owner: str) -> dict:
        if not owner or len(owner) > 128:
            raise ValueError("valid WebUSB owner token required")
        with self.condition:
            self._expire_locked()
            if self.mode == "webusb" and self.owner != owner:
                raise UsbOwnershipConflict("USB device is leased to another WebUSB page")
            self.mode = "webusb"
            self.owner = owner
            self.lease_deadline = time.monotonic() + self.lease_seconds
            self.condition.notify_all()
            while self.backend_active:
                self.condition.wait()
            return self._snapshot_locked()

    def set_backend(self, owner: str, clean: bool = True) -> dict:
        with self.condition:
            self._expire_locked()
            if self.mode == "webusb" and self.owner != owner:
                raise UsbOwnershipConflict("WebUSB lease owner mismatch")
            self.mode = "backend"
            self.owner = None
            self.lease_deadline = 0.0
            self.backend_reset_required = self.backend_reset_required or not clean
            self.condition.notify_all()
            return self._snapshot_locked()

    @contextmanager
    def backend_session(self):
        with self.condition:
            self._expire_locked()
            while self.backend_active and self.mode == "backend":
                self.condition.wait()
                self._expire_locked()
            if self.mode != "backend":
                raise BackendUsbPaused("backend USB access is paused while WebUSB owns the device")
            self.backend_active = True
            reset_required = self.backend_reset_required
            self.backend_reset_required = False
        try:
            yield reset_required
        finally:
            with self.condition:
                self.backend_active = False
                self.condition.notify_all()


USB_OWNERSHIP = UsbOwnership()


def reset_usb_device(vid: int, pid: int) -> None:
    import usb.core
    import usb.util

    device = usb.core.find(idVendor=vid, idProduct=pid)
    if device is None:
        raise RuntimeError(f"USB device 0x{vid:04x}:0x{pid:04x} missing during takeover reset")
    try:
        device.reset()
    finally:
        usb.util.dispose_resources(device)
    time.sleep(0.5)


def available_ports() -> list[str]:
    ports = sorted(monitor.glob.glob("/dev/cu.usbmodem*") + monitor.glob.glob("/dev/cu.usbserial*"))
    return [port for port in ports if Path(port).exists()]


def parse_symbols(raw: str | None) -> list[str]:
    if not raw:
        return DEFAULT_SYMBOLS
    return [item.strip() for item in raw.split(",") if item.strip()]


def read_monitor(
    transport_name: str,
    port: str | None,
    elf: Path,
    names: list[str],
    timeout: float,
    vid: int,
    pid: int,
) -> dict:
    started = time.perf_counter()
    symbols = monitor.find_symbols(elf, names)
    names = [name for name in names if name in symbols]
    if not names:
        raise RuntimeError("no requested monitor symbols found in ELF")

    if transport_name == "bulk":
        with monitor.BulkMonitorTransport(vid, pid, timeout) as transport:
            info, values = monitor.run_monitor_query(transport, names, symbols, timeout)
        port_value = None
    else:
        import serial

        if port is None:
            raise RuntimeError("no serial ports found")
        with serial.Serial(port, 115200, timeout=0.02) as ser:
            ser.reset_input_buffer()
            info, values = monitor.run_monitor_query(monitor.SerialMonitorTransport(ser), names, symbols, timeout)
        port_value = port

    return {
        "info": info,
        "values": values,
        "symbols": {name: symbols[name] for name in names},
        "transport": transport_name,
        "port": port_value,
        "sampled_at": datetime.now(timezone.utc).isoformat(),
        "duration_ms": round((time.perf_counter() - started) * 1000, 1),
    }


class Handler(BaseHTTPRequestHandler):
    server_version = "HpmMonitorWeb/0.1"

    def do_GET(self) -> None:
        parsed = urlparse(self.path)
        if parsed.path == "/api/ports":
            self.write_json(
                {"ports": available_ports(), "default_elf": str(DEFAULT_ELF), "usb": USB_OWNERSHIP.snapshot()}
            )
            return
        if parsed.path == "/api/usb-mode":
            self.write_json(USB_OWNERSHIP.snapshot())
            return
        if parsed.path == "/api/symbols":
            query = parse_qs(parsed.query)
            elf = Path(query.get("elf", [str(DEFAULT_ELF)])[0]).expanduser()
            names = parse_symbols(query.get("symbols", [None])[0])
            try:
                symbols = monitor.find_symbols(elf, names)
                self.write_json({"elf": str(elf), "symbols": symbols})
            except Exception as exc:
                self.write_json({"error": str(exc), "elf": str(elf)}, status=500)
            return
        if parsed.path == "/api/read":
            query = parse_qs(parsed.query)
            transport = query.get("transport", ["serial"])[0]
            port = query.get("port", [None])[0] or (available_ports()[0] if available_ports() else None)
            if transport == "serial" and port is None:
                self.write_json({"error": "no serial ports found"}, status=404)
                return
            elf = Path(query.get("elf", [str(DEFAULT_ELF)])[0]).expanduser()
            names = parse_symbols(query.get("symbols", [None])[0])
            timeout = float(query.get("timeout", ["2.0"])[0])
            vid = int(query.get("vid", ["0x1209"])[0], 0)
            pid = int(query.get("pid", ["0x5e31"])[0], 0)
            try:
                if transport == "bulk":
                    with USB_OWNERSHIP.backend_session() as reset_required:
                        if reset_required:
                            reset_usb_device(vid, pid)
                        result = read_monitor(transport, port, elf, names, timeout, vid, pid)
                else:
                    result = read_monitor(transport, port, elf, names, timeout, vid, pid)
                self.write_json(result | {"elf": str(elf)})
            except BackendUsbPaused as exc:
                self.write_json({"error": str(exc), "usb": USB_OWNERSHIP.snapshot()}, status=409)
            except Exception as exc:
                self.write_json({"error": str(exc), "transport": transport, "port": port, "elf": str(elf)}, status=500)
            return
        if parsed.path == "/favicon.ico":
            self.send_response(204)
            self.end_headers()
            return
        self.serve_static(parsed.path)

    def do_POST(self) -> None:
        parsed = urlparse(self.path)
        if parsed.path != "/api/usb-mode":
            self.write_json({"error": "unknown API endpoint"}, status=404)
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
            if length < 1 or length > 4096:
                raise ValueError("invalid JSON body length")
            payload = json.loads(self.rfile.read(length))
            mode = payload.get("mode")
            owner = payload.get("owner", "")
            clean = payload.get("clean", True)
            if not isinstance(clean, bool):
                raise ValueError("clean must be a boolean")
            if mode == "webusb":
                result = USB_OWNERSHIP.set_webusb(owner)
            elif mode == "backend":
                result = USB_OWNERSHIP.set_backend(owner, clean=clean)
            else:
                raise ValueError("mode must be backend or webusb")
            self.write_json(result)
        except UsbOwnershipConflict as exc:
            self.write_json({"error": str(exc), "usb": USB_OWNERSHIP.snapshot()}, status=409)
        except (ValueError, TypeError, json.JSONDecodeError) as exc:
            self.write_json({"error": str(exc)}, status=400)

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
        try:
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def log_message(self, fmt: str, *args) -> None:
        print(f"{self.address_string()} - {fmt % args}")


def main() -> int:
    global DEFAULT_ELF

    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--elf", type=Path, default=DEFAULT_ELF)
    args = parser.parse_args()
    DEFAULT_ELF = args.elf.expanduser().resolve()
    if not DEFAULT_ELF.is_file():
        parser.error(f"monitor ELF not found: {DEFAULT_ELF}")
    server = ThreadingHTTPServer((args.host, args.port), Handler)
    print(f"hpm monitor web: http://{args.host}:{args.port} (elf={DEFAULT_ELF})")
    server.serve_forever()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
