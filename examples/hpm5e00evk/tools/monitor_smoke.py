#!/usr/bin/env python3
"""Smoke test for the HPM Monitor V2 subset used by monitor_usb_lite."""

from __future__ import annotations

import argparse
import glob
import shutil
import struct
import subprocess
import sys
import time
from pathlib import Path

START = 0x4850
MAGIC = 0x4D44
VERSION = 2
END = 0xA55A
HOST_MASK = 0x8000
ACK_MASK = 0x8000

CMD_CONNECT = 0x0001
CMD_GETINFO = 0x0002
CMD_GETVALUE = 0x00A0

TYPE_UINT32 = 0x05
RESULT_SUCCESS = 0x00FF

PROFILE_MIN_SIZE = 34
CRC_OFFSET = 10
HEADER_CRC_OFFSET = 14
PAYLOAD_OFFSET = 32


def crc32(data: bytes) -> int:
    polynomial = 0xBEAF5AA5
    crc = 0xFFFFFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            mask = -(crc & 1) & 0xFFFFFFFF
            crc = ((crc >> 1) ^ (polynomial & mask)) & 0xFFFFFFFF
    return crc


def make_packet(index: int, cmd: int, payload: bytes = b"", result: int = 0) -> bytes:
    frame = bytearray(PROFILE_MIN_SIZE + len(payload))
    struct.pack_into("<HHHI", frame, 0, START, MAGIC, VERSION, 0)
    struct.pack_into("<HHHII", frame, 14, index, cmd | HOST_MASK, result, 0, 0)
    struct.pack_into("<I", frame, 28, len(payload))
    frame[PAYLOAD_OFFSET : PAYLOAD_OFFSET + len(payload)] = payload
    struct.pack_into("<H", frame, len(frame) - 2, END)
    struct.pack_into("<I", frame, CRC_OFFSET, crc32(frame[HEADER_CRC_OFFSET:]))
    return bytes(frame)


def write_serial_frame(ser, frame: bytes) -> None:
    for offset in range(0, len(frame), 64):
        ser.write(frame[offset : offset + 64])
        ser.flush()


def read_packet(ser, timeout: float) -> dict[str, int | bytes]:
    deadline = time.monotonic() + timeout
    buf = bytearray()
    while time.monotonic() < deadline:
        chunk = ser.read(1)
        if chunk:
            buf += chunk
            pos = find_header(buf)
            if pos > 0:
                del buf[:pos]
            if len(buf) >= PROFILE_MIN_SIZE:
                length = struct.unpack_from("<I", buf, 28)[0]
                total = PROFILE_MIN_SIZE + length
                if len(buf) >= total:
                    frame = bytes(buf[:total])
                    return parse_packet(frame)
        else:
            time.sleep(0.005)
    raise TimeoutError("timeout waiting for monitor response")


def find_header(buf: bytearray) -> int:
    for i in range(max(0, len(buf) - 3)):
        if struct.unpack_from("<HH", buf, i) == (START, MAGIC):
            return i
    return 0


def parse_packet(frame: bytes) -> dict[str, int | bytes]:
    start, magic, version = struct.unpack_from("<HHH", frame, 0)
    if (start, magic, version) != (START, MAGIC, VERSION):
        raise ValueError("bad monitor header")
    length = struct.unpack_from("<I", frame, 28)[0]
    if len(frame) != PROFILE_MIN_SIZE + length:
        raise ValueError("bad monitor frame length")
    if struct.unpack_from("<H", frame, len(frame) - 2)[0] != END:
        raise ValueError("bad monitor trailer")
    expected_crc = struct.unpack_from("<I", frame, CRC_OFFSET)[0]
    actual_crc = crc32(frame[HEADER_CRC_OFFSET:])
    if expected_crc != actual_crc:
        raise ValueError(f"bad crc: expected 0x{expected_crc:08x}, got 0x{actual_crc:08x}")
    index, cmd, result, timing, _extend2 = struct.unpack_from("<HHHII", frame, 14)
    return {
        "index": index,
        "cmd": cmd,
        "result": result,
        "timing": timing,
        "payload": frame[PAYLOAD_OFFSET : PAYLOAD_OFFSET + length],
    }


def require_success(resp: dict[str, int | bytes], cmd: int) -> None:
    result = int(resp["result"])
    if int(resp["cmd"]) != cmd:
        raise ValueError(f"unexpected cmd 0x{int(resp['cmd']):04x}")
    if result != (ACK_MASK | RESULT_SUCCESS):
        raise ValueError(f"monitor command failed: result=0x{result:04x}")


def make_getvalue_payload(names: list[str], symbols: dict[str, int]) -> bytes:
    items = bytearray()
    for name in names:
        addr = symbols[name]
        items += struct.pack("<BBHI", TYPE_UINT32, 0, 0, addr)
        items += b"\x00\x00\x00\x00"
    return struct.pack("<HBBII", len(names), 0, 0, len(names), 0) + bytes(items)


def parse_getvalue_payload(names: list[str], payload: bytes) -> dict[str, int]:
    count = struct.unpack_from("<H", payload, 0)[0]
    if count != len(names):
        raise ValueError(f"unexpected value count {count}")
    values: dict[str, int] = {}
    offset = 12
    for name in names:
        ty, _rsvd1, _rsvd2, _addr = struct.unpack_from("<BBHI", payload, offset)
        if ty != TYPE_UINT32:
            raise ValueError(f"{name}: unexpected type 0x{ty:02x}")
        values[name] = struct.unpack_from("<I", payload, offset + 8)[0]
        offset += 12
    return values


def find_symbols(elf: Path, names: list[str]) -> dict[str, int]:
    nm = shutil.which("riscv64-unknown-elf-nm") or shutil.which("llvm-nm") or shutil.which("nm")
    if nm is None:
        raise RuntimeError("nm tool missing")
    output = subprocess.check_output([nm, "-g", str(elf)], text=True)
    symbols: dict[str, int] = {}
    wanted = set(names)
    for line in output.splitlines():
        parts = line.split()
        if len(parts) >= 3 and parts[2] in wanted:
            symbols[parts[2]] = int(parts[0], 16)
    return symbols


def default_port() -> str | None:
    candidates = glob.glob("/dev/tty.usbmodem*") + glob.glob("/dev/tty.usbserial*")
    return sorted(candidates)[0] if candidates else None


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default=default_port())
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument(
        "--elf",
        type=Path,
        default=Path("target/riscv32imafc-unknown-none-elf/debug/monitor_uart_lite"),
    )
    parser.add_argument("--timeout", type=float, default=2.0)
    args = parser.parse_args()

    if args.port is None:
        raise SystemExit("serial port missing; pass --port /dev/tty...")

    try:
        import serial
    except ImportError as exc:
        raise SystemExit("pyserial is required: python3 -m pip install pyserial") from exc

    names = [
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
    symbols = find_symbols(args.elf, names)
    names = [name for name in names if name in symbols]
    if not names:
        raise SystemExit("no monitor symbols found in ELF")

    with serial.Serial(args.port, args.baud, timeout=0.02) as ser:
        ser.reset_input_buffer()
        write_serial_frame(ser, make_packet(1, CMD_CONNECT))
        require_success(read_packet(ser, args.timeout), CMD_CONNECT)

        write_serial_frame(ser, make_packet(2, CMD_GETINFO))
        info = read_packet(ser, args.timeout)
        require_success(info, CMD_GETINFO)
        vid, pid, version, maxsize, memsize, max_count, threshold = struct.unpack(
            "<HHQIIHH", bytes(info["payload"])
        )
        print(
            f"info: vid=0x{vid:04x} pid=0x{pid:04x} version={version} "
            f"maxsize={maxsize} memsize={memsize} max_count={max_count} threshold={threshold}"
        )

        write_serial_frame(ser, make_packet(3, CMD_GETVALUE, make_getvalue_payload(names, symbols)))
        values = read_packet(ser, args.timeout)
        require_success(values, CMD_GETVALUE)
        for name, value in parse_getvalue_payload(names, bytes(values["payload"])).items():
            print(f"{name}=0x{value:08x} ({value})")

    return 0


if __name__ == "__main__":
    sys.exit(main())
