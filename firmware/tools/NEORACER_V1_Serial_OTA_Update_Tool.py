#!/usr/bin/env python3
"""Serial OTA updater for osrcore app images.

This updates only the application OTA slot. The device must already be running
an osrcore build with the `fw` serial commands and OTA partition table.
"""

from __future__ import annotations

import argparse
import errno
import fcntl
import glob
import hashlib
import os
import select
import struct
import sys
import termios
import time
from pathlib import Path


CHUNK_SIZE = 128
MAX_CHUNK_SIZE = 384
IOSSIOSPEED = 0x80045402  # macOS custom serial baud rate ioctl


class SerialPort:
    def __init__(self, port: str, baud: int):
        self.fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        self._old_attrs = termios.tcgetattr(self.fd)
        attrs = termios.tcgetattr(self.fd)
        baud_const = getattr(termios, f"B{baud}", None)
        custom_baud = baud_const is None
        if custom_baud:
            baud_const = termios.B9600

        attrs[0] = 0
        attrs[1] = 0
        attrs[2] = termios.CLOCAL | termios.CREAD | termios.CS8
        attrs[3] = 0
        attrs[4] = baud_const
        attrs[5] = baud_const
        attrs[6][termios.VMIN] = 0
        attrs[6][termios.VTIME] = 0
        termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
        if custom_baud:
            try:
                fcntl.ioctl(self.fd, IOSSIOSPEED, struct.pack("I", baud))
            except OSError as exc:
                raise SystemExit(f"ERROR: unsupported baud rate {baud}: {exc}") from exc
        termios.tcflush(self.fd, termios.TCIOFLUSH)

    def close(self) -> None:
        if self.fd is not None:
            termios.tcsetattr(self.fd, termios.TCSANOW, self._old_attrs)
            os.close(self.fd)
            self.fd = None

    def __enter__(self) -> "SerialPort":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def write_line(self, line: str) -> None:
        data = line.encode("ascii") + b"\n"
        while data:
            _, writable, _ = select.select([], [self.fd], [], 2.0)
            if not writable:
                raise TimeoutError("timeout writing serial port")
            n = os.write(self.fd, data)
            data = data[n:]

    def readline(self, timeout_s: float) -> bytes:
        deadline = time.monotonic() + timeout_s
        buf = bytearray()
        while time.monotonic() < deadline:
            readable, _, _ = select.select([self.fd], [], [], max(0.0, deadline - time.monotonic()))
            if not readable:
                continue
            try:
                chunk = os.read(self.fd, 256)
            except BlockingIOError as exc:
                if exc.errno in (errno.EAGAIN, errno.EWOULDBLOCK):
                    time.sleep(0.01)
                    continue
                raise
            if not chunk:
                continue
            for b in chunk:
                if b == 10:
                    return bytes(buf)
                buf.append(b)
        return bytes(buf)

    def drain_input(self, timeout_s: float = 0.2) -> None:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            readable, _, _ = select.select([self.fd], [], [], max(0.0, deadline - time.monotonic()))
            if not readable:
                return
            try:
                chunk = os.read(self.fd, 512)
            except BlockingIOError as exc:
                if exc.errno in (errno.EAGAIN, errno.EWOULDBLOCK):
                    time.sleep(0.01)
                    continue
                raise
            if not chunk:
                return


def find_port() -> str:
    ports = sorted(glob.glob("/dev/cu.usbmodem*") + glob.glob("/dev/cu.SLAB_USBtoUART*"))
    if not ports:
        raise SystemExit("ERROR: no serial port found; pass --port /dev/cu.usbmodemXXXX")
    if len(ports) > 1:
        raise SystemExit("ERROR: multiple serial ports found; pass --port explicitly:\n" + "\n".join(ports))
    return ports[0]


def read_fw_response(ser: SerialPort, timeout_s: float = 5.0) -> str:
    deadline = time.monotonic() + timeout_s
    seen_lines: list[str] = []
    while time.monotonic() < deadline:
        raw = ser.readline(max(0.1, deadline - time.monotonic()))
        if not raw:
            continue
        line = raw.decode("utf-8", errors="replace").strip()
        if line.startswith("OK fw") or line.startswith("ERROR fw"):
            return line
        if line:
            seen_lines.append(line)
            seen_lines = seen_lines[-5:]
            if line.startswith("ERROR: unknown command"):
                raise TimeoutError(
                    "device does not support fw commands; flash the OTA full firmware once first"
                )

    detail = ""
    if seen_lines:
        detail = " Last serial lines:\n  " + "\n  ".join(seen_lines)
    raise TimeoutError("timeout waiting for fw response." + detail)


def send_line(ser: SerialPort, line: str) -> str:
    ser.write_line(line)
    return read_fw_response(ser)


def main() -> int:
    parser = argparse.ArgumentParser(description="Update osrcore app firmware over serial OTA")
    parser.add_argument("image", type=Path, help="app image, usually build/osrcore.bin")
    parser.add_argument("--port", help="serial port; auto-detects /dev/cu.usbmodem* if omitted")
    parser.add_argument("--baud", type=int, default=460800, help="serial baud rate")
    parser.add_argument("--chunk", type=int, default=CHUNK_SIZE, help="raw bytes per fw data frame")
    parser.add_argument("--force-low-voltage", action="store_true",
                        help="allow OTA when the firmware reports low battery voltage")
    args = parser.parse_args()

    image = args.image
    if not image.exists():
        raise SystemExit(f"ERROR: image not found: {image}")
    if "full" in image.name or "merged" in image.name:
        raise SystemExit(
            "ERROR: serial OTA updates the app image only. "
            "Use build/osrcore.bin, not the full/merged firmware image."
        )
    data = image.read_bytes()
    digest = hashlib.sha256(data).hexdigest()

    if args.chunk <= 0 or args.chunk > MAX_CHUNK_SIZE:
        raise SystemExit(f"ERROR: --chunk must be 1..{MAX_CHUNK_SIZE}")

    port = args.port or find_port()
    print(f"Port: {port}")
    print(f"Image: {image} ({len(data)} bytes)")
    print(f"SHA256: {digest}")

    with SerialPort(port, args.baud) as ser:
        ser.drain_input()
        try:
            send_line(ser, "fw abort")
            ser.drain_input()
        except Exception:
            ser.drain_input()

        force = " force" if args.force_low_voltage else ""
        resp = send_line(ser, f"fw begin {len(data)} {digest}{force}")
        print(resp)
        if not resp.startswith("OK fw begin"):
            return 1

        total_chunks = (len(data) + args.chunk - 1) // args.chunk
        try:
            for seq, start in enumerate(range(0, len(data), args.chunk)):
                chunk = data[start : start + args.chunk]
                resp = send_line(ser, f"fw data {seq} {chunk.hex()}")
                if not resp.startswith(f"OK fw data {seq} "):
                    print(resp, file=sys.stderr)
                    try:
                        send_line(ser, "fw abort")
                    except Exception:
                        pass
                    return 1
                if seq == 0 or (seq + 1) == total_chunks or (seq + 1) % 25 == 0:
                    pct = (seq + 1) * 100.0 / total_chunks
                    print(f"Progress: {seq + 1}/{total_chunks} ({pct:.1f}%)")
        except Exception:
            try:
                send_line(ser, "fw abort")
            except Exception:
                pass
            raise

        resp = send_line(ser, "fw end",)
        print(resp)
        if not resp.startswith("OK fw reboot"):
            try:
                send_line(ser, "fw abort")
            except Exception:
                pass
            return 1

    print("Done. Device is rebooting into the updated app.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
