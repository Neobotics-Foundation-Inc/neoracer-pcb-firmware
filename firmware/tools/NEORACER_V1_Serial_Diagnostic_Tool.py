#!/usr/bin/env python3
"""Send one or more osrcore serial commands and print command responses."""

from __future__ import annotations

import argparse
import time

from serial_ota import SerialPort, find_port


RESPONSE_PREFIXES = (
    "OK",
    "ERROR",
    "FW:",
    "FW_VERSION:",
    "Status:",
    "IMU:",
    "Heater:",
    "Diag:",
    "DIAG:",
    "Chassis:",
    "SAFETY:",
    "PID:",
    "STREAM:",
    "LEVEL:",
    "CF:",
    "CH:",
    "MAG:",
)


def read_responses(ser: SerialPort, timeout_s: float) -> list[str]:
    deadline = time.monotonic() + timeout_s
    lines: list[str] = []
    while time.monotonic() < deadline:
        raw = ser.readline(max(0.1, deadline - time.monotonic()))
        if not raw:
            continue
        line = raw.decode("utf-8", errors="replace").strip()
        if line.startswith(RESPONSE_PREFIXES):
            lines.append(line)
            deadline = time.monotonic() + 0.25
    return lines


def main() -> int:
    parser = argparse.ArgumentParser(description="Send osrcore serial commands")
    parser.add_argument("commands", nargs="+", help="commands to send, e.g. 'stream off' status 'pid get'")
    parser.add_argument("--port", help="serial port; auto-detects /dev/cu.usbmodem* if omitted")
    parser.add_argument("--baud", type=int, default=460800, help="serial baud rate")
    parser.add_argument("--timeout", type=float, default=1.0, help="response timeout per command")
    args = parser.parse_args()

    port = args.port or find_port()
    with SerialPort(port, args.baud) as ser:
        ser.drain_input()
        for command in args.commands:
            ser.write_line(command)
            for line in read_responses(ser, args.timeout):
                print(line)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
