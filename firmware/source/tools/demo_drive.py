#!/usr/bin/env python3
"""Simple serial demos for the OSRacer Ackermann chassis.

The demos use the existing osrcore USB CDC serial protocol, so they do not
require ROS or a custom firmware build. Motion demos are intentionally slow and
always send a stop command on exit.
"""

from __future__ import annotations

import argparse
import glob
import sys
import time
from dataclasses import dataclass

from serial_ota import SerialPort


DEFAULT_BAUD = 460800
DEFAULT_SPEED = 0.20
MAX_DEMO_SPEED = 0.50
MAX_DEMO_STEER = 22.0

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


@dataclass(frozen=True)
class Step:
    speed: float
    steer: float
    duration: float
    label: str


def find_serial_port() -> str:
    patterns = (
        "/dev/cu.usbmodem*",
        "/dev/cu.SLAB_USBtoUART*",
        "/dev/ttyACM*",
        "/dev/ttyUSB*",
    )
    ports: list[str] = []
    for pattern in patterns:
        ports.extend(glob.glob(pattern))
    ports = sorted(set(ports))
    if not ports:
        raise SystemExit("ERROR: no serial port found; pass --port explicitly")
    if len(ports) > 1:
        raise SystemExit("ERROR: multiple serial ports found; pass --port:\n" + "\n".join(ports))
    return ports[0]


def clamp_motion(speed: float, steer: float) -> tuple[float, float]:
    speed = max(-MAX_DEMO_SPEED, min(MAX_DEMO_SPEED, speed))
    steer = max(-MAX_DEMO_STEER, min(MAX_DEMO_STEER, steer))
    return speed, steer


def read_responses(ser: SerialPort, timeout_s: float = 0.35) -> list[str]:
    deadline = time.monotonic() + timeout_s
    lines: list[str] = []
    while time.monotonic() < deadline:
        raw = ser.readline(max(0.05, deadline - time.monotonic()))
        if not raw:
            continue
        line = raw.decode("utf-8", errors="replace").strip()
        if line.startswith(RESPONSE_PREFIXES):
            lines.append(line)
            deadline = time.monotonic() + 0.08
    return lines


def send_command(ser: SerialPort, command: str, timeout_s: float = 0.35, quiet: bool = False) -> list[str]:
    if not quiet:
        print(f"> {command}")
    ser.write_line(command)
    lines = read_responses(ser, timeout_s)
    if not quiet:
        for line in lines:
            print(line)
    return lines


def stop(ser: SerialPort, quiet: bool = False) -> None:
    try:
        send_command(ser, "v 0 0", quiet=quiet)
    except Exception as exc:
        if not quiet:
            print(f"WARN: failed to send stop command: {exc}", file=sys.stderr)


def run_step(ser: SerialPort, step: Step, dry_run: bool = False) -> None:
    speed, steer = clamp_motion(step.speed, step.steer)
    print(f"\n[{step.label}] speed={speed:.2f}m/s steer={steer:.1f}deg duration={step.duration:.1f}s")
    if dry_run:
        return

    send_command(ser, f"v {speed:.3f} {steer:.2f}", timeout_s=0.5)
    deadline = time.monotonic() + step.duration
    next_status = time.monotonic()
    while time.monotonic() < deadline:
        now = time.monotonic()
        if now >= next_status:
            send_command(ser, "status", timeout_s=0.45)
            next_status = now + 1.0
        time.sleep(0.05)


def build_plan(name: str, speed: float, duration: float, loops: int) -> list[Step]:
    if name == "straight":
        return [Step(speed, 0.0, duration, "straight")]
    if name == "arc-left":
        return [Step(speed, 12.0, duration, "gentle left arc")]
    if name == "arc-right":
        return [Step(speed, -12.0, duration, "gentle right arc")]
    if name == "figure8":
        plan: list[Step] = []
        for i in range(loops):
            plan.append(Step(speed, 14.0, duration, f"figure8 loop {i + 1} left"))
            plan.append(Step(speed, -14.0, duration, f"figure8 loop {i + 1} right"))
        return plan
    if name == "patrol":
        plan = []
        for i in range(loops):
            plan.extend([
                Step(speed, 0.0, duration, f"patrol {i + 1} straight out"),
                Step(speed, 13.0, max(1.0, duration * 0.7), f"patrol {i + 1} turn left"),
                Step(speed, 0.0, duration, f"patrol {i + 1} straight back"),
                Step(speed, -13.0, max(1.0, duration * 0.7), f"patrol {i + 1} turn right"),
            ])
        return plan
    raise SystemExit(f"ERROR: unknown demo: {name}")


def run_health(ser: SerialPort) -> None:
    commands = ("stream off", "status", "pid", "cf", "level get", "mc get", "ch")
    for command in commands:
        send_command(ser, command, timeout_s=0.8)


def confirm_motion(args: argparse.Namespace, plan: list[Step]) -> None:
    if args.yes or args.dry_run:
        return
    print("\nMotion plan:")
    for step in plan:
        speed, steer = clamp_motion(step.speed, step.steer)
        print(f"  - {step.label}: {speed:.2f}m/s, {steer:.1f}deg, {step.duration:.1f}s")
    print("\nPut the car on a clear floor or lift it safely. Press Enter to start, Ctrl-C to cancel.")
    input()


def main() -> int:
    parser = argparse.ArgumentParser(description="Run simple OSRacer serial demos")
    parser.add_argument("demo", choices=("health", "stop", "straight", "arc-left", "arc-right", "figure8", "patrol"))
    parser.add_argument("--port", help="serial port; auto-detects macOS/Linux USB serial if omitted")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="serial baud rate")
    parser.add_argument("--speed", type=float, default=DEFAULT_SPEED, help="demo speed in m/s")
    parser.add_argument("--duration", type=float, default=3.0, help="step duration in seconds")
    parser.add_argument("--loops", type=int, default=2, help="loop count for figure8/patrol")
    parser.add_argument("--yes", action="store_true", help="start motion without interactive confirmation")
    parser.add_argument("--dry-run", action="store_true", help="print the planned commands without opening serial")
    args = parser.parse_args()

    if args.duration <= 0:
        raise SystemExit("ERROR: --duration must be positive")
    if args.loops <= 0:
        raise SystemExit("ERROR: --loops must be positive")

    if args.demo in ("health", "stop"):
        plan: list[Step] = []
    else:
        plan = build_plan(args.demo, args.speed, args.duration, args.loops)
        confirm_motion(args, plan)

    if args.dry_run:
        for step in plan:
            speed, steer = clamp_motion(step.speed, step.steer)
            print(f"v {speed:.3f} {steer:.2f}  # {step.duration:.1f}s, {step.label}")
        print("v 0 0")
        return 0

    port = args.port or find_serial_port()
    print(f"Port: {port}")
    with SerialPort(port, args.baud) as ser:
        ser.drain_input()
        if args.demo == "health":
            run_health(ser)
            return 0
        if args.demo == "stop":
            stop(ser)
            return 0

        try:
            send_command(ser, "stream off", timeout_s=0.6)
            for step in plan:
                run_step(ser, step)
        finally:
            print("\nStopping vehicle")
            stop(ser)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
