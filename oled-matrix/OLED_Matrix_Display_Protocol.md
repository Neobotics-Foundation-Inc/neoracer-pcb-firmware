# OLED Matrix Display Protocol

Date: 2026-06-29  
Technical support: winter@osrbot.com

## Overview

This document describes the OLED matrix display communication used by the current NEORACER V1 delivery version. The protocol matches the current host driver:

- `led_matrix.py`
- `led_matrix.launch.py`

## Serial Parameters

| Parameter | Value |
| --- | --- |
| Default device | `/dev/osrbot_led_matrix` |
| Default baud rate | `115200` |
| Data bits | 8 |
| Parity | None |
| Stop bits | 1 |
| Flow control | None |

## udev Binding And Device Check

The OLED matrix display is expected to be bound to `/dev/osrbot_led_matrix` through udev rules. Without the udev rules, the system may only expose `/dev/ttyACM*` or `/dev/ttyUSB*`, and the default host launch parameters will not find the display.

Run from the host software source directory:

```bash
cd osracer_bringup/script/udev
sudo bash install_udev.sh
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Reconnect the OLED matrix display, then verify:

```bash
ls -l /dev/osrbot_led_matrix
```

Rule file:

```text
osracer_bringup/script/udev/99-osrbot-led-matrix.rules
```

If the device name is not created, temporarily check enumerated serial devices:

```bash
ls /dev/ttyACM* /dev/ttyUSB*
dmesg | tail -n 50
```

## ROS Interface

| Type | Topic | Description |
| --- | --- | --- |
| `std_msgs/String` | `led_matrix/command` | Pass-through text or command string |
| `std_msgs/UInt8MultiArray` | `led_matrix/columns` | Column data for the matrix display |

## Command Format

### Set Number of Chained Modules

```text
N <num_matrices>#
```

- `<num_matrices>`: 1 to 4.
- The host calculates total columns as `num_matrices * 8`.
- This command is sent automatically after serial initialization.

### Send Column Data

```text
P<hex_columns>#
```

- Each column is 1 byte, encoded as two hexadecimal digits.
- `bit7` is the bottom row, and `bit0` is the top row.
- Extra columns are truncated. Missing columns are padded with `0x00`.

Example: 8 columns off.

```text
P0000000000000000#
```

Example: 8 columns on.

```text
PFFFFFFFFFFFFFFFF#
```

### Set Brightness

```text
L <brightness>#
```

- `<brightness>`: 0 to 15.

### Set Scroll Speed

```text
S <speed_ms>#
```

- `<speed_ms>`: 10 to 1000 ms.

### Display Text

Send text directly. The host appends a newline by default.

```text
HELLO
```

## Host Fault Tolerance

- If the serial port is unavailable, the host retries at `reconnect_interval_s`, default 2 seconds.
- If serial write fails, the host closes the port and waits for the next reconnect attempt.
- Column data is truncated if too long and padded with zeros if too short.

## Usage Notes

- Command messages use `#` as the terminator.
- Text display appends a newline by default.
- Brightness range is 0 to 15.
- Scroll speed range is 10 to 1000 ms.
- With multiple chained modules, total columns are calculated as `num_matrices * 8`.
- Self-deployed environments must complete udev binding before starting `led_matrix.launch.py`.
