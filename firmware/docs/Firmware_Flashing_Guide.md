# OSRCORE Firmware Flashing Guide

Technical support: winter@osrbot.com

This guide covers firmware flashing, app updates, and field troubleshooting for
the Neoracer V1 / OSRCORE ESP32-S3 development board.

## Flashing Modes

| Mode | Use Case | BOOT Required | Writes |
|---|---|---:|---|
| Factory restore / first flash | First install, factory restore, unbootable firmware, bootloader or partition-table updates | Yes | bootloader, partition table, OTA data, app |
| App update | Device is already running an osrcore firmware that supports the `fw` protocol | No | app slot |

Factory restore recreates the full flash layout. App update writes only the
inactive OTA app slot and keeps the bootloader, partition table, flash settings,
and NVS calibration/PID parameters.

## Preparation

- Use a stable USB data cable and stable power.
- Host serial baud: `460800`.
- macOS ports usually look like `/dev/cu.usbmodemXXXX`.
- Orin Nano / Linux ports usually look like `/dev/ttyACM0` or `/dev/ttyUSB0`.
- For bench recovery or tests, keep battery voltage above 10.8 V when possible.
  For low-voltage bench tests, use the updater's low-voltage override option.

Find the serial port:

```bash
ls /dev/cu.usbmodem*
ls /dev/ttyACM* /dev/ttyUSB*
```

## Factory Restore / First Flash

From the delivery package root, enter the firmware package folder:

```bash
cd 01_Firmware/Firmware_Package_20260628
```

If the current terminal is already in `English_Documents/`, run:

```bash
cd ..
```

Put the device into ESP32-S3 ROM BOOT download mode, then write the delivered merged full image at address `0x0`:

```bash
python -m esptool \
  --chip esp32s3 -p /dev/cu.usbmodemXXXX -b 460800 \
  --before default-reset --after hard-reset \
  write-flash --flash-mode dio --flash-size 16MB --flash-freq 80m \
  0x0 Firmware_Files/NEORACER_V1_OSCORE_NEO_ESP32S3_IDF_FullFlash_RevA_20260628.bin
```

## App Update

When the device is already running osrcore, update the app without entering BOOT. Run the following commands from `01_Firmware/Firmware_Package_20260628/`:

```bash
python3 Flashing_and_Diagnostic_Tools/NEORACER_V1_Serial_OTA_Update_Tool.py \
  --port /dev/cu.usbmodemXXXX \
  Firmware_Files/NEORACER_V1_OSCORE_NEO_ESP32S3_IDF_Project_RevA_20260628.bin
```

For low-voltage bench tests:

```bash
python3 Flashing_and_Diagnostic_Tools/NEORACER_V1_Serial_OTA_Update_Tool.py \
  --port /dev/cu.usbmodemXXXX \
  --force-low-voltage \
  Firmware_Files/NEORACER_V1_OSCORE_NEO_ESP32S3_IDF_Project_RevA_20260628.bin
```

## Post-Flash Check

Run the following commands from `01_Firmware/Firmware_Package_20260628/`:

```bash
python3 Flashing_and_Diagnostic_Tools/NEORACER_V1_Serial_Diagnostic_Tool.py --port /dev/cu.usbmodemXXXX \
  "stream off" "fw version" "sn get" "status" "diag" "s" "b" "pid get" "odom get"
```

Verify at minimum:

- `fw version` reports the release date, build identifier, build time, and IDF version.
- `sn get` returns the ESP32-S3 hardware serial number.
- `status` reports speed, voltage, control mode, IMU, heater, safety summary,
  and chassis calibration.
- `diag` returns a compact diagnostic code line for OSRBOT support tools.
- `s` returns a synchronized odom/IMU snapshot.
- `b` returns battery voltage.
- `pid get` and `odom get` respond normally.

Optional internal host-link check:

```bash
python3 Flashing_and_Diagnostic_Tools/NEORACER_V1_Serial_Diagnostic_Tool.py --port /dev/cu.usbmodemXXXX \
  "stream off" "link up ros" "diag" "link down ros" "diag"
```

`link ping ros` is intentionally silent and is intended for ROS host heartbeat use.

## Troubleshooting

| Symptom | Action |
|---|---|
| `timeout waiting for response` | The serial port is likely held by `screen`, ROS, the web flasher, or another process. Close it and retry. |
| `ERROR fw begin invalid` | The current app does not support this serial OTA attempt, or the image does not match. Use factory restore / full flashing. |
| `Invalid head of packet (0x45)` | The device state and flashing mode likely do not match, or stale serial output interfered. Close the monitor, reselect the port, and retry. |
| Firmware file is missing | Confirm the current directory is `01_Firmware/Firmware_Package_20260628/`, then check that `Firmware_Files/` is complete. |
| Web serial monitor has no output after flashing | The browser serial handle may become stale after reset. Reselect the serial port and open the monitor again. |
