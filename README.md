# NeoRacer PCB Firmware

Firmware, flashing tools, and control-board documentation for the OSCORE_NEO board, the ESP32-S3 controller inside every NeoRacer V1.

This is the low-level brain of the car. It reads the encoder, runs a closed-loop speed controller, drives the steering servo and the ESC, fuses the IMU, decodes the SBUS receiver, and talks to the Jetson (or any host) over USB serial. If you want to change how fast the car goes, how tightly it holds a commanded speed, or how it steers straight, this is where those numbers live.

The firmware is `osrcore`, built with ESP-IDF for the ESP32-S3. It was developed for NeoRacer V1 by Seeed Studio and OSRBOT and is published here so anyone reflashing a car understands their options before they touch it.

## What is in here

| Path | What it is |
|------|------------|
| `firmware/source/` | Full ESP-IDF project source, buildable as delivered. Every parameter in this README traces back to `firmware/source/main/config.h`. |
| `firmware/binaries/` | Prebuilt images: the full-flash image (bootloader + partitions + app) and the app-only image for OTA, plus their SHA-256. |
| `firmware/tools/` | The two tools you actually run: the serial OTA updater and the serial diagnostic tool. |
| `firmware/docs/` | Vendor reference: the flashing guide, the complete serial protocol, and the feature list for this build. |
| `hardware/` | OSCORE_NEO RevA: schematic, PCB reference-designator map, board photos, 3D STEP model, and the hardware manual. |
| `oled-matrix/` | The OLED matrix display protocol and its host-side driver. |
| `troubleshooting/` | A blank record template for logging field issues. |

## The board in one paragraph

OSCORE_NEO is an ESP32-S3 board that takes 9 to 26 V in, distributes 5 V and 3.3 V, and carries the IMU (QMI8658A + QMC6309 magnetometer, with a small heater to hold the IMU at a stable temperature), a CH339F USB hub, 100M Ethernet, CAN, and the SBUS / PWM / encoder lines for the drivetrain. Throttle PWM is on GPIO1, steering PWM on GPIO2, the SBUS receiver on UART0 (GPIO44 RX), and the wheel encoder on GPIO3/9. Full pinout is in `firmware/source/main/config.h` and the hardware manual.

## Flashing

Two modes. Read `firmware/docs/Firmware_Flashing_Guide.md` for the full procedure. The short version:

**App update (normal case).** The car already runs `osrcore` and you just want a newer app. No BOOT button, keeps your calibration and PID values in NVS:

```bash
python3 firmware/tools/NEORACER_V1_Serial_OTA_Update_Tool.py \
  --port /dev/ttyACM0 \
  firmware/binaries/NEORACER_V1_OSCORE_NEO_ESP32S3_IDF_Project_RevA_20260628.bin
```

**Factory restore (first flash or a bricked board).** Rewrites the whole flash layout. Put the ESP32-S3 in ROM BOOT download mode first:

```bash
python -m esptool --chip esp32s3 -p /dev/ttyACM0 -b 460800 \
  --before default-reset --after hard-reset \
  write-flash --flash-mode dio --flash-size 16MB --flash-freq 80m \
  0x0 firmware/binaries/NEORACER_V1_OSCORE_NEO_ESP32S3_IDF_FullFlash_RevA_20260628.bin
```

After either, verify with the diagnostic tool:

```bash
python3 firmware/tools/NEORACER_V1_Serial_Diagnostic_Tool.py --port /dev/ttyACM0 \
  "fw version" "status" "pid get" "odom get"
```

Serial baud for all host tools is `460800`. On the Jetson the port is usually `/dev/ttyACM0`.

## Tuning: where every knob lives

There are three tiers, and knowing which tier a value is on tells you how to change it and how risky it is.

1. **Runtime, over serial, saved to flash.** Send a command, it applies live and persists across reboots. No reflash. This is where you should do almost all tuning.
2. **Build-time, in `config.h` / `menuconfig`.** Baked into the image. Changing these means rebuilding and reflashing.
3. **Physical ceiling.** Motor, ESC, gearing, battery. No amount of software raises these.

### Speed

The firmware speed limit is `MAX_SPEED_FORWARD = 6.0 m/s` (and `-6.0` reverse), set in `config.h`. Both serial `v` commands and RC throttle are mapped into that range, so **6 m/s is the ceiling this firmware will command**. To go faster you rebuild with a higher `MAX_SPEED_FORWARD` and reflash, and only up to what the drivetrain can physically deliver.

Before you raise it, understand the feed-forward. Throttle output is `neutral + THROTTLE_FF_GAIN_US_PER_MPS * target + PID correction`. At `68 us/m/s`, a 6 m/s target already asks for about 408 us of the roughly 500 us of usable ESC travel, so the linear headroom runs out not far above 6 m/s. Past that the ESC saturates and the closed loop can no longer regulate. Measure actual wheel speed against commanded speed (drive it, watch the `o` odometry frame) before assuming a higher number is real.

Two speed limits are **not** in this firmware and are worth knowing:

- **RC low-speed mode.** SBUS **CH7** low = throttle scaled to `RC_LOW_SPEED_PERCENT = 15%`, high = full speed. If manual driving feels slow, this switch is the first thing to check, not the firmware.
- **Host-side caps.** If you drive over ROS, the student library and the ROS driver apply their own multipliers on top. Those live in the Jetson software, not here.

### Speed PID

Defaults (`config.h`): `Kp = 447.0`, `Ki = 4.7`, `Kd = 4.7`. Change them live:

```
pid set 447 4.7 4.7      # apply and queue an NVS save
pid get                  # read current gains
pid reset                # back to firmware defaults
```

Tune at low speed first. Raise `Kp` until the car holds a commanded speed crisply without buzzing, back off if it oscillates. `Ki` removes steady-state error on slopes and against rolling resistance; keep it small. `Kd` damps overshoot. The controller runs every `CONTROL_INTERVAL = 20 ms` on an encoder-speed estimate low-passed with `SPEED_LPF_ALPHA = 0.15`. After tuning, wait about a second before power-cycling so the background NVS save finishes.

### Steering

Steering is a servo on a `1000..2000 us` pulse, `STEERING_CENTER = 1500`, mechanical limit `STEERING_MAX_ANGLE_DEG = 30 deg`. The one field knob is the center trim, to make the car track straight:

```
trim set -2.5     # degrees, clamped to -5.0..+5.0, saved to NVS
trim get
trim reset
```

Positive trim steers left. Trim is stored as an angle so it stays consistent with the vehicle geometry across recalibration.

### Odometry scale

If the car's reported distance is off, correct it with a straight-line test over 1 or 2 m:

```
odom scale set 1.0     # clamped 0.50..1.50, saved to NVS
odom scale get
odom scale reset
```

Calibration order that works: fix `trim` first so it drives straight, then tune `odom scale` on the straight-line run.

### Throttle deadband

Small motors need a minimum pulse past neutral before the wheels move at all. That floor is the feed-forward deadband:

```
speed deadband set 90     # microseconds, clamped 0..300, saved to NVS
speed deadband get
speed deadband reset
```

Default is `90 us`. Raise it if the car stalls at very low commanded speeds, lower it if it creeps when told to stop.

### Build-time chassis constants

These describe the physical car and only change if you change hardware. They live in `menuconfig` (or `sdkconfig.defaults`) under "OSCore chassis", read into `config.h`:

| Setting | Default | Meaning |
|---------|--------:|---------|
| `OSCORE_ENCODER_PPR` | 1024 | Encoder pulses per motor revolution |
| `OSCORE_GEAR_RATIO_MILLI` | 10550 | Motor-to-wheel gear ratio x1000 (10.55:1) |
| `OSCORE_WHEEL_RADIUS_UM` | 42500 | Wheel radius in micrometers (85 mm diameter) |
| `OSCORE_SPEED_CALC_INTERVAL_MS` | 20 | Encoder speed sampling interval |
| `OSCORE_ENCODER_A_GPIO` / `B` | 3 / 9 | Encoder input pins |

If your gearing, wheel size, or encoder change and you skip these, reported speed and odometry will be wrong by the corresponding ratio.

## Full runtime parameter reference

Everything settable over serial that persists to flash. Full command set is in `firmware/docs/Serial_Protocol.md`.

| Parameter | Command | Default | Range | Reflash needed |
|-----------|---------|--------:|-------|:--:|
| PID gains | `pid set <kp> <ki> <kd>` | 447 / 4.7 / 4.7 | free | no |
| Steering trim | `trim set <deg>` | 0.0 | -5.0 .. +5.0 deg | no |
| Odometry scale | `odom scale set <v>` | 1.0 | 0.50 .. 1.50 | no |
| Throttle deadband | `speed deadband set <us>` | 90 | 0 .. 300 us | no |
| Magnetometer cal | `mc cal [sec]` / `mc set` | identity | 12 floats | no |
| Level cal | `level cal` | auto | accel offset | no |
| Forward/reverse speed cap | `MAX_SPEED_FORWARD` in `config.h` | 6.0 m/s | build-time | **yes** |
| Feed-forward slope | `THROTTLE_FF_GAIN_US_PER_MPS` | 68 us/m/s | build-time | **yes** |
| RC low-speed percent | `RC_LOW_SPEED_PERCENT` | 15% | build-time | **yes** |
| Control loop period | `CONTROL_INTERVAL` | 20 ms | build-time | **yes** |

## Serial commands you will actually use

```
v <mps> <deg>     drive: set speed and steering (serial mode, CH6 >= 1500)
status            speed, voltage, control source, IMU, heater, safety, calibration
diag              one-line machine-readable status + diagnostic code
s                 one synchronized odom + IMU snapshot
b                 battery voltage
pid get           read PID gains
odom get          read odometry state
odom reset        zero the pose and yaw
fw version        firmware build info
help              full command list
```

Control arbitration: SBUS **CH6** decides who drives. Below 1500, the RC transmitter has priority and serial `v` is ignored. At or above 1500, serial control is active. Serial `v` must be refreshed within 500 ms or the car stops on its own.

## Building from source

Standard ESP-IDF (target `esp32s3`):

```bash
cd firmware/source
idf.py set-target esp32s3
idf.py menuconfig      # optional: OSCore chassis + USB settings
idf.py build
idf.py -p /dev/ttyACM0 flash
```

The image self-versions from git via `version.h.in`. Board-specific chassis and USB options are under the "OSCore chassis" and "OSCore USB" menus.

## Safety notes

- The firmware forces the throttle to neutral whenever safety says so: low battery latched, OTA in progress, RC failsafe, or a lost SBUS frame. `status` and `diag` report the reason.
- Keep battery above 10.8 V for bench work. Below that the low-voltage alarm latches and motion is inhibited; the OTA tool has a `--force-low-voltage` override for bench tests only.
- Always bench-test a new speed cap or PID change with the wheels off the ground first, then on the floor at low speed. Higher speed makes every tuning mistake bite harder.

## Provenance and support

`osrcore` firmware and the OSCORE_NEO board were developed for NeoRacer V1 by Seeed Studio and OSRBOT. This repository is Neobotics Foundation's published copy of the RevA delivery (firmware dated 2026-06-28, board RevA) for the NeoRacer community. Vendor technical support: winter@osrbot.com. For NeoRacer questions: support@neobotics.org.
