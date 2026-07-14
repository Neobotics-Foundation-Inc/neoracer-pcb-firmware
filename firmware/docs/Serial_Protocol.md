# OSRacer Firmware Serial Protocol

Technical support: winter@osrbot.com

This document describes the two serial links used by `osrcore`.

## 1. USB CDC Serial

Purpose: bidirectional communication with the host computer or Orin Nano.

Transport:

- Port: TinyUSB CDC by default
- Baud used by host tools: `460800`
- USB descriptors: product `Neoracer V1`, manufacturer `Seeed Studio & OSRBOT`, serial `OSRCOREV01`
- Payload: plain ASCII text
- Framing: newline-delimited lines
- Checksum: none
- Message type: first character or command token at the beginning of the line

Periodic telemetry is enabled by default at boot and can be controlled with `stream sync|legacy|off`. The firmware receives commands on the same link. The serial TX path is decoupled through realtime overwrite slots and a FreeRTOS queue:

- Realtime overwrite slots: `i/o/s/m`. Only the latest frame is kept; stale frames are dropped to avoid serial backlog and delayed pose data.
- Normal command-response queue depth: `128`
- Maximum line length: `192` bytes
- Dedicated `serial_tx` task sends realtime slots first, then drains the normal response queue and writes to `stdout`
- Dropped TX lines are counted and reported by `status` as `SerialTxDrops`

### Device To Host Telemetry

| Type | Default Interval | Format |
|---|---:|---|
| `s` Sync snapshot | 5 ms, default stream | `s px py pz vx vy vz yaw qx qy qz qw ax ay az gx gy gz` |
| `i` IMU | 5 ms, legacy stream | `i qx qy qz qw ax ay az gx gy gz` |
| `m` Magnetometer | 50 ms | `m mx my mz` |
| `o` Odometry | 20 ms, legacy stream | `o px py pz vx vy vz yaw` |
| `b` Battery | 2 s | `b voltage` |
| `r` RC channels | 100 ms | `r ch0 ch1 ch2 ch3 ch4 ch5 ch6 ch7 ch8 ch9` |

Default `stream sync` sends `s/m/r/b`. `stream legacy` sends `i/o/m/r/b` for older host clients.

Example:

```text
i 0.0010 -0.0030 0.7070 0.7070 0.120000 -0.050000 9.810000 0.001000 -0.002000 0.003000
m 0.2340 -0.1120 0.4500
o 1.234000 0.056000 0.000000 0.500000 0.010000 0.000000 0.314000
s 1.2340 0.0560 0.0000 0.5000 0.0000 0.0000 0.3140 0.0010 -0.0030 0.7070 0.7070 0.1200 -0.0500 9.8100 0.0010 -0.0020 0.0030
b 12.10
r 1500 1500 1000 1500 1500 1500 2000 1000 1500 1500
```

Units:

- Quaternion: `qx qy qz qw`, Madgwick full attitude with the current odometry yaw-zero applied. Roll/pitch are preserved for slopes and 3D lidar use; yaw is relative to the latest odometry zero.
- Acceleration: `m/s^2`
- Gyro: `rad/s`
- Magnetometer: Gauss, calibrated
- Battery voltage: V
- Odometry position: meters
- Odometry velocity: `m/s`
- Odometry yaw: radians, Madgwick yaw after subtracting the current odometry yaw-zero; magnetometer is diagnostic only by default
- RC channel values: raw SBUS-style channel values, or `-1` for each channel during failsafe

### Host To Device Commands

| Command | Description |
|---|---|
| `v <vx_m/s> <steer_deg>` | Set speed and steering. Example: `v 1.5 10.0`. Serial control times out after 500 ms without a new command and stops the vehicle. |
| `i` | Query one IMU frame |
| `b` | Query one battery-voltage frame |
| `o` | Query one odometry frame |
| `m` | Query one magnetometer frame |
| `s` | Query one synchronized snapshot frame |
| `sn get` | Query ESP32-S3 hardware serial number as a 12-character uppercase hex string |
| `stream sync` | Enable default periodic `s/m/r/b` telemetry |
| `stream off` | Disable periodic telemetry |
| `stream legacy` | Enable compatibility periodic `i/o/m/r/b` telemetry |
| `status` | Query speed, voltage, control source, IMU, heater, diagnostics, and chassis calibration |
| `mc cal [sec]` | Run magnetometer calibration, default 30 s |
| `mc set <12f>` | Set hard-iron and soft-iron calibration from 12 floats and queue an NVS save |
| `mc get` | Query current magnetometer calibration |
| `mc reset` | Reset magnetometer calibration to identity |
| `level cal` | Trigger accelerometer level calibration |
| `level get` | Query level offset |
| `level reset` | Reset level calibration and re-arm auto-calibration |
| `odom get` | Query odometry state |
| `odom reset` | Reset odometry pose and yaw zero; the same yaw-zero is applied to published yaw and quaternions |
| `odom scale get` | Query odometry scale |
| `odom scale set <value>` | Set odometry scale, clamped to `0.50..1.50`, and queue an NVS save |
| `odom scale reset` | Reset odometry scale to `1.0` |
| `trim get` | Query steering center trim |
| `trim set <deg>` | Set steering trim, clamped to `-5.0..5.0 deg`, and queue an NVS save |
| `trim reset` | Reset steering trim to `0 deg` |
| `speed deadband get` | Query throttle feed-forward deadband |
| `speed deadband set <us>` | Set throttle feed-forward deadband, clamped to `0..300 us`, and queue an NVS save |
| `speed deadband reset` | Reset throttle feed-forward deadband to default |
| `pid get` | Query PID gains |
| `pid set <kp> <ki> <kd>` | Set PID gains and queue an NVS save |
| `pid reset` | Reset PID gains to firmware defaults |
| `fw begin <size> <sha256>` | Begin serial OTA app update |
| `fw data <seq> <hex>` | Send one OTA data chunk |
| `fw end` | Verify, switch OTA app slot, and reboot |
| `fw abort` | Abort OTA update |
| `fw status` | Query OTA state |
| `fw version` | Query firmware version, build identifier and build time, and IDF version |
| `diag` | Internal support query for the compact diagnostic code line |
| `reset` | Reboot controller |
| `help` | Print command list |

Commands that persist settings to NVS apply the runtime value first and then enqueue a background NVS save. `OK ... save queued` means the save job was queued; `ERROR ... NVS save queue full` means the value remains active in RAM, but persistence was not queued. After field tuning, wait about 1 second before power-cycling so the background save can finish.

`status` includes a firmware-version line and a machine-readable safety summary:

```text
FW: Product=Neoracer V1, Firmware=osrcore, Hardware=OSCORE_NEO_ESP32S3_RevA, Version=<YYYYMMDD>, Dirty=No, Build=<time>, ProjectVer=NEORACER_V1-<YYYYMMDD>-g<sha>
SAFETY: Level=WARN, AllowMotion=Yes, Top=LEVEL_NOT_CALIBRATED, Mask=0x00000100
```

Current safety levels are `OK`, `INFO`, `WARN`, `ERROR`, and `FATAL`. `AllowMotion=No` means firmware forces throttle neutral until the blocking condition clears.

### Internal Diagnostic Commands

These commands are intended for OSRBOT tools and support workflows. They are not part of the realtime telemetry stream and must not be required by the ROS data parser.

`diag` returns one compact, machine-readable status line:

```text
DIAG: Level=1, Code=110, Name=HOST_LOST, Motion=Yes, Control=Serial, V=11.8, Uptime=123
```

Fields:

- `Level`: `0=OK`, `1=WARN`, `2=ERROR`, `3=SERVICE`
- `Code`: stable numeric status code for host UI logic
- `Name`: readable status name for logs/support
- `Motion`: whether firmware currently allows motion output
- `Control`: `Serial` or `Remote`
- `V`: battery voltage
- `Uptime`: firmware uptime in seconds

Current diagnostic codes:

| Code | Name | Level | Meaning |
|---:|---|---:|---|
| `000` | `READY` | 0 | System ready |
| `010` | `INIT` | 1 | IMU/heater initialization is still in progress |
| `110` | `HOST_LOST` | 1 | A host link was seen, then no heartbeat was received for 5 s |
| `210` | `LOW_BAT_ERROR` | 2 | Low-voltage alarm is latched; motion is inhibited |
| `310` | `RC_FAILSAFE` | 1 | SBUS receiver reports failsafe/lost frame |
| `400` | `IMU_NOT_READY` | 1 | IMU readiness or gyro bias is not ready |
| `420` | `LEVEL_NOT_CALIBRATED` | 1 | Accelerometer level calibration is not ready |
| `700` | `OTA_ACTIVE` | 3 | Serial OTA update is active; motion is inhibited |

Reserved code `510 I2C_SENSOR_ERROR` is kept for future board-level sensor fault reporting and is not emitted by this firmware yet.

Host-link hint commands:

```text
link up ros
link ping ros
link down ros
```

The ROS host should send `link up ros` after the serial port is opened and `stream sync` is configured, then send `link ping ros` about once per second. `link ping ros` is intentionally silent to avoid adding response lines to the realtime stream. On normal shutdown the host may send `link down ros`. These commands only drive the support-facing LED/buzzer host-link indicator and `diag` state; they do not affect motion control or telemetry framing.

### Control Arbitration

USB-JTAG serial and SBUS share the same control state in `task_comm`.

- When SBUS CH6 is `< 1500`, remote control has priority and serial `v` commands are ignored.
- When SBUS CH6 is `>= 1500`, serial control is active.
- Serial `v` commands must be refreshed within `500 ms`; otherwise target speed becomes `0` and steering returns to the trimmed center.
- During firmware OTA, throttle is forced to neutral and steering returns to the trimmed center.

## 2. SBUS Receiver Link

Purpose: one-way remote-control receiver input and safety/control-mode switching.

Transport:

- Protocol: Futaba SBUS
- Baud: `100000`
- UART mode: `8E2`
- Signal: inverted
- ESP32-S3 UART: UART0
- RX pin: GPIO44
- TX pin: GPIO43, configured but not required for normal receive-only operation

Frame format:

```text
[0x0F] [22 bytes channel data: 16 channels x 11-bit packed] [flags] [0x00]
```

Example frame with centered channels:

```text
0F 00 04 10 40 00 01 04 10 40 00 01 04 10 40 00 01 04 10 40 00 01 04 00 00
```

The parser keeps byte-level frame sync across calls and validates the footer before decoding.

### Channel Mapping

| Channel | Use | Mapping |
|---|---|---|
| CH0 | Steering | SBUS `240..1810` to servo `1000..2000 us`, plus `steering_trim_deg` converted to pulse offset |
| CH2 | Throttle | SBUS `240..1810` to `-6.0..+6.0 m/s` |
| CH6 | Control mode | `<1500` = remote priority, `>=1500` = serial control |
| CH7 | Speed mode | `<1500` = 15% speed limit, `>=1500` = full speed |

Failsafe behavior:

- `failsafe` or `lost_frame` causes target speed to become `0`
- Steering returns to the trimmed center
- Periodic `r` telemetry reports all channels as `-1`

## Notes

- In the current indoor navigation firmware configuration, magnetometer data is reported for diagnostics only and is not fused into odometry heading.
- The host-side `/odometry/filtered` output should primarily fuse encoder odometry and IMU data. During navigation, AMCL or an equivalent localization module maintains the `map -> odom` transform.
- For chassis calibration, adjust `trim` first to correct steering center, then tune `odom scale` with a 1 m or 2 m straight-line distance test. The `trim` parameter is stored as a steering angle so it remains consistent with vehicle geometry and calibration records.
