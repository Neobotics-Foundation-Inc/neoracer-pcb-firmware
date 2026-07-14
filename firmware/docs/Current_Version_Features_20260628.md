# NEORACER V1 OSCORE NEO Firmware Current Version Features

Version: 20260628  
Hardware: OSCORE NEO ESP32S3 RevA  
Technical support: winter@osrbot.com

## Version Notes

This firmware package is prepared for the current NEORACER V1 ODM/OEM batch delivery and has been validated on the vehicle. The main updates are centralized RC low-speed configuration and speed-proportional throttle feed-forward.

## Current Features

- Default serial baud rate is 460800.
- TinyUSB CDC descriptor strings: Product `Neoracer V1`, Manufacturer `Seeed Studio & OSRBOT`, Serial `OSRCOREV01`.
- Supports `fw version` to report product, hardware, version date, build identifier, and build time.
- Adds an `FW:` line to `status` for direct firmware identification.
- Supports `stream sync|legacy|off`; sync stream is the default host-facing mode.
- Supports the synchronized frame `s px py pz vx vy vz yaw qx qy qz qw ax ay az gx gy gz`.
- Supports legacy realtime frames `i/o/m/r/b`.
- Supports `sn get` for the ESP32-S3 hardware serial number.
- Supports serial OTA app update with `fw begin/data/end/abort/status`.
- Supports `diag` summary output for troubleshooting low voltage, RC failsafe, IMU/level state, host link state, and other primary status items.
- Supports ROS host link tones for host connect and disconnect events.
- Factory PID defaults remain `Kp=447.0, Ki=4.7, Kd=4.7`.
- RC low-speed mode is configured through `RC_LOW_SPEED_PERCENT` / `RC_LOW_SPEED_SCALE`; this version keeps 15%.
- Throttle feed-forward is expressed as `THROTTLE_FF_GAIN_US_PER_MPS`, meaning PWM microseconds per `1 m/s`; `MAX_SPEED_FORWARD/REVERSE` only limit commanded speed and no longer affect low-speed feed-forward scaling.

## Firmware Files

- Full flash: `../Firmware_Files/NEORACER_V1_OSCORE_NEO_ESP32S3_IDF_FullFlash_RevA_20260628.bin`
- App only: `../Firmware_Files/NEORACER_V1_OSCORE_NEO_ESP32S3_IDF_Project_RevA_20260628.bin`
- SHA256: `../Firmware_Files/NEORACER_V1_OSCORE_NEO_ESP32S3_SHA256_Checksum_RevA_20260628.txt`
