# NEORACER V1 Troubleshooting Record Template

Technical support: winter@osrbot.com

## 1. Basic Information

Customer / project:  
Device serial number:  
Issue date:  
Contact person:  
Firmware version:  
Host software version:  

## 2. Firmware and Serial Check

Reference firmware file:

- `../01_Firmware/Firmware_Package_20260628/Firmware_Files/NEORACER_V1_OSCORE_NEO_ESP32S3_IDF_FullFlash_RevA_20260628.bin`

Suggested command:

```bash
python3 ../01_Firmware/Firmware_Package_20260628/Flashing_and_Diagnostic_Tools/NEORACER_V1_Serial_Diagnostic_Tool.py \
  --port <PORT> \
  "stream off" "fw version" "status" "diag" "pid get"
```

Record:

- Serial port:
- `fw version` output:
- `status` output:
- `diag` output:
- `pid get` output:
- Reproduction result:

## 3. Host Software Check

| Item | Result | Notes |
| --- | --- | --- |
| ROS 2 environment starts correctly | Pass / Fail |  |
| Host software detects chassis serial port | Pass / Fail |  |
| Demo launch flow runs correctly | Pass / Fail |  |
| Stop script clears remaining processes | Pass / Fail |  |
| Logs are collected for feedback | Pass / Fail |  |

## 4. Hardware Check

| Item | Result | Notes |
| --- | --- | --- |
| Connectors are secure | Pass / Fail |  |
| Cable orientation is correct | Pass / Fail |  |
| Control board has no visible damage | Pass / Fail |  |
| OLED matrix display connection is normal | Pass / Fail |  |
| Sensor connections are normal | Pass / Fail |  |

## 5. Issue Record

Symptom:

Reproduction steps:

Actions already tried:

Attachments:

- Photos:
- Videos:
- Serial logs:
- Host software logs:

## 6. Conclusion

Initial assessment:

Recommended next step:
