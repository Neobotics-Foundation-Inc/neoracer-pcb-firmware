# NEORACER V1 OSCORE NEO Hardware Manual

Revision: RevA  
Date: 2026-06-29  
Applies to: NEORACER V1 / OSCORE NEO ESP32S3 control board  
Purpose: customer use, interface identification, maintenance, and troubleshooting  
Technical support: winter@osrbot.com

## 1. Scope

This manual and the files in this folder help identify interfaces, check wiring, locate components, confirm mechanical clearance, and report hardware issues during field use and maintenance.

This folder includes:

- Schematic PDF
- PCB reference designator / drill reference PDF
- Front board view
- Back board view
- Interface description image
- PCB 3D STEP model

## 2. File List

| File | Use |
| --- | --- |
| `NEORACER_V1_OSCORE_NEO_Schematic_RevA_20260624.pdf` | Check electrical connections, signal names, and power paths |
| `NEORACER_V1_OSCORE_NEO_PCB_Reference_Designator_Map_RevA_20260624.pdf` | Check component reference designators and drill reference information for photos, mounting checks, and support feedback |
| `NEORACER_V1_OSCORE_NEO_Board_Front_View_RevA_20260604.png` | View front-side interfaces and component locations |
| `NEORACER_V1_OSCORE_NEO_Board_Back_View_RevA_20260604.png` | View back-side interfaces and component locations |
| `NEORACER_V1_OSCORE_NEO_Board_Interface_Description_RevA_20260604.png` | Check numbered interfaces and their functions |
| `NEORACER_V1_OSCORE_NEO_Board_3D_Model_STEP_RevA_20260624.step` | Check mechanical installation, clearance, and board shape |

## 3. Board Overview

The control board is an OSCORE ESP32S3 development board based on the ESP32-S3 WROOM-1U-N16R8 module. It integrates IMU / magnetometer, USB-C communication, USB Hub interfaces, Ethernet, power input/output, CAN / XT30 output, expansion IO, buttons, buzzer, WS2812 LED, and TF card slot.

For troubleshooting, start with:

1. `NEORACER_V1_OSCORE_NEO_Board_Interface_Description_RevA_20260604.png`
2. `NEORACER_V1_OSCORE_NEO_Board_Front_View_RevA_20260604.png`
3. `NEORACER_V1_OSCORE_NEO_Board_Back_View_RevA_20260604.png`
4. `NEORACER_V1_OSCORE_NEO_PCB_Reference_Designator_Map_RevA_20260624.pdf`

## 4. Interface Numbering

The following numbering follows the interface description image.

| No. | Interface / Part | Troubleshooting Use |
| --- | --- | --- |
| 1 | Power input, DC 9-26V input + power switch + reverse polarity protection | Check main power input, switch, and input polarity |
| 2 | QMI8658A 6-axis sensor + QMC6309 3-axis magnetometer | Check IMU / magnetometer issues |
| 3 | MX1.25-8P IO expansion interface | Check 8-pin IO expansion wiring |
| 4 | MX1.25-4P USB-HUB output interface *3 | Check USB Hub output ports |
| 5 | MX1.25-4P USB-HUB input interface *1 | Check USB Hub input port |
| 6 | Type-C communication interface | Check control board communication, flashing, and serial enumeration |
| 7 | SH1.0-8P 100Mbps Ethernet interface | Check 100M Ethernet interface |
| 8 | MX1.25-4P IO expansion interface | Check 4-pin IO expansion wiring |
| 9 | WS2812 LED | Check onboard RGB LED status |
| 10 | 2.54mm female header IO expansion interface | Check header expansion IO |
| 11 | Reset button | Reset the controller |
| 12 | BOOT button | Enter download / boot-related modes |
| 13 | XT30 2+2 power output, controllable power output + CAN bus | Check controllable power output and CAN wiring |
| 14 | SH1.0-2P power output with self-recovery fuse | Check controllable power output |
| 15 | Expansion switch interface | Check expansion switch wiring |
| 16 | Buzzer | Check buzzer indication |
| 17 | MX1.25-4P IO expansion interface | Check 4-pin IO expansion wiring |
| 18 | MX1.25-2P DC 5V output | Check 5V output |
| 19 | MX1.25-4P 100Mbps Ethernet interface | Check 100M Ethernet interface |
| 20 | SD card slot | Check TF / SD card slot |
| 21 | WS2812 expansion interface | Check external WS2812 expansion |

## 5. Common Troubleshooting Entry Points

### USB Communication or Flashing Fails

Check:

- Type-C communication interface, No. 6.
- Whether the USB cable supports data.
- Whether the OSCORE / USB CDC serial device appears on the host system.
- Whether firmware commands such as `fw version`, `status`, and `diag` work.

### Sensor Data Is Abnormal

Check:

- IMU / magnetometer area, No. 2.
- Board installation direction.
- Strong magnetic interference or loose mounting.
- Host logs and firmware `diag` output.

### Peripheral or USB Device Issue

Check:

- USB Hub output interfaces, No. 4.
- USB Hub input interface, No. 5.
- Cable orientation and connector seating.

### Power Issue

Check:

- DC 9-26V input, No. 1.
- Controllable power outputs, No. 13 and No. 14.
- 5V output, No. 18.
- Reverse connection, undervoltage, loose cable, or short circuit.

### CAN or External Power Output Issue

Check:

- XT30 2+2 power output, No. 13.
- CAN wiring and peripheral power state.
- Firmware `status` / `diag` output for low voltage, failsafe, or host link status.

### OLED Matrix Display or LED Status Issue

Check:

- Onboard WS2812 LED, No. 9.
- WS2812 expansion interface, No. 21.
- OLED matrix display protocol document and host driver `led_matrix.py`.

## 6. Recommended Customer Feedback

For efficient support, provide:

- Issue description and reproduction steps.
- Device serial number, firmware version, and host software version.
- `fw version`, `status`, and `diag` outputs.
- Clear front/back photos with the affected interface or component area marked.
- Reference designator if a component is involved.
- 3D STEP comparison screenshot or mechanical photos if installation clearance is involved.
- Startup logs or serial logs if communication or host software is involved.

## 7. Notes

- Check power state before connecting or disconnecting peripherals.
- Power input should remain within DC 9-26V.
- When troubleshooting power output, check the load for short circuits.
- BOOT / Reset buttons are used for startup, reset, or flashing workflows.
- This manual does not replace the schematic. Use the schematic PDF when signal names, power paths, or interface electrical connections must be confirmed.
