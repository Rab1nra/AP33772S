# AP33772S Arduino Library By Rabindra 

Full-featured Arduino library for the **Diodes Inc. AP33772S** USB PD 3.1 EPR
Sink Controller — the chip used on the [CentyLab PicoPD Pro](https://centylab.com)
and compatible boards.

[![Arduino Library](https://img.shields.io/badge/Arduino-Library-blue)](https://www.arduino.cc/reference/en/libraries/)
[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)
[![Platform](https://img.shields.io/badge/platform-RP2040%20%7C%20ESP32-orange)](https://github.com/YOURUSERNAME/AP33772S)

---

## Features

- ✅ PDO discovery — Fixed, PPS (SPR), AVS (EPR) — all 13 slots
- ✅ Auto voltage selection with `setVoltage()` — picks best PDO automatically
- ✅ Manual `setFixPDO()`, `setPPSPDO()`, `setAVSPDO()` for full control
- ✅ PPS/AVS keep-alive via `task()` — maintains variable voltage reliably
- ✅ Live ADC readings — voltage (80mV LSB), current (24mA LSB), temperature
- ✅ Full protection config — OVP, UVP, OCP, OTP, thermal de-rating
- ✅ Interrupt-driven status monitoring
- ✅ NTC thermistor calibration (4-point table)
- ✅ Hard reset, output switch, cable flip detection
- ✅ Compatible with RP2040 (Arduino-Pico) and ESP32 (Arduino)
- ✅ Bit-field decoding verified against official CentyLab PicoPD library

---

## Supported Hardware

| Board | SDA | SCL | INT |
|---|---|---|---|
| PicoPD Pro (RP2040) | GP20 | GP21 | GP6 |
| ESP32 DevKit | GPIO21 | GPIO22 | GPIO32 |
| Any RP2040 board | configurable | configurable | configurable |

---

## Installation

### Arduino IDE (ZIP)
1. Download this repo as a ZIP: **Code → Download ZIP**
2. Arduino IDE → **Sketch → Include Library → Add .ZIP Library**
3. Select the downloaded ZIP

### Arduino IDE (Git)
Clone directly into your Arduino libraries folder:
```bash
cd ~/Arduino/libraries
git clone https://github.com/YOURUSERNAME/AP33772S.git
```

### PlatformIO
Add to your `platformio.ini`:
```ini
lib_deps =
    https://github.com/YOURUSERNAME/AP33772S.git
```

---

## Quick Start
```cpp
#include <Wire.h>
#include "AP33772S.h"

AP33772S pd(Wire, 6);   // Wire, INT pin

void setup() {
    Serial.begin(115200);
    Wire.begin(20, 21);          // SDA, SCL for PicoPD Pro
    Wire.setClock(400000);

    pd.begin();
    pd.waitForStartup(5000);
    pd.waitForPDOs(8000);
    pd.readAllPDOs();
    pd.printPDOs();              // Print charger capabilities to Serial

    pd.setVoltage(12000, 2000);  // Request 12V, min 2A
    pd.waitForNegotiation(3000);
}

void loop() {
    pd.task();  // REQUIRED for PPS/AVS keep-alive

    Serial.printf("V=%umV  I=%umA  P=%umW  T=%d°C\n",
        pd.getVoltage_mV(), pd.getCurrent_mA(),
        pd.getPower_mW(),   pd.getTemperature_C());
    delay(1000);
}
```

> ⚠️ **Always call `pd.task()` in `loop()`** when using PPS or AVS mode.
> Without it, the charger reverts to 5V after ~5 seconds.

---

## API Reference

### Initialisation

| Function | Description |
|---|---|
| `begin(enableEPR, enablePPS)` | Init I2C, configure PDCONFIG and protections |
| `waitForStartup(timeoutMs)` | Wait for cable attach (STATUS.STARTED) |
| `waitForPDOs(timeoutMs)` | Wait for PDOs ready (STATUS.NEWPDO + READY) |
| `task()` | Keep-alive for PPS/AVS — call every loop() |

### PDO Discovery

| Function | Description |
|---|---|
| `readAllPDOs()` | Burst-read all 13 PDOs, returns valid count |
| `readPDO(index, pdo)` | Read single PDO by index (1–13) |
| `getPDO(index)` | Get cached PDO struct |
| `printPDOs(stream)` | Print table to Serial |
| `getPPSIndex()` | First PPS slot index, -1 if none |
| `getAVSIndex()` | First AVS slot index, -1 if none |

### Voltage Requests

| Function | Description |
|---|---|
| `setVoltage(mV, minMA)` | Auto-select best PDO and request voltage |
| `setFixPDO(index, mA)` | Request Fixed PDO at max voltage |
| `setPPSPDO(index, mV, mA)` | Request PPS at specific voltage (100mV steps) |
| `setAVSPDO(index, mV, mA)` | Request AVS at specific voltage (200mV steps) |
| `waitForNegotiation(ms)` | Wait for PD_MSGRLT.SUCCESS |
| `issueHardReset()` | Send PD hard reset |

### Measurements

| Function | Returns |
|---|---|
| `getVoltage_mV()` | VOUT in mV (LSB = 80mV) |
| `getCurrent_mA()` | IOUT in mA (LSB = 24mA) |
| `getPower_mW()` | V × I in mW |
| `getTemperature_C()` | NTC temperature in °C |
| `getRequestedVoltage_mV()` | Agreed voltage from last negotiation |
| `getRequestedCurrent_mA()` | Agreed current from last negotiation |

### Protection

| Function | Description |
|---|---|
| `setOVPOffset_mV(mV)` | OVP = VREQ + offset (LSB 80mV) |
| `setUVPThreshold(mode)` | UVP_80PCT / UVP_75PCT / UVP_70PCT |
| `setOCPThreshold_mA(mA)` | 0 = auto (110% of PDO max) |
| `setOTPThreshold_C(°C)` | NTC over-temperature trip |
| `setDeratingThreshold_C(°C)` | Thermal de-rating start |
| `setProtectionConfig(...)` | Enable/disable each protection individually |

### Status

| Function | Returns |
|---|---|
| `isPDConnected()` | true if USB-PD source detected |
| `isLegacyConnected()` | true if non-PD charger (5V only) |
| `isCableFlipped()` | true if CC2 active (cable flipped) |
| `isDerating()` | true if thermal de-rating active |
| `isFault()` | true if any protection has tripped |
| `getFaultString()` | "FAULT: OVP UVP ..." string |

---

## PDO Bit-Field Decoding

The AP33772S uses a 16-bit compressed PDO format (little-endian):
```
Bit 15     : detect     — 1 = slot populated
Bit 14     : type       — 0 = Fixed,  1 = PPS (SPR) or AVS (EPR)
Bits 13:10 : current_max — 4-bit range code (see currentMap)
Bits  9:8  : voltage_min — 2-bit indicator (PPS/AVS only)
Bits  7:0  : voltage_max — 8-bit
               SPR: × 100 mV/unit
               EPR: × 200 mV/unit
```

Key correction vs naive decoding: `voltage_max` is in **bits[7:0]**, not
bits[13:8]. This was verified against the official CentyLab PicoPD library
source code.

---

## Examples

| Sketch | Description |
|---|---|
| `01_ScanPDOs` | Connect and print all PDOs from any charger |
| `02_RequestVoltage` | Auto-request a target voltage |
| `03_PPSVariable` | Sweep voltage up and down using PPS/AVS |
| `04_ProtectionMonitor` | Interrupt-driven fault detection and recovery |
| `05_FullSystem` | State machine with Serial command interface |

---

## Acknowledgements

- Bit-field definitions and I2C protocol based on the official
  [CentyLab PicoPD](https://github.com/CentyLab/PicoPD) library
  by VicentN / CentyLab LLC
- Datasheet: AP33772S DS46176 Rev. 9-2 (Diodes Incorporated, Feb 2026)

---

## License

MIT — see [LICENSE](LICENSE)
