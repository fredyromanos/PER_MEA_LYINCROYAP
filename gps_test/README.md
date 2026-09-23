# GPS Standalone Test

Minimal Arduino sketch to test u-blox NEO-6M GPS on LilyGO T-Beam V1.1 independently.

## Hardware Connections

| Signal | T-Beam GPIO |
|--------|-------------|
| GPS RX (GPS→ESP32) | GPIO34 (Serial1 RX, input-only) |
| GPS TX (ESP32→GPS) | GPIO12 (Serial1 TX) |
| GPS Power | AXP192 LDO3 (3.3V) — **must enable in code** |

## Build & Flash

```bash
cd /home/ArchH/Downloads/Project_Ecoresponsable/2026/S9P-2026/03_Developpement_logiciel/PER_MEA_LYINCROYAP/gps_test

# Compile
~/bin/arduino-cli compile --fqbn esp32:esp32:t-beam .

# Flash (adjust port)
~/bin/arduino-cli upload --port /dev/ttyUSB0 --fqbn esp32:esp32:t-beam .
```

## Serial Monitor (115200 baud)

```bash
# Linux/macOS - avoid DTR/RTS reset
python3 -c "
import serial, sys
s = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.5, dsrdtr=False, rtscts=False)
s.dtr = False; s.rts = False
while True:
    line = s.readline()
    if line: print(line.decode('utf-8','replace').rstrip())
"
```

## Expected Output

```
[GPS] AXP192 LDO3 enabled (GPS power ON)
[GPS] Serial1 started 9600 baud RX=GPIO34 TX=GPIO12
[GPS] CFG-CFG sent — factory reset to ROM defaults
[GPS] CFG-ANT sent — active antenna supervisor + bias enabled
[GPS] Waiting for NMEA...
[GPS] NO FIX  visible=0  chars=1245  badCRC=0
        last: $GPGSV,3,1,09,02,45,185,40,...
...
[GPS] FIX ACQUIRED  lat=48.360476 lon=-4.566822 sats=7 hdop=2.2
[GPS] lat=48.360476 lon=-4.566822 spd=0.0km/h hdg=0.0° sat=7 hdop=2.2 age=120ms
[GPS] lat=48.360476 lon=-4.566822 spd=0.0km/h hdg=0.0° sat=7 hdop=2.2 age=200ms
...
```

## Troubleshooting

| Symptom | Cause | Fix |
|---------|-------|-----|
| `chars=10` frozen, no NMEA | Saved config disabled NMEA | CFG-CFG reset (in code) |
| `visible=9` but `NO FIX` | Antenna supervisor off | CFG-ANT `flags=0x001B` (in code) |
| 0 satellites outdoors | No LiPo → cold start every boot | **Connect LiPo battery** |
| `hdop=99.9` | No valid HDOP | Wait for more satellites |
| `age=4294967295` | No fix yet | Wait for fix |

## Key Requirements

1. **LiPo battery connected** — without it, cold start adds ~2 min to fix
2. **External active antenna** on u.FL (Taoglas ADFGP.25A or similar)
3. **Clear sky view** — indoors = no fix
4. **AXP192 LDO3 enabled** — code does this automatically

## Files

```
gps_test/
├── gps_test.ino          # Main sketch
├── GpsUart.h             # Copied from firmware (no dependencies)
├── GpsUart.cpp           # Copied from firmware
├── BoardConfig.h         # Minimal pin definitions
├── DebugConfig.h         # Debug macros
└── Types.h               # GpsPosition struct
```