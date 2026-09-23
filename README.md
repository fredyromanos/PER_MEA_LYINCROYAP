# Dron à Voile — Firmware (Post-Claude)

Firmware for an autonomous wind-powered surface drone based on the LilyGO T-Beam V1.1 (ESP32).  
The drone collects scientific measurements at GPS waypoints using wind propulsion, with a propeller-based fallback for calm conditions.

---

## Physical Concept

The hull carries a **freely-rotating wing-profile sail** on a vertical axis. Because of its aerodynamic shape the sail always self-aligns so the wind arrives at roughly 45° to the drone's centerline — no active sail-angle control is needed for basic propulsion.

**Heading control** is achieved by two actuators working together:

| Actuator | How it controls the boat |
|---|---|
| **Aileron servo** (sail trailing edge) | Binary ±10° flap deflection creates a lateral force that pushes the hull to port or starboard, effectively choosing which tack the boat sails on |
| **Rotor/rudder winch** (stern) | Continuous multi-turn winch connected to a rotor that acts as a rudder — deviation from 1500 µs sets rotation speed and direction, steering the heading |

When wind is insufficient, **two ESCs with differential thrust** take over for direct propeller propulsion and steering.

Wind direction is never measured directly — it is **estimated from GPS track history** across two tack headings (Phase 5).

---

## Hardware

| Component | Details |
|---|---|
| **MCU board** | LilyGO T-Beam V1.1 — ESP32 WROVER + u-blox NEO-6M GPS + SX1276 LoRa 433 MHz + AXP192 PMIC |
| **Custom PCB** | PCB V2 (KiCad 9.0) — NPN 3.3→5V level shifters on PWM outputs, Pololu step-down 5V rail, screw terminals for ESCs/servos |
| **Sail servo** | Futaba S3003 — aileron at trailing edge of sail, binary ±10° deflection only (center = 1520 µs) |
| **Rotor servo** | Graupner Regatta ECO II (model 5176) — **winch servo**, 6 full rotations, controls yaw via rotor/rudder linkage |
| **ESCs** | Pro-Tronik Black Fet ×2 (BF30A or BF45A) — differential thrust steering, must be programmed with EPRG-3 card for correct LiPo cell count |
| **RC system** | Pro-Tronik PTR-6A transmitter + R8X 8ch receiver, 2.4 GHz FHSS |
| **Battery** | 3S–4S LiPo — required for GPS warm start (without it, cold start on every power cycle adds ~2 min to fix acquisition) |

### Pin mapping

| Signal | GPIO | Notes |
|---|---|---|
| RC CH2 (sail toggle) | 13 | Interrupt PWM input |
| RC CH3 (throttle) | 22 | Shared with AXP192 SCL — OK after `Wire.end()` |
| RC CH4 (rotor / differential) | 21 | Shared with AXP192 SDA — OK after `Wire.end()` |
| RC CH5 (mode selector) | 4 | 3-position switch |
| RC CH6 (reserved) | 23 | Conflicts with LoRa RST — LoRa driver skips hardware reset, no issue in practice |
| Sail servo PWM | 2 | MCPWM Timer 0A |
| Rotor winch PWM | 25 | MCPWM Timer 0B |
| ESC 1 PWM | 32 | MCPWM Timer 1A |
| ESC 2 PWM | 33 | MCPWM Timer 1B |
| Battery ADC | 15 | ADC2_CH3 — external divider R1=560 kΩ R2=120 kΩ |
| GPS RX (GPS→ESP32) | 34 | Serial1, input-only |
| GPS TX (ESP32→GPS) | 12 | Serial1 |
| LoRa SCK/MISO/MOSI/CS | 5/19/27/18 | SPI HSPI |
| LoRa DIO0 | 26 | RX/TX done interrupt (polling used — not strictly required) |

---

## Software Architecture

Single-core event loop (no RTOS), layered OOP. All source lives under `main/src/`.

```
main.ino
  └── DroneApp          (orchestrator — software scheduler)
        ├── DRIVERS
        │     ├── AxpPower        — AXP192 init: enables GPS/LoRa/3.3V rails, then Wire.end()
        │     ├── RcReceiver      — 5-channel interrupt PWM, ISR-safe portMUX, 100 ms timeout
        │     ├── McpwmActuators  — MCPWM output (sail, rotor, ESC1, ESC2), slew rate limiting
        │     ├── GpsUart         — Serial1, TinyGPSPlus NMEA parser, drained every loop()
        │     ├── LoRaRadio       — SX1276 SPI driver, 433 MHz, blocking TX, polling RX
        │     └── BatteryAdc      — analogReadMilliVolts(), 11 dB attenuation, divider ratio 5.67×
        ├── COMM
        │     └── LoRaComm        — JSON heartbeat TX 1 Hz + command dispatcher
        ├── CONTROL
        │     ├── ModeManager     — CH5 pulse → ControlMode enum
        │     └── ManualController— RC → ActuatorCommand: binary sail, winch pass-through,
        │                           ESC arming (2 s hold at ≤1050 µs), differential thrust
        └── NAVIGATION
              ├── Navigator       — haversine distance + bearing (namespace, no state)
              ├── MissionPlan     — data struct: Waypoint[16], count, MissionMode
              └── MissionManager  — state machine: Idle → Running → Returning → Complete
```

### Update timing

| Task | Period | Notes |
|---|---|---|
| GPS drain | Every loop | `gps_.update()` — must not be rate-limited |
| LoRa RX poll | Every loop | `lora_.update()` — non-blocking `parsePacket()` |
| Control loop | 20 ms | RC read → mode decode → actuator write |
| Battery sample | 250 ms | |
| LoRa heartbeat TX | 1000 ms | Blocks loop ~450 ms during TX at SF7/BW125 |
| Debug serial print | 200 ms | 5 Hz — readable on serial monitor |

---

## Control Modes (CH5)

| CH5 pulse | Mode | Behaviour |
|---|---|---|
| < 1000 µs | **Automatic** | GPS waypoint navigation — MissionManager active |
| 1400–1600 µs | **ManualServo** | CH2 → sail toggle (±10°), CH4 → rotor winch direct |
| > 1800 µs | **ManualProp** | CH3 → throttle, CH4 → differential ESC steering |
| 0 (signal lost) | **Failsafe** | All actuators to safe neutral |

### Actuator command ranges

| Actuator | Stop / neutral | Range | Notes |
|---|---|---|---|
| Sail servo | 1520 µs | 1465 / 1575 µs | Binary only — no intermediate positions |
| Rotor winch | 1500 µs | 1000–2000 µs | Deviation from 1500 sets speed & direction |
| ESC 1 & 2 | 1000 µs | 1000–2000 µs | Must arm first; slew limited to 30 µs/tick |

### ESC arming sequence (ManualProp mode)

Hold CH3 (throttle) at ≤ 1050 µs for 2 continuous seconds. Serial prints `[hold throttle low to ARM]` during countdown. Arming state is cleared on any mode switch — re-arming required each time ManualProp is entered.

---

## Mission System

Missions are loaded at runtime via LoRa command. Nothing is hardcoded.

```
loadMission(plan)  →  start()  →  Running
                                     │
                    Circuit: loop indefinitely ◄──────┐
                    Linear:  advance waypoints ──(end)─┘
                                     │ last waypoint reached (Linear)
                                     ▼
                                 Returning  ←── emergencyReturn() from any state
                                     │ arrived at home point
                                     ▼
                                  Complete
```

- **MissionPlan**: up to 16 `Waypoint` structs `{lat, lon, radiusM}` + `MissionMode`
- **Home point**: set independently via `setHome(lat, lon)` over LoRa — survives mission reloads
- **Arrival detection**: `Navigator::distanceM() ≤ waypoint.radiusM` (default 10 m; home uses 15 m)
- **Emergency return**: `emergencyReturn()` via LoRa command — bypasses remaining waypoints, navigates directly to home

---

## LoRa Communication

The drone transmits a JSON heartbeat every 1 second and listens for commands at all times.  
A second T-Beam running `transceiver/transceiver.ino` acts as the ground station, bridging USB serial to LoRa.

**Radio config:** 433 MHz, default spreading factor (SF7), bandwidth 125 kHz, coding rate 4/5.

### Heartbeat (drone → ground, 1 Hz)

```json
{
  "origin": "boat",
  "type": "info",
  "message": {
    "mode": "standby | route-ready | navigate",
    "location": [48.380000, -4.490000],
    "servos": { "sail": 0, "rudder": 0 },
    "control_mode": "radio | autonomous",
    "heading": 213.5,
    "wind": 0,
    "bat": 12.54,
    "waypoints": { "total": 5, "current": 2 }
  }
}
```

Mode strings: Failsafe/Manual → `"standby"` | Auto+Idle/Complete → `"route-ready"` | Auto+Running/Returning → `"navigate"`.

### Commands (ground → drone)

| Command JSON | Effect |
|---|---|
| `{"origin":"server","type":"command","message":{"navigate":true}}` | Start mission |
| `{"origin":"server","type":"command","message":{"stop":true}}` | Stop mission |
| `{"origin":"server","type":"command","message":{"restart":true}}` | Reboot drone |
| `{"origin":"server","type":"command","message":{"home":{"lat":48.38,"lon":-4.49}}}` | Set home point |
| `{"origin":"server","type":"command","message":{"wind-command":{"value":225}}}` | Set wind direction (Phase 5) |
| `{"origin":"server","type":"command","message":{"wind-observation":true}}` | Start wind observation (Phase 5) |
| `{"origin":"server","type":"command","message":{"waypoints":{"number":2,"points":"48.38,-4.49,48.39,-4.50"}}}` | Load waypoints |

### Ground station transceiver

Flash `transceiver/transceiver.ino` onto a second T-Beam. Connect to PC via USB at 115200 baud.  
Type shorthand commands in any serial monitor — the transceiver wraps them in the correct JSON and forwards via LoRa:

```
navigate                      → start mission
stop                          → stop mission
restart                       → reboot drone
wind-obs                      → start wind observation
wind 225                      → set wind direction to 225°
home 48.380000,-4.490000      → set home point
wpt 48.380,-4.490,48.381,-4.491   → load 2 waypoints (Linear mode)
{...raw JSON...}              → pass through unchanged (for scripting)
```

Received heartbeats are printed as:
```
[RX rssi=-52] {"origin":"boat","type":"info","message":{...}}
```

---

## Serial Monitor Output

At 115200 baud, 5 Hz:

```
[SAIL    ] CH2=1500 CH3=1500 CH4=1500 CH5=1490 CH6=   0 | sail=1520 rotor=1500 esc1=1000 esc2=1000 | bat=12.54V
[GPS ] lat= 48.380123 lon=  -4.491234 spd=  0.0km/h hdg=  0.0° sat=8 hdop=1.2 age=200ms
[LORA] tx=42  rxRssi=-52
```

When in Automatic mode, a fourth line is added:
```
[MISN ] RUNNING  LINEAR  wp=2/5 target=(48.381000,-4.492000 r=10m) dist=47m brg=213°
```

When GPS has no fix:
```
[GPS ] NO FIX  visible=9  chars=35280  badCRC=0
       last: $GPGSV,3,1,09,02,45,185,40,...
```

When LoRa receives a command, it prints immediately:
```
[LORA] RX rssi=-52  {"origin":"server","type":"command","message":{"navigate":true}}
[LORA] CMD: navigate → startMission
```

---

## Building

Requires the **ESP32 Arduino core** (esp32:esp32 3.x) and these libraries:

- `AXP202X_Library` 1.1.2
- `TinyGPSPlus` 1.0.3
- `LoRa` 0.8.0

### Command-line (arduino-cli)

```bash
# Drone firmware
~/bin/arduino-cli compile --fqbn esp32:esp32:t-beam \
  /path/to/Informatica/Post-Claude/main

# Ground station transceiver
~/bin/arduino-cli compile --fqbn esp32:esp32:t-beam \
  /path/to/Informatica/Post-Claude/transceiver

# Flash (adjust port as needed)
~/bin/arduino-cli upload --port /dev/ttyUSB0 --fqbn esp32:esp32:t-beam \
  /path/to/Informatica/Post-Claude/main
```

Last successful build: **359 KB flash (27%), 25 KB RAM (7%)** — drone firmware.  
Transceiver: **302 KB flash (23%), 21 KB RAM (6%)**.

### Arduino IDE

Open `main/main.ino` (drone) or `transceiver/transceiver.ino` (ground station) in Arduino IDE 2.x, select board **T-Beam**, port, and Upload.

### Serial monitoring (Linux, no IDE)

```bash
python3 -m venv /tmp/venv && /tmp/venv/bin/pip install pyserial -q
/tmp/venv/bin/python3 -c "
import serial, time, sys
s = serial.Serial()
s.port='/dev/ttyUSB0'; s.baudrate=115200; s.timeout=0.5
s.dsrdtr=False; s.rtscts=False; s.open()
s.dtr=False; s.rts=False
while True:
    line = s.readline()
    if line: print(line.decode('utf-8','replace').rstrip(), flush=True)
"
```

> **Note:** Opening the port with DTR/RTS asserted holds the ESP32 in reset. Use `dsrdtr=False` and explicitly deassert `dtr`/`rts` after opening, as shown above.

---

## Native Unit Tests

Logic tests run on the host (no hardware needed):

```bash
cd test
mkdir -p build && cd build
cmake .. && make
./test_runner
```

Currently covers ModeManager and ManualController: **106 tests, all passing**.

---

## Implementation Status

| Phase | Scope | Status |
|---|---|---|
| 1 — Manual RC | AXP192, RC 5-ch interrupts, MCPWM actuators, mode switching, ESC arming | ✅ Complete — 106 tests pass |
| 2 — GPS | GpsUart driver (TinyGPSPlus), position displayed on serial | ✅ Complete — hardware tested |
| 2b — Navigation | Navigator (haversine), MissionPlan, MissionManager state machine | ✅ Complete |
| 3 — LoRa | SX1276 TX heartbeat (1 Hz JSON) + RX command parsing + transceiver sketch | ✅ Complete — hardware tested |
| 4 — Sensors | I2C sensor bus (pins TBD — not GPIO21/22), sample collection | 🔲 Not started |
| 5 — Wind estimator | GPS-track-based wind direction estimation, 2-tack method | 🔲 Not started |
| 6 — Autonomous sailing | SailAutoMode, PropulsionAutoMode, WinchTracker | 🔲 Not started |
| 7 — Full mission | Load waypoints via LoRa, log to SPIFFS, full water test | 🔲 Not started |

---

## Known Hardware Issues

| Issue | Impact | Resolution |
|---|---|---|
| GPIO23 shared by LoRa RST and RC CH6 | CH6 and hardware LoRa reset cannot both be active | Solved: LoRa driver skips RST (passes -1) — SX1276 inits reliably without it |
| GPS requires LiPo battery for warm starts | Without battery, cold start on every power cycle (~2 min to fix outdoors) | Install LiPo before field testing |
| ESC LiPo cell count requires EPRG-3 card | Wrong LVC cutoff if not programmed | Program before any powered ESC test |
| GPIO35 unusable for external ADC | T-Beam has a 10 kΩ + 10 kΩ internal divider permanently soldered to GPIO35 — clamps and drains any external signal | Battery ADC uses GPIO15 with external divider instead |
| GPIO32/33 shared by ESC outputs and LoRa DIO1/DIO2 | Potential conflict if LoRa library uses DIO1/2 interrupt path | Not an issue: LoRa library uses only DIO0 (GPIO26) for polling; DIO1/2 unused |
| LoRa TX blocks main loop ~450 ms | GPS UART might lose chars; control loop pauses during TX | GPS FIFO buffers adequately; acceptable for Phase 3. Switch to async TX before Phase 6 |

---

## Repository Layout

```
Post-Claude/
├── CLAUDE.md                  — full project reference, architecture decisions, progress log
├── README.md                  — this file
├── main/
│   ├── main.ino               — entry point
│   └── src/
│       ├── app/               — DroneApp orchestrator
│       ├── comm/              — LoRaComm protocol layer
│       ├── config/            — BoardConfig.h, Calibration.h (pin assignments, µs ranges)
│       ├── control/           — ModeManager, ManualController
│       ├── core/              — Types.h (enums and structs shared across layers)
│       ├── drivers/           — AxpPower, RcReceiver, McpwmActuators, GpsUart, LoRaRadio, BatteryAdc
│       └── navigation/        — Navigator, MissionPlan, MissionManager
├── transceiver/
│   └── transceiver.ino        — ground station sketch (second T-Beam, USB ↔ LoRa bridge)
└── test/
    ├── CMakeLists.txt
    ├── stubs/Arduino.h        — minimal stub for host compilation
    └── test_runner.cpp        — 106 test cases
```
