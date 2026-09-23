# Sail Drone T-Beam — Project Reference & Progress Log

**Author:** Facundo Arito  
**Board:** LilyGO T-Beam V1.1 (ESP32 + SX1276 LoRa + u-blox GPS + AXP192)  
**Language:** Arduino (C++), targeting ESP32 Arduino core  
**All new files must go in:** `Informatica/Post-Claude/`

---

## 1. Physical Concept

The drone is a sail-powered surface vehicle. The key mechanics:

- **Wing-profile sail** freely rotates around a vertical axis attached to the hull. Its aerodynamic profile passively orients it so the wind is always ~45° from the drone's centerline.
- **Aileron servo (sail servo):** Controls a small flap at the trailing edge of the sail. Range is strictly ±10°. This binary-ish deflection determines whether the sail pushes the drone to port or starboard. It cannot be moved to arbitrary angles — only the two extreme positions have physical meaning.
- **Rotor servo (Safran/rudder servo):** Controls the differential between the sail's free rotation and a fixed rotor at the stern. This is the primary yaw/heading actuator. **The Regatta ECO II is a positional multi-turn winch servo** — PWM sets a target shaft position (not a speed). 1500 µs = mechanical center, 1000 µs = one travel extreme, 2000 µs = other extreme. It holds position like a standard servo but has 6 full turns of travel instead of 180°. In manual mode CH4 stick position maps directly to rotor position; deadband snaps to exact center.
- **Propeller (single ESC, GPIO15):** Used when wind is insufficient. In the unified manual mode it runs alongside the servos, throttled by CH3 (see Section 13). ESC2/differential thrust removed in PCB V2.
- **No wind sensor** — wind direction must be estimated from GPS track and actuator state.

### Control summary

| What you want | How you do it |
|---|---|
| Change lateral drift direction | Toggle aileron servo ±10° (CH2) |
| Change heading / yaw | Move rotor servo (CH4) |
| Run the propeller | CH3 throttle (manual mode, single ESC, inverse) |
| Switch mode | CH5: low=Auto, mid=Sail (inert), high=Manual |

---

## 2. Hardware & Pin Mapping

### LilyGO T-Beam V1.1 built-in
- ESP32 WROVER module
- SX1276 LoRa (433 MHz or 868/915 MHz depending on variant)
- u-blox NEO-6M GPS (internal, but project uses external via Serial1)
- AXP192 power management IC (internal I2C at GPIO21/22 — see conflict note below)
- USB-C, LiPo connector, 18650 battery holder

### PCB V2 pin mapping (from `BoardConfig.h` + schematic) — FINAL

| Signal | GPIO | Notes |
|---|---|---|
| RC CH2 (sail toggle) | 39 | SVN, input-only GPIO |
| RC CH3 (propulsion throttle) | 14 | |
| RC CH4 (rotor) | 13 | |
| RC CH5 (mode selector) | 4 | |
| Sail servo PWM out | 2 | Via BC548 NPN level shifter (DUTY_MODE_0) |
| Rotor servo PWM out | 25 | Via BC548 NPN level shifter (DUTY_MODE_0) |
| ESC 1 PWM out | 15 | Single ESC — no ESC2 |
| Battery ADC | 36 | SVP, external divider R5=562kΩ / R6=120kΩ |
| GPS RX (Serial1) | 34 | Input only GPIO |
| GPS TX (Serial1) | 12 | |
| LoRa SCK | 5 | SPI HSPI |
| LoRa MISO | 19 | SPI HSPI |
| LoRa MOSI | 27 | SPI HSPI |
| LoRa CS | 18 | SPI HSPI |
| LoRa RST | 23 | Now enabled (CH6 removed) |
| LoRa IRQ | 26 | |
| I2C SDA (expansion) | 21 | Free — wired to PCB expansion connector |
| I2C SCL (expansion) | 22 | Free — wired to PCB expansion connector |

**I2C status:** GPIO21/22 are now FREE in PCB V2. RC channels no longer use these pins. Wire.end() removed from AxpPower — I2C bus stays persistent for future sensors. The expansion connector on PCB V2 exposes GPIO21/22 for external I2C devices.

### PCB components (V2 schematic)
- NPN transistors for 3.3V→5V level shifting on PWM outputs
- Schottky diodes for power path protection
- Step-down regulator (Pololu ref: 2866) for 5V rail from battery
- Capacitors on power rails
- Screw terminals for ESC power (`POW_ESC1`, `POW_ESC2`), battery, servos
- 3-pin connectors for each servo/ESC output (`PWM1`, `PWM2`, `ESC1`, `ESC2`)
- 8-pin headers matching T-Beam expansion

### RC Receiver
- Pro-Tronik 2.4 GHz
- Standard servo PWM (1000–2000 µs, 50 Hz)
- CH5 controls mode (3-pos switch): ≤1250 µs = **Automatic**, 1400–1600 µs = **Sail** (inert), >1800 µs = **Manual**
- CH3 (propeller throttle) measured travel: 1100 µs = 100 %, 1990 µs = 0 % (**inverse**)

### Calibration values (from `Calibration.h` — Post-Claude)
- Sail servo center: **1520 µs** (Futaba S3003 standard)
- Sail servo ±10° positions: SAIL_PLUS_US=1575, SAIL_MINUS_US=1465 — **NEEDS BENCH CALIBRATION**
- Rotor servo: **limited to ±90°** → ROTOR_MIN_US=1417, ROTOR_MAX_US=1583, center 1500 µs, deadband ±35 µs, ROTOR_RANGE_DEG=90
  - Full hardware range is 1000–2000 µs (±3 turns = ±1080°), limited to ±83 µs from center for controllability
- CH4 measured travel: CH4_MIN_US=1180, CH4_MAX_US=1790 (center ~1500 µs) — maps to ROTOR_MIN/MAX in manual mode
- Auto-mode rudder uses only ±ROTOR_AUTO_RANGE_DEG=20° of winch travel (full ±90° too aggressive)
- ESC **BIDIRECTIONAL** (2026-06-10): ESC_REVERSE_MIN_US=1000 (−100 % reverse), ESC_NEUTRAL_US=1500 (neutral/stop, = ESC_STOP_US), ESC_MAX_US=2000 (+100 % forward); single ESC on GPIO15. ⚠️ Reverse only physically works if the ESC is programmed for bidirectional throttle via the **EPRG-3 card** — otherwise <1500 µs is ignored.
- **Propeller (manual mode):** CH3 bidirectional ratchet throttle — CH3_FULL_US=1100 → +100 % forward, CH3_CENTER_US=1545 → 0 % stop (deadband ESC_CENTER_DEADBAND_US=40 µs), CH3_ZERO_US=1990 → −100 % reverse. CH3=0 (lost) → neutral. **No software arming gesture** (removed — see Section 13). CH3_CENTER_US needs bench verification.
- Slew rate limit: 30 µs/tick (prevents hard jerks on actuator changes)
- Battery divider: R5=562kΩ (battery→ADC), R6=120kΩ (GND), ratio=5.683, GPIO36 ADC1_CH0

---

## 3. Software Architecture

The architecture from `Post_GPT` is adopted and extended. It is **layered OOP, event-loop style** (no RTOS tasks, single-core loop with a software scheduler).

```
main.ino
  └── DroneApp (orchestrator, tick-based scheduler)
        ├── DRIVERS (hardware interface)
        │     ├── RcReceiver      — interrupt-driven, 4 channels (CH2/3/4/5), ISR-safe
        │     ├── McpwmActuators  — MCPWM (not LEDC!) for sail, rotor, ESC1 (single)
        │     ├── GpsUart         — Serial1, TinyGPSPlus parser
        │     ├── LoRaRadio       — SX1276 via LoRa library, SPI, RST=GPIO23
        │     ├── BatteryAdc      — GPIO36 ADC, R5=562kΩ/R6=120kΩ divider
        │     ├── SensorBus       — I2C sensors (pins TBD — not GPIO21/22)
        │     ├── Storage         — SPIFFS or SD card
        │     └── BtPositionBridge— Bluetooth fallback position
        ├── SERVICES (data processing)
        │     ├── PositionService — fuses GPS + BT sources, marks stale
        │     ├── BatteryService  — voltage, percentage, state
        │     ├── TelemetryService— LoRa heartbeat TX + command RX
        │     ├── LoggingService  — status log + sensor sample routing
        │     ├── SensorManager   — polls SensorBus, buffers samples
        │     └── WindEstimator   — infers wind from GPS track + actuator state
        ├── NAVIGATION
        │     ├── Navigator       — haversine distance + bearing
        │     ├── MissionPlan     — ordered waypoint list (max 32)
        │     └── MissionManager  — state machine: Idle/Running/Holding/Completed/Failed
        ├── CONTROL
        │     ├── ModeManager     — CH5 → ControlMode enum
        │     ├── ManualController— unified manual: sail + winch + propeller (CH3)
        │     ├── AutoController  — dispatches to SailAutoMode or PropulsionAutoMode
        │     ├── SailAutoMode    — autonomous sail navigation (THE CORE ALGORITHM)
        │     └── PropulsionAutoMode — autonomous propeller navigation
        └── CONFIG (header-only constants)
              ├── BoardConfig.h   — pin assignments
              ├── MissionConfig.h — timing periods, limits
              └── Calibration.h   — servo/ESC microsecond ranges
```

### Update timing (from `MissionConfig.h`)
| Task | Period |
|---|---|
| Control loop (RC read, actuator write) | 20 ms |
| Position update | 100 ms |
| Battery | 250 ms |
| Wind estimator | 250 ms |
| Sensors | 500 ms |
| Telemetry (LoRa heartbeat) | 1000 ms |
| Logging | 1000 ms |
| Debug serial print | 100 ms |

---

## 4. Critical Technical Decisions

### MCPWM instead of LEDC (non-negotiable)
The ESP32 LEDC peripheral conflicts with the interrupt-driven RC receiver. When LEDC is active, the RC pulse interrupts are unreliable. MCPWM avoids this. This was discovered and resolved in the Post_GPT rewrite. **Never use ESP32Servo or analogWrite() for the actuators in this project.**

### Interrupt-driven RC reading
RC PWM is decoded via GPIO interrupts (CHANGE edge). Timing is captured with `micros()`. A timeout of 100 ms marks a channel as lost (returns 0). This approach is ISR-safe and avoids `pulseIn()` blocking.

### Actuators commanded in microseconds
All actuator outputs are in microseconds (1000–2000 µs range), not in degrees. This avoids unit conversion errors and maps directly to the hardware.

### Sail is a binary state
The aileron servo only has two meaningful positions: +10° and -10°. In code, this is represented as `sailState = +1` or `sailState = -1`. The servo value is computed as `SAIL_CENTER_US ± SAIL_DELTA_US`.

### Wind estimation without a sensor
Wind direction is inferred from the GPS track over time, combined with knowledge of which tack the boat is sailing on. A confidence level must be tracked. The estimator should require a minimum travel distance before claiming a valid estimate.

---

## 5. Key Algorithms to Implement

### 5.1 WindEstimator

The boat's aerodynamics guarantee that wind is always ~45° from centerline when sailing freely. From the GPS track:
- If `sailState = +1` (aileron pushes to starboard), the boat drifts in the direction of the wind vector's projection
- Collect heading samples while sailing a straight line on each tack
- Wind bisects the angle between the two tack headings
- Minimum required travel distance before estimate is considered valid: suggest 30 m

### 5.2 SailAutoMode (autonomous sailing)

```
Input: current position, waypoint, wind direction estimate, current GPS heading
Output: sailUs (±delta), rotorUs (continuous)

1. Compute bearing_to_waypoint and distance_to_waypoint
2. Compute relative_angle_wind_to_waypoint (angle between wind direction and bearing)
3. Determine if waypoint is "reachable" (not in the upwind dead zone ±45°):
   - If reachable:
       a. Choose tack (sail state) that puts the boat on a heading toward the waypoint
       b. Set aileron to that tack state
       c. Set rotor to steer toward waypoint (proportional or bang-bang)
   - If not reachable (upwind):
       a. Use tacking maneuver: sail on one tack for a calculated distance
       b. Switch tack when enough lateral progress has been made
       c. The two tack headings should be approximately ±45° from the wind direction
4. Check waypoint reached (within acceptance radius, default 10 m)
5. Advance to next waypoint
```

### 5.3 PropulsionAutoMode (no-wind fallback)

```
Input: current position, waypoint
Output: esc1Us, esc2Us (differential for steering)

1. Compute bearing error (desired heading - GPS track heading)
2. Apply proportional steering: differential between ESC1 and ESC2
3. Throttle: fixed medium value or adaptive
4. Check waypoint reached
```

### 5.4 LoRa protocol (IMPLEMENTED — Phase 3)

JSON format matching the original `boat.ino` protocol for server compatibility.
Parsed with `strstr()` on char buffers — no ArduinoJson dependency.

**Heartbeat TX (drone → ground), every 1 s** (current format — `control_mode` removed, GPS/RC + wind-obs fields added, size-trimmed):
```json
{"origin":"boat","type":"info","message":{"mode":"standby|route-ready|navigate",
"location":[lat,lon],"servos":{"sail":±10,"rudder":deg},
"heading":H,"wind":0,"bat":0.00,
"fix":0|1,"sat":N,"hdop":X.X,"rc":0|1,
"wt":total,"wc":current,"wobs":NN,"rssi":-NNN}}
```
- `servos.sail` = ±10° (binary), `servos.rudder` = winch degrees (manual ±90° / auto ±20°).
- `fix/sat/hdop` = GPS health; `rc` = 1 if the RC receiver is delivering pulses (any channel ≠ 0).
- **`wt`/`wc`** = waypoints total/current (flattened from the old `waypoints:{total,current}` object — saves ~22 B). IHM reads `wt`/`wc` (fallback to old shape).
- **`wobs`** = wind-observation progress 0–100 % — **conditional**: present ONLY while a wind measurement is running (`AutoController::windObsProgressPct()` = travelled / `WIND_OBS_DISTANCE_M`). Absent the rest of the time.
- **`control_mode` deleted** — 100 % correlated with `mode` (standby ⟺ radio, route-ready/navigate ⟺ autonomous). The IHM derives it. Frees ~28 B.
- **Size cuts to stay well under 255 B**: `control_mode` removed, `heading` integer (`%.0f`), `location` 5 decimals (`%.5f` ≈ 1.1 m, map-only), `waypoints` flattened to `wt`/`wc`.
- **`rssi`** is **NOT sent by the boat** — the transceiver injects `,"rssi":<packetRssi>` before the closing `}}` on each forwarded line (measured on the ground side, over USB → doesn't count against the 255 B LoRa budget).
- Mode mapping: Failsafe/Manual/Sail → `"standby"`, Auto+Idle/Complete → `"route-ready"`, Auto+Running/Returning → `"navigate"`.
- Measured size ≈ 210 B with rssi (≈ 224 B with `wobs`); worst case ≈ 232 B — comfortable margin.

**Commands RX (ground → drone):**
```json
{"origin":"server","type":"command","message":{"navigate":true}}
{"origin":"server","type":"command","message":{"stop":true}}
{"origin":"server","type":"command","message":{"restart":true}}
{"origin":"server","type":"command","message":{"wind-observation":true}}
{"origin":"server","type":"command","message":{"wind-command":{"value":225}}}
{"origin":"server","type":"command","message":{"home":{"lat":48.38,"lon":-4.49}}}
{"origin":"server","type":"command","message":{"waypoints":{"number":2,"points":"lat1,lon1,lat2,lon2"}}}
```

**Packet size limit**: SX1276 FIFO = 255 bytes. Heartbeat ≈ 210 bytes. Waypoint commands with many points approach the limit — roughly 8 waypoints at full precision.

**TX blocking note**: `LoRa.endPacket()` is synchronous. At SF7/BW125 kHz, a 210-byte packet takes ≈ 450 ms. This blocks the main loop once per second. Acceptable for Phase 3; switch to async before Phase 6 if control loop jitter becomes an issue.

---

## 6. Current Implementation Status

### Post-Claude Phase 0 — IHM Ground Station (now at `IHM/`, unified into Post-Claude)

| Module | File | Status | Notes |
|---|---|---|---|
| `webserver.py` | `IHM/app/webserver.py` | ✅ | FastAPI, port 5000, sirve HTML estático |
| `serial_link.py` | `IHM/app/serial_link.py` | ✅ | Lee/escribe JSON por serial, MongoDB como bus. Ráfaga 3× (anti-colisión LoRa). **Re-resuelve el puerto por nº de serie en cada (re)conexión** y reconecta solo ante desconexión/re-enumeración USB. Abre con dsrdtr/rtscts=False (no resetea el ESP32 al reconectar) |
| `config.py` | `IHM/app/config.py` | ✅ | **Auto-detección del transceiver por nº de serie CP2104** vía `/dev/serial/by-id/` (sin udev/root). Prioridad: override CLI → detección → fallback .env. `find_transceiver_port()` / `resolve_serial_port()` |
| `routes/messages.py` | `IHM/app/routes/messages.py` | ✅ | API REST. `BASE_DIR` dinámico. `reset-transceiver` y relanzado usan detección por serie |
| `static/nav.js` | `IHM/app/static/nav.js` | ✅ | **Réplica JS exacta de `navigation.h`** + simulador cinemático para la predicción de trayectoria |
| `static/map.js` + `script.js` | `IHM/app/static/` | ✅ | Leaflet: waypoints con clic, enviar ruta, polling 1 Hz, brújula de viento, toggle trayectoria, barra de progreso medición de viento |
| `start_ihm.sh` / `stop_ihm.sh` | `IHM/` | ✅ | **Reescritos robustos**: stop mata TODO (TERM→KILL, libera puerto 5000 = worker uvicorn, `docker compose down`); start limpia desde cero (llama a stop), trunca logs, Python `-u` (logs en vivo), verifica cada componente. `start_ihm.sh /dev/ttyUSBx` fuerza puerto |
| `99-autoboat.rules` | `IHM/` | ✅ | Regla udev opcional → `/dev/ttyAUTOBOAT_TRX` y `_BOAT` fijos. **Transceiver = serie 01C00B54, barco = 02126CF1** |
| `docker-compose.yml` | `IHM/` | ✅ | MongoDB local (mongo:7, 27017). `.env`: SIMULATION=false, TRANSCEIVER_SERIAL=01C00B54, DB_NAME=autoboat |

**Panel de diagnóstico (`static/`):** semáforos de salud Liaison / GPS / Batterie / Radio (RC) / Contrôle.
- **Liaison**: por antigüedad del último timestamp NUEVO (verde <3 s, amarillo 3–10 s, rojo >10 s); muestra **RSSI dBm** (amarillo si ≤ −110).
- **GPS**: usa `fix`/`sat`/`hdop` (verde si fix & sat≥5 & hdop≤2.5).
- **Batterie**: umbrales LiPo 2S (verde ≥7.0 V, amarillo ≥6.6, rojo <6.6).
- **Radio (RC)**: `rc`=1 → OK.
- **Contrôle**: derivado de `mode` — verde "auto", **celeste** "radio".
- Si la Liaison se pierde, los otros pasan a amarillo + advertencia en francés ("paramètres non fiables").

**Toggle Sim/Réel** (persistido en localStorage): en modo Réel oculta Start/Réinit-coms y llama `POST /api/set-mode` (relanza serial_link en el puerto real); en Sim muestra esos botones (la pila se lanza con Start). Botón **🔌 Reconnecter TRX** (solo modo real) → `POST /api/reset-transceiver` (pulso DTR/RTS = hard-reset del ESP32 + relanza serial_link).

**Endpoints añadidos en `messages.py`:** `/api/stop`, `/api/set-mode`, `/api/reconnect`, `/api/reset-transceiver` (+ `launch_process` acepta `env=`).

**Brújula de viento** (esquina sup. izq. del mapa): flecha que apunta a la **fuente** del viento (`wind` = de dónde viene; mapa norte-arriba). Gris si no hay dato. El control de zoom de Leaflet se movió a la derecha para liberar la esquina.

**Predicción de trayectoria** (`nav.js`, toggle "🧭 Trajectoire"): réplica exacta de `navigation.h` en JS (mismas constantes/llamada que `AutoController`) + simulador cinemático (el rumbo sigue al timón, velocidad constante — el firmware no define la dinámica, así que el TRAZADO es aproximado aunque la lógica de barra/vela sea exacta; constantes `SIM_*` ajustables en `nav.js`). Dibuja polilínea punteada del barco al waypoint `wc`.

**Barra de progreso de medición de viento**: sección "📏 Mesure du vent" visible solo mientras llega `wobs`; se llena hasta 100 % a los 30 m y desaparece al terminar.

**Notas IHM:** (1) El puerto del transceiver ya no importa — se **auto-detecta por nº de serie** (`config.py`). (2) Nunca abrir gtkterm/pyserial sobre el puerto del transceiver mientras corre la IHM — dos lectores compiten y corrompen el JSON. (3) Falta en la UI el botón **Stop** aunque el endpoint existe.

### Post-Claude Phase 1 — Manual RC Control ✅ COMPLETE (⚠️ unit tests now STALE — see note)

| Module | File | Status | Notes |
|---|---|---|---|
| `AxpPower` | `drivers/AxpPower.h/.cpp` | ✅ | AXP192 init: LDO2/LDO3/DCDC1 enabled. Wire.end() REMOVED — I2C stays persistent |
| `RcReceiver` | `drivers/RcReceiver.h/.cpp` | ✅ | 4-ch (CH2/3/4/5) interrupt PWM, ISR-safe portMUX, 100 ms timeout. CH6 removed. |
| `McpwmActuators` | `drivers/McpwmActuators.h/.cpp` | ✅ | MCPWM Timers 0+1, slew limiting, single ESC on GPIO15. ESC2 removed. |
| `ModeManager` | `control/ModeManager.h/.cpp` | ✅ | CH5 → Failsafe / **Sail** (inert) / **Manual** / Automatic |
| `ManualController` | `control/ManualController.h/.cpp` | ✅ | **Unified manual mode**: binary sail (CH2) + winch (CH4) + propeller (CH3 inverse, 10 % deadzone). Arming removed. |
| `DroneApp` | `app/DroneApp.h/.cpp` | ✅ | 20 ms control tick; dispatch Manual→manual, Sail→neutral, Auto/Failsafe→mission |
| `Types` | `core/Types.h` | ✅ | ControlMode enum (Failsafe/Sail/Manual/Automatic), RcFrame, ActuatorCommand |
| `BoardConfig` | `config/BoardConfig.h` | ✅ | Pin assignments, RC thresholds, CH3/CH4 travel |
| `Calibration` | `config/Calibration.h` | ✅ | Servo/ESC µs values (sail positions need bench calibration) |
| `main.ino` | `main/main.ino` | ✅ | Instantiates DroneApp, calls begin()/update() |

⚠️ **Test suite stale (does NOT build):** `test/test_runner.cpp` predates the `main/src/` reorg (can't find `core/Types.h`) and the single-ESC migration (references `esc2Us`, old 1300/1700 CH5 thresholds, `isEscArmed()`, the removed arming). The "106/106" milestone is historical. Firmware itself compiles & flashes fine; the native test build is broken and needs a rewrite to match current behavior (Sail/Manual modes, CH3 inverse throttle, rotor remap). Separate task — not done yet.

### Post-Claude Phase 2 — GPS + Navigation ✅ COMPLETE (hardware tested outdoors)

| Module | File | Status | Notes |
|---|---|---|---|
| `GpsUart` | `drivers/GpsUart.h/.cpp` | ✅ | Serial1 GPIO34/12, TinyGPSPlus, two-buffer NMEA capture, satsInView() |
| `Navigator` | `navigation/Navigator.h/.cpp` | ✅ | Haversine distanceM() + bearingDeg(), namespace (no state) |
| `MissionPlan` | `navigation/MissionPlan.h` | ✅ | Up to 16 Waypoints, MissionMode (Linear/Circuit) |
| `MissionManager` | `navigation/MissionManager.h/.cpp` | ✅ | State machine Idle→Running→Returning→Complete, emergencyReturn() |
| `Types` | `core/Types.h` | ✅ | Added GpsPosition, Waypoint, MissionMode, MissionState |

GPS hardware notes: board requires LiPo battery for warm starts (without battery every power cycle = cold start, ~2 min for fix). LED on NEO-6M TIMEPULSE pin: off = no fix, 1 Hz blink = fix acquired.

### Post-Claude Phase 3 — LoRa ✅ COMPLETE (hardware tested — TX verified, RX functional)

| Module | File | Status | Notes |
|---|---|---|---|
| `LoRaRadio` | `drivers/LoRaRadio.h/.cpp` | ✅ | SPI HSPI, 433 MHz, RST=GPIO23 (enabled — CH6 removed), blocking TX |
| `LoRaComm` | `comm/LoRaComm.h/.cpp` | ✅ | JSON heartbeat TX 1 Hz (now with sail/rudder deg + fix/sat/hdop/rc, no control_mode), command dispatch (navigate/stop/home/waypoints/wind/restart) |
| `DroneApp` | `app/DroneApp.h/.cpp` | ✅ | loraHbTick() at 1 s, lora_.update() every loop, [LORA] debug line |
| Transceiver | `transceiver/transceiver.ino` | ✅ | Ground station — shorthand commands + raw JSON passthrough + **injects RSSI** into each forwarded heartbeat. Hardware tested. |

### Post-Claude Phase 5 — Wind Estimation from GPS ✅ CODED (untested on water)

| Module | File | Status | Notes |
|---|---|---|---|
| `AutoController` (wind obs) | `control/AutoController.h/.cpp` | ✅ | `beginWindObservation()`/`observeWind()`: sails fixed +10° tack, circular-EMA smooths GPS course, after `WIND_OBS_DISTANCE_M`=30 m latches wind = smoothHeading+90° via `nav_handleWindObservation()` |
| `DroneApp` | `app/DroneApp.h/.cpp` | ✅ | `windObsActive_`/`windValid_` flags; wind-observation cmd wired; nav holds neutral until wind known |

⚠️ The +90° offset and 30 m threshold need field validation. Manual `wind-command` (Envoi vent) is the reliable fallback. **Observability gap:** heartbeat looks identical to idle during observation — can't tell from telemetry if it's running until `wind` changes after 30 m travel.

### Post-Claude Phase 6 — Autonomous Sailing ✅ CODED (untested on water, no unit tests)

| Module | File | Status | Notes |
|---|---|---|---|
| `navigation.h` | `navigation/navigation.h` | ✅ | **Upgraded 2026-06-10 to DeltaGod/PER_MEA_LYINCROYAP branch `Testautoboat`** — upwind/downwind zigzag, gybe avoidance (empannage), cross-track corridor, lofer/abattre. **New vs prior port:** smarter initial tack via `nav_sideMovingTowardWaypoint()` (picks side making most progress to the waypoint, not just wind-axis side); "passed-the-waypoint-plane within corridor" arrival check via `nav_hasPassedWaypoint()`; default corridor half-width 20→**100 m**. Public entry points (`nav_handleNavigationWithState`, `nav_handleWindObservation`) UNCHANGED — drop-in. **Do not change logic.** |
| `NavigationConfig.h` / `NavigationSelector.h` | `navigation/` | ✅ | Compile-time switch: `USE_OLD_NAVIGATION` (default 0 = current `navigation.h`; 1 = `oldNavigation.h`). AutoController includes the selector, not navigation.h directly. |
| `oldNavigation.h` | `navigation/oldNavigation.h` | ✅ | Legacy fallback (same public names, empty `NavState{}`). NOT compiled unless the switch is flipped. |
| `AutoController` | `control/AutoController.h/.cpp` | ✅ | Includes `NavigationSelector.h`; persists rudder/sail angle state; maps nav rudder **1:1 to physical winch degrees, clamped ±ROTOR_AUTO_RANGE_DEG=20°** (full ±90° was too aggressive). NOTE: AutoController itself kept as Facundo's adaptation — Testautoboat's AutoController/DroneApp/LoRaComm/ManualController were NOT merged (they predate the single-ESC + unified-manual + heartbeat work). |
| `DroneApp` | `app/DroneApp.h/.cpp` | ✅ | controlTick dispatches to AutoController in Automatic+Failsafe when target active & wind valid |

### Post_GPT (reference only — do not modify)

| Module | Status |
|---|---|
| `RcReceiver`, `McpwmActuators`, `ModeManager`, `ManualController` | ✅ Working — adopted as reference |
| `Navigator`, `MissionPlan`, `MissionManager` | ✅ Working — ported to Post-Claude Phase 2 |

### Phases not yet started (deliberately deferred per scope decision)
Sensors / `SensorBus` (Phase 4), Storage/Logging + Autonomous Propulsion + WinchTracker + Full Mission (Phase 7). Also pending: unit tests for navigation/AutoController, bench calibration of SAIL_PLUS/MINUS_US and rudder gains.

---

## 7. Implementation Plan

### Phase 1 — Manual RC Control ✅ CODED
- AXP192 boot, RC receiver (5ch interrupts), MCPWM actuators
- ManualController: binary sail + ESC arming + differential thrust
- **Next action: bench test (see Section 14)**

### Phase 2 — GPS integration ✅ COMPLETE
- GpsUart driver (TinyGPSPlus), position/speed/heading/sats/hdop displayed on serial
- Navigator (haversine), MissionPlan, MissionManager state machine
- Hardware tested: GPS receives NMEA (cold start without LiPo — fix after ~2 min outdoors)
- **Remaining hardware action**: install LiPo battery for warm start

### Phase 3 — LoRa integration ✅ COMPLETE (all 7 uplink commands verified)
- LoRaRadio driver: SPI HSPI, 433 MHz, RST=GPIO23 (enabled after PCB V2 removes CH6)
- LoRaComm: JSON heartbeat TX at 1 Hz, full command dispatcher
- Transceiver ground station sketch: shorthand CLI + raw JSON passthrough
- Hardware tested: all 7 uplink commands verified (navigate/stop/home/wpt/wind-cmd/wind-obs/restart)
- End-to-end test complete: RSSI −101 to −106 dBm at close range

### Phase 4 — Sensors (I2C)
- Identify which sensors are used and their I2C addresses
- GPIO21/22 are now available (PCB V2 expansion connector) — can be used for I2C sensors
- Implement `SensorBus.cpp` with bus scan and sample collection

### Phase 5 — Wind Estimator
- GPS-track-based wind estimation
- Require minimum 30 m travel and 2 tack observations for valid estimate
- Manual wind override via LoRa command as fallback

### Phase 6 — Autonomous Sailing
- Implement `SailAutoMode.cpp` using algorithm in Section 5.2
- **WinchTracker** will be needed: software position estimator for Regatta ECO II
  (speed × time integration — the winch has no position feedback)
- Test bench-side first (simulate position updates), then on water at low speed
- Tune `SAIL_PLUS_US`, `SAIL_MINUS_US`, and rotor gain constants

### Phase 7 — Full Mission Test
- Load waypoints via LoRa
- Start mission, monitor via LoRa heartbeats
- Log sensor data to SPIFFS

---

## 8. Open Questions (resolve before implementing)

1. **Sensors**: What specific sensors are used? (temperature, salinity, turbidity, pH, current…) What are their I2C addresses? Which alternate I2C pins are available on the PCB?
2. **Storage**: Is there an SD card slot on the PCB V2? Or use SPIFFS only?
3. **Ground station**: ✅ Resolved — second T-Beam running `transceiver/transceiver.ino` (Post-Claude). Bridges USB serial ↔ LoRa. Shorthand commands: navigate, stop, restart, wind-obs, wind \<deg\>, home \<lat,lon\>, wpt \<lat,lon,...\>.
4. **Compass/IMU**: Is there an IMU or magnetometer? Crucial when stationary or in currents for heading without GPS motion. Not mentioned in any file so far.
5. **Single vs dual propeller**: ✅ Resolved — PCB V2 uses single ESC on GPIO15. ESC2 removed from code and hardware.
6. **LoRa band**: 433 MHz confirmed in both versions. Regional regulations (Europe/Argentina)?
7. **Bluetooth position bridge**: Is this feature actually planned? (GPS phone backup via BT SPP)

### Resolved questions
- **AXP192 usage**: ✅ Yes — AxpPower.cpp explicitly enables LDO2 (LoRa), LDO3 (GPS), DCDC1 (3.3V), battery ADC. Wire.end() REMOVED from success path — I2C bus stays persistent for future sensors on GPIO21/22 expansion connector.
- **Battery ADC**: ✅ Implemented — BatteryAdc.cpp reads GPIO36 (SVP, ADC1_CH0) with external voltage divider R5=560kΩ / R6=120kΩ (ratio=5.683). This is NOT GPIO35 (which has a low-resistance divider causing 20 mA drain). GPIO36 divider uses high-value resistors: drain ≈ 10.9 µA at 7.4 V.
- **Sail center calibration**: ✅ Futaba S3003 center = 1520 µs confirmed. Calibration.h updated. SAIL_PLUS_US=1575, SAIL_MINUS_US=1465 still need bench verification.

---

## 9. Files in This Folder

| File | Description |
|---|---|
| `CLAUDE.md` | This file — project reference and progress log |
| `main/main.ino` | Entry point — calls DroneApp.begin() and DroneApp.update() |
| `main/config/BoardConfig.h` | All pin assignments, RC thresholds, mode selector thresholds |
| `main/config/Calibration.h` | Servo/ESC µs values (SAIL_PLUS/MINUS_US need bench calibration) |
| `main/core/Types.h` | ControlMode enum, RcFrame struct, ActuatorCommand struct |
| `main/drivers/AxpPower.h/.cpp` | AXP192 init: enables LDO2 (LoRa), LDO3 (GPS), DCDC1 (3.3V). Wire stays active. |
| `main/drivers/BatteryAdc.h/.cpp` | GPIO36 ADC1_CH0, R5=562kΩ/R6=120kΩ divider, 11dB attenuation |
| `main/drivers/RcReceiver.h/.cpp` | Interrupt-driven RC PWM reading, 4 channels (CH2/3/4/5), ISR-safe with portMUX |
| `main/drivers/McpwmActuators.h/.cpp` | MCPWM output for sail servo (GPIO2), rotor (GPIO25), ESC1 (GPIO15). Single ESC. |
| `main/control/ModeManager.h/.cpp` | Decodes CH5 → ControlMode (Failsafe / Sail / Manual / Automatic) |
| `main/control/ManualController.h/.cpp` | RC → ActuatorCommand: unified manual (binary sail + winch + CH3 inverse propeller). No arming. |
| `main/app/DroneApp.h/.cpp` | Orchestrator: 20/200/1000 ms ticks (control/debug/LoRa heartbeat) |
| `main/drivers/GpsUart.h/.cpp` | Serial1 TinyGPSPlus wrapper — position, sats-in-view, last NMEA line |
| `main/drivers/LoRaRadio.h/.cpp` | SX1276 driver — SPI HSPI 433 MHz, blocking TX, polling RX |
| `main/comm/LoRaComm.h/.cpp` | LoRa protocol — JSON heartbeat builder + command dispatcher |
| `main/navigation/Navigator.h/.cpp` | Haversine distance + bearing (stateless namespace) |
| `main/navigation/MissionPlan.h` | Up to 16 Waypoints + MissionMode |
| `main/navigation/MissionManager.h/.cpp` | Mission state machine (Idle/Running/Returning/Complete) |
| `transceiver/transceiver.ino` | Ground station sketch — shorthand CLI + raw JSON ↔ LoRa bridge |
| `test/CMakeLists.txt` | Native Linux build for logic unit tests (no hardware needed) |
| `test/stubs/Arduino.h` | Minimal Arduino type stub (uint8_t etc.) for host compilation |
| `test/test_runner.cpp` | ⚠️ STALE — does not build (predates `main/src/` reorg + single-ESC + Sail/Manual rename). Needs rewrite. |
| `docs/rapport_projet.tex` | French technical report (pdflatex, 29 pages) — project history, phases, problems/solutions |
| `docs/rapport_projet.pdf` | Compiled PDF — regenerate with `pdflatex -interaction=nonstopmode rapport_projet.tex` (run twice) |

### Note on Arduino IDE folder structure
The Arduino IDE only compiles `.cpp` files that are in the same folder as the `.ino` OR inside a `src/` subdirectory. All source subdirectories live under `main/src/`. All `#include` statements within `src/` use relative paths (`../sibling/` or `./samedir`) — the IDE does **not** add `src/` subdirectories to the include search path automatically.

---

## 10. Component Specifications

### Futaba S3003 (Sail aileron servo)
| Parameter | Value |
|---|---|
| PWM frequency | 50 Hz |
| Pulse range | 1000–2000 µs |
| Center (neutral) | **1520 µs** (Futaba standard, NOT 1500) |
| Dead band | 100 µs |
| Operating voltage | 4.8 V or 6.0 V |
| Stall torque | 3.2 kg·cm @ 4.8V / 4.1 kg·cm @ 6.0V |
| Speed | 0.23 s/60° @ 4.8V / 0.19 s/60° @ 6.0V |
| Dimensions | 40.4 × 19.8 × 36 mm |
| Weight | 37.2 g |
| Connector | Futaba "J" |
| Gear | Nylon/plastic |

**Note:** Center is 1520 µs. `Calibration.h` uses 1500 µs — needs adjustment on calibration.

### Graupner Regatta ECO II (Rotor/Safran servo — Model 5176)
| Parameter | Value |
|---|---|
| Type | **WINCH servo** (multi-turn, NOT positional) |
| PWM frequency | 50 Hz |
| Pulse range | 1000–2000 µs (standard RC) |
| Center (neutral / stop) | **1500 µs** |
| Travel range | **6 full rotations** (±3 turns from center) |
| Operating voltage | 4.8 – 7.2 V |
| Idle current | 300–350 mA |
| Load current | 2500–3300 mA (2.5–3.3 A!) |
| Torque | ~9.5 kg·cm @ 6V / ~10.9 kg·cm @ 7.4V |
| Speed | 0.60–0.72 s/360° @ 6V |
| Dimensions | 40.6 × 20 × 38.9 mm |
| Drum diameter | 25 mm |
| Weight | 56 g |
| Weather protection | Splash resistant (not waterproof) |
| Connector | JR standard |

**Critical note:** This is a **positional multi-turn winch servo**. PWM sets a target shaft position, not a speed. 1500 µs = mechanical center (servo holds there). 1000 µs = one travel extreme. 2000 µs = other travel extreme. It holds position under load like a standard servo. Unlike a continuous-rotation servo, it does NOT free-spin — it seeks and holds the commanded position within its 6-turn travel range. The PCB power path must support up to 3.3 A peak for this servo.

### Pro-Tronik PTR-6A Transmitter
| Parameter | Value |
|---|---|
| Channels | 6 (4 proportional + CH5 3-pos switch + CH6 2-pos switch) |
| Protocol | Proprietary Pro-Tronik FHSS 2.4 GHz |
| PWM neutral | 1500 µs |
| PWM range | 1000–2000 µs |
| PWM frequency | 50 Hz |
| TX voltage | 3.7–8.4 V (default 4.8V NiMH) |
| Autonomy | > 8–10 h |
| Model memories | 8 |
| Trims | 4 digital, auto-stored |
| Not compatible with | Futaba, Spektrum, FrSky, etc. |

### Pro-Tronik Receiver (R8X, bundled with PTR-6A V2)
| Parameter | Value |
|---|---|
| Model | R8X (8-channel) |
| Protocol | Pro-Tronik FHSS V2 2.4 GHz |
| Channels | 8 (channels 7–8 may output neutral when TX sends 6ch) |
| PWM neutral | 1500 µs |
| PWM range | 1000–2000 µs |
| PWM frequency | 50 Hz |
| Operating voltage | 4.8–8.4 V |
| PPM output | Available on BATT/PPM pin (hold BIND 5 s to toggle) |
| Range | ~2400 m |
| Weight | ~10 g |
| Failsafe | Built-in |

### Pro-Tronik ESC — Black Fet series (BF20A / BF30A / BF45A)
| Parameter | Value |
|---|---|
| Protocol | Standard RC PWM |
| PWM range | 1000–2000 µs |
| PWM center (idle/off) | 1500 µs (bidirectional) or 1000 µs (unidirectional) |
| BEC output | 5 V |
| BEC current | 2–4 A depending on model |
| LiPo cell count setting | **Must be set via EPRG-3 card** (cannot do it from TX) |
| Input voltage | 2S–4S (BF20A/30A) or 2S–6S (BF45A) |
| Cont. current (BF30A) | 30 A |
| Cont. current (BF45A) | 45 A |

**Arming:** Must calibrate endpoints (full throttle then idle at power-on) or use EPRG-3 card. In code, ESC arms after holding ≤1300 µs for 2 seconds. Note: PTR-6A stick minimum is ~1265 µs, so the old 1050 µs threshold was unreachable — updated in Calibration.h.

---

## 11. T-Beam V1.1 — Full Verified Pin Map

| Signal | GPIO | Notes |
|---|---|---|
| LoRa SCK | 5 | SPI HSPI |
| LoRa MOSI | 27 | SPI HSPI |
| LoRa MISO | 19 | SPI HSPI |
| LoRa CS/NSS | 18 | Active-low |
| **LoRa RESET** | **23** | ⚠️ Multiple sources confirm GPIO23, NOT GPIO14 |
| LoRa DIO0 (IRQ) | 26 | |
| LoRa DIO1 | 33 | |
| LoRa DIO2 | 32 | |
| GPS TX (board→GPS) | 12 | UART1 |
| GPS RX (GPS→board) | 34 | UART1, input-only |
| I2C SDA | 21 | AXP192 (0x34) + OLED (0x3C) — PCB V2: free, wired to expansion connector |
| I2C SCL | 22 | AXP192 + OLED — PCB V2: free, wired to expansion connector |
| User button | 38 | Pull-up, falling-edge |
| Blue LED | 14 | |
| Battery ADC | 35 | Input-only |

### AXP192 power rail assignments
| Rail | Peripheral | Voltage |
|---|---|---|
| DCDC1 | OLED / 3.3V I/O | 3300 mV |
| DCDC3 | ESP32 core | 3300 mV |
| LDO2 | LoRa SX1276 | 3000–3300 mV |
| LDO3 | GPS NEO-6M | 3300 mV |
| LDO1 | RTC backup | Always on |

**⚠️ CRITICAL: GPS does not work until AXP192 LDO3 is set to 3300 mV and enabled.** This must be done in code before initializing Serial1 for GPS.

**LoRa RST pin:** T-Beam V1.1 hardware uses GPIO23. In PCB V1, GPIO23 was shared with RC CH6, so RST was passed as -1. In PCB V2, CH6 is removed — GPIO23 is now free and LoRa RST is properly connected. `LoRaRadio.cpp` updated to pass RST=GPIO23.

---

## 12. Progress Log

| Date | What was done |
|---|---|
| 2026-04-27 | Initial session: full project read-through, architecture planning, this CLAUDE.md created |
| 2026-04-27 | Component research: full specs gathered for Futaba S3003, Regatta ECO II, Pro-Tronik PTR-6A, R8X receiver, Black Fet ESC, T-Beam V1.1 |
| 2026-04-27 | Confirmed Futaba S3003 center = 1520 µs (not 1500). Confirmed Regatta ECO II is a winch, not positional. Confirmed LoRa RST = GPIO23 on T-Beam V1.1. |
| 2026-04-27 | Phase 1 coded (10 files): main.ino, BoardConfig.h, Calibration.h, Types.h, AxpPower.h/.cpp, RcReceiver.h/.cpp, McpwmActuators.h/.cpp, ModeManager.h/.cpp, ManualController.h/.cpp, DroneApp.h/.cpp |
| 2026-04-27 | Bug fixes during coding: (1) RcReceiver.h — replaced explicit FreeRTOS includes with `<Arduino.h>` to avoid ESP32 core 3.x conflicts; (2) DroneApp — added 20 ms rate limit on controlTick() to avoid hammering ISR critical sections; (3) ActuatorCommand default sailUs corrected to 1520 |
| 2026-04-27 | Battery drain diagnosis: GPIO35 reads ~20 mA from battery continuously via onboard voltage divider (~370 Ω total). This is a hardware issue, not a code bug. GPIO35 is input-only and cannot source current — the drain is from the resistor divider itself. Recommended fix: replace divider with 100 kΩ + 160 kΩ OR switch to AXP192 internal ADC (preferred). Decision: use AXP192 ADC, do not read GPIO35. |
| 2026-04-27 | Native unit test harness built and run (test/): 59 tests, 1 bug found and fixed. Bug: ManualController::update() did not call disarm() when entering ManualServo mode, allowing the ESC to stay armed across a mode switch — pilot could switch back to ManualProp and drive motors without re-arming. Fix: added disarm() call in the ManualServo branch of update(). All 59 tests now pass. |
| 2026-04-28 | Edge-case test round (47 new tests, 106 total): all pass. Key findings: (1) sail/rotor deadband inclusive boundaries confirmed correct; (2) uint32_t arming timer overflow at ~49 days handled correctly by unsigned arithmetic; (3) armStartMs_=0 sentinel defers countdown by one tick when nowMs=0 at boot — harmless in practice since millis()>100ms before first control tick; (4) re-arm after mode switch confirmed to require full 2s hold; (5) differential left/right steer is perfectly symmetric. No new bugs found. |
| 2026-04-28 | Arduino IDE compile fix: all source subdirectories moved into `main/src/` so the Arduino IDE build system picks them up. All `#include` paths updated to use relative paths (`../`) since the IDE only adds the sketch directory to the include search path, not `src/` subdirectories. Confirmed clean compile via arduino-cli: 320 KB flash (24%), 24 KB RAM (7%). |
| 2026-04-28 | Phase 2 — GPS: GpsUart driver (TinyGPSPlus), GpsPosition struct, Navigator (haversine), MissionPlan, MissionManager state machine. DroneApp updated with gps_.update() every loop, mission integration in controlTick(), GPS + mission debug lines. |
| 2026-04-28 | GPS hardware diagnostic: confirmed no fix indoors (expected). Outdoors, 9 satellites in view but ephemeris download in progress (cold start without LiPo). Root cause: without battery, every USB power cycle loses GPS almanac. Fix: install LiPo battery. Added TinyGPSCustom gsvTotal_ for satsInView(), two-buffer NMEA line capture (lineBuf_ + completedLine_ with memcpy), visible= counter in NO FIX debug line. |
| 2026-05-04 | Phase 3 — LoRa: LoRaRadio driver (SPI HSPI, 433 MHz, RST=-1), LoRaComm protocol layer (JSON heartbeat TX 1 Hz, command dispatcher: navigate/stop/home/waypoints/wind-command/wind-observation/restart). DroneApp updated with loraHbTick() at 1 s, [LORA] debug line. Serial monitoring via pyserial venv: confirmed [LORA] OK at boot and tx counter incrementing at 1 Hz. Blocking TX ≈450 ms per packet noted — acceptable for Phase 3. |
| 2026-05-04 | Phase 3 — Transceiver: created `transceiver/transceiver.ino` ground station sketch. Shorthand CLI commands (navigate, stop, restart, wind-obs, wind \<deg\>, home \<lat,lon\>, wpt \<lat,lon,...\>) + raw JSON passthrough. Compiled and hardware-tested on T-Beam: [LORA] OK, 433 MHz, listening. Both sketches now at 359 KB / 302 KB flash respectively. |
| 2026-05-05 | Phase 3 — Full uplink verified: added DEBUG_MODE flag (`DebugConfig.h`), debug prints in `LoRaRadio::poll()` and `LoRaComm::dispatch()`, `rxDetected_` counter. Created `lora_uplink_test.py` (5-burst per command strategy). All 7 commands pass: navigate ✓ stop ✓ home ✓ waypoints ✓ wind-command ✓ wind-obs ✓ restart ✓. RSSI −101 to −106 dBm at 5 cm range. Timing collision explained and mitigated by burst strategy. Phase 3 COMPLETE. |
| 2026-05-20 | PWM2 / Rotor servo fix — Regatta ECO II on GPIO25 (PWM_2) was not responding. Root cause: `McpwmActuators::begin()` called `mcpwm_set_duty_type(..., MCPWM_OPR_B, MCPWM_DUTY_MODE_1)` on the rotor channel under the assumption that the BC548 NPN level-shifter inverts the signal and the MCPWM output needed pre-inversion. But the sail servo on the same type of NPN circuit works with DUTY_MODE_0 (default), proving MODE_1 was double-inverting the rotor signal into an unusable waveform. Fix: removed the `mcpwm_set_duty_type` call entirely — both OPR_A (sail) and OPR_B (rotor) now use DUTY_MODE_0. Also corrected Calibration.h and ManualController.cpp: Regatta ECO II is a positional multi-turn servo (PWM = target position, not speed), ROTOR_CENTER_US=1500 is the neutral/hold position. Rotor block in computeServoMode() indentation fixed. |
| 2026-05-20 | GPS external antenna debug — Taoglas ADFGP.25A connected to T-Beam u.FL, no fix outdoors. Diagnosed via serial: `chars=10` frozen (vs thousands expected) = GPS outputting zero NMEA. Root cause: a prior session had saved a CFG-PRT config to the NEO-6M's internal flash that disabled NMEA output. GPS was alive (ACKed UBX commands — those 10 bytes = one UBX ACK-ACK frame) but silent. Fix 1: UBX CFG-CFG factory reset (clearMask=0x1F, loadMask=0x1F, deviceMask=0x17) wipes the saved flash config and restores ROM defaults including NMEA at 9600 baud. Fix 2: UBX CFG-ANT (flags=0x001B) enables the antenna supervisor: provides DC bias voltage through coax to power the Taoglas active LNA, and asserts ANT_FLAG to switch the T-Beam RF switch from internal ceramic patch to external u.FL. Both commands sent in GpsUart::begin() on every boot. Result: NMEA restored, fix acquired outdoors in <60 s, 7 satellites, hdop=2.2, position 48.360476 −4.566822 (Brest area). |
| 2026-05-24 | PCB V2 hardware adaptation — all 13 source files updated to match final PCB V2 pin mapping: CH2=GPIO39, CH3=GPIO14, CH4=GPIO13, CH5=GPIO4, ESC1=GPIO15, BatADC=GPIO36, LoRa RST=GPIO23. CH6 and ESC2 removed entirely. Wire.end() removed from AxpPower (I2C stays persistent, GPIO21/22 free for sensors). BatteryAdc driver added (R5=562kΩ/R6=120kΩ, GPIO36, 11dB attenuation). ESC_ARM_MAX_US raised to 1300 µs (PTR-6A stick minimum ≈1265 µs). Clean compile: 27% flash, 7% RAM. |
| 2026-05-24 | Technical report — created `docs/rapport_projet.tex` (29 pages, French, pdflatex). Covers all phases, problems and solutions, hardware specs, PCB V1/V2 changes, code architecture, development journal. Compiled to `rapport_projet.pdf`. |
| 2026-06-03 | IHM read-through — `IHM_Facu_essay_Version/` documented: FastAPI + MongoDB + serial_link.py stack. Added `stop_ihm.sh` to kill all IHM processes quickly. |
| 2026-06-03 | DebugConfig.h rewrite — master switch `DEBUG_ENABLED` + 9 per-section flags (DEBUG_RADIO off by default). All source files migrated from generic `DBG("TAG",...)` to typed macros `DBG_RADIO()`, `DBG_GPS()`, etc. Raw `Serial.printf("[LORA]")` in LoRaComm/DroneApp also migrated. Boot-time `[LORA] OK/ERROR` prints kept unconditional. |
| 2026-06-03 | Rotor servo range limited to ±180° — Regatta ECO II full range (1000–2000 µs = ±1080°) too difficult to control manually. New values: ROTOR_MIN_US=1417, ROTOR_MAX_US=1583 (±83 µs from center = ±180°, derived from 2.16°/µs). CH4 real travel (1180–1790 µs) added as CH4_MIN_US/CH4_MAX_US in BoardConfig.h. ManualController updated to map CH4 range to new rotor range. |
| 2026-06-03 | T-Beam power issue diagnosed — PCB V2 feeds T-Beam via +5V expansion header pin (Pololu 2866 output). Pin is on USB VBUS path after protection diode, so AXP192 doesn't always start reliably without LiPo on JST. Workaround: cut USB-A→USB-C cable, solder USB-A end to Pololu +5V/GND, plug USB-C into T-Beam. Boot via USB-C is the correct AXP192 VBUS input. |
| 2026-06-04 | Autonomous-sailing prep for general test (4 fixes). (1) **Rotor range corrected to ±90°** (was mislabeled ±180° in Calibration.h/heartbeat): ROTOR_MIN/MAX=1417/1583 = physical ±90°, added ROTOR_RANGE_DEG=90. (2) **AutoController rudder mapping fixed** — was 25 µs/° (assumed full 1000–2000 µs range) causing saturation past ~3.3°; now maps nav rudder ±NAV_RUDDER_LIMIT_DEG (±20°) onto the full ±83 µs travel (≈4.15 µs/°). (3) **Persistent nav state** — AutoController now keeps rudderAngle_/sailAngle_ as members instead of reconstructing from clamped µs each tick (the lofer/abattre integrator depends on it); compute() signature dropped the currentSailUs/currentRotorUs params, DroneApp call site updated. (4) **Wind handling** — windValid_ flag (set by wind-command OR estimation); navigation holds neutral until wind known. wind-observation command now wired to AutoController::beginWindObservation()/observeWind(): sails fixed +10° tack, circular-EMA smooths GPS course, after WIND_OBS_DISTANCE_M=30 m latches wind = smoothHeading+90° via nav_handleWindObservation(). Heartbeat rotorDeg now -90..+90. Compiles 368 KB (28%) / 25 KB (7%). navigation.h logic untouched. |
| 2026-06-04 | Auto rudder range reduced to ±20°. First fix mapped nav rudder ±20° onto the full ±90° winch (±83 µs) — too aggressive on the bench. Reverted to a gentle envelope: nav rudder degrees now map **1:1 to physical winch degrees**, clamped to `ROTOR_AUTO_RANGE_DEG`=20° (new Calibration.h constant). usPerDeg = (ROTOR_MAX−CENTER)/ROTOR_RANGE_DEG = 83/90 ≈ 0.92 µs/° → auto winch travel only 1482–1518 µs. Manual mode keeps full ±90°. Flashed to boat (chip ESP32-D0WDQ6-V3, ttyUSB1). |
| 2026-06-04 | IHM session: launched full stack on real hardware — confirmed end-to-end LoRa link (boat heartbeats → transceiver → serial_link → Mongo → web map, bat 7.7 V, GPS fix in Brest). Added burst retransmit (3×, 0.35 s) in serial_link.py to beat LoRa half-duplex collisions (commands are idempotent). Diagnosed wind-observation: command reaches boat but observation is invisible in telemetry until it completes after 30 m travel. Transceiver USB port alternates ttyUSB0/ttyUSB1 on reconnect — pass explicit port to start_ihm.sh. |
| 2026-06-04 | Heartbeat servos now report real degrees: sail = ±10° (binary, from sailUs vs SAIL_CENTER), rudder = winch degrees (linear from rotorUs). sendHeartbeat() gained sailUs/rotorUs params; DroneApp passes lastCommand_. Was hardcoded `{"sail":0,"rudder":0}`. |
| 2026-06-10 | **IHM diagnostics — 4 stages.** (1) Front-only health panel (Liaison/GPS/Batterie/RC/Contrôle semaphores) + Sim/Réel toggle (hides sim-only buttons) + staleness detection by timestamp delta; lost link greys the other rows yellow with a French "non fiable" warning. (2) Backend endpoints `/api/set-mode`, `/api/reconnect`, `/api/reset-transceiver` (ESP32 hard-reset via DTR/RTS), `/api/stop`; `launch_process(env=)`; `/start` now launches serial_link in sim mode explicitly. `.env`→local Mongo, `docker-compose.yml` added. (3) Firmware heartbeat: added `fix/sat/hdop/rc`, **removed `control_mode`** (redundant with mode, frees 28 B). (4) Transceiver injects ground-measured `rssi` into each forwarded heartbeat. Liaison row shows dBm. Verified end-to-end: rssi −102 dBm, packet 235 B. Boat + transceiver reflashed. |
| 2026-06-10 | **Unified manual mode.** CH5 redistributed: low=Automatic (unchanged), middle=**Sail** (inert — neutral outputs), high=**Manual**. Manual = servos exactly as before (CH2 sail, CH4 winch) **plus** propeller on CH3, **inverse** (1100 µs=100 %, 1990 µs=0 %), forced 0 % below 10 % power; CH3=0 (lost) → ESC stop. Enum renamed ManualServo→Sail, ManualProp→Manual across Types/ModeManager/DroneApp/LoRaComm/ManualController. **ESC arming gesture removed** (incompatible with inverse throttle). ManualController rewritten (single computeManual, no arming/disarm/toSigned1000). Compiles 368 KB; flashed to boat (ttyUSB1, needed BOOT-mode retry). |
| 2026-06-10 | Linker fix (pre-existing WIP): `AutoController::computeAutoPropulsionUs` was declared as a static member but defined as a file-local `static` free function → undefined reference. Qualified the definition with `AutoController::`. |
| 2026-06-10 | Wind-observation progress bar. Firmware: `AutoController::windObsProgressPct()` (travelled/30 m), heartbeat gained a **conditional** `wobs` field (present only while measuring). To stay under 255 B with it, trimmed the heartbeat: `heading` → integer, `location` → 5 decimals, `waypoints:{total,current}` → flat `wt`/`wc`. IHM: progress section "Mesure du vent" + parser updated to `wt`/`wc`. Boat reflashed; verified ~210 B typical / ~232 B worst. |
| 2026-06-10 | IHM diagnostics v2 (frontend-only). (1) **Wind compass** top-left of map — arrow points to wind source (`wind` = from), greys when no data; Leaflet zoom moved to top-right. (2) **Trajectory prediction** — `nav.js` is a faithful JS port of `navigation.h` (upwind/downwind zigzag, corridor cross-track, empannage, lofer/abattre) + a simple kinematic simulator (heading follows rudder, constant speed — path is approximate, steering logic exact). Toggle button "🧭 Trajectoire" draws a dashed polyline boat→current-waypoint. No reflash. |
| 2026-06-10 | USB auto-detection by serial. Both boards are Silicon Labs **CP2104** with unique factory serials. `config.py` resolves the transceiver port from `/dev/serial/by-id/` by `TRANSCEIVER_SERIAL` (no udev/root needed); `serial_link.py` re-resolves on every (re)connect and reconnects on disconnect/USB re-enumeration (opens with dsrdtr/rtscts=False to avoid resetting the ESP32). Added `99-autoboat.rules` (optional fixed `/dev/ttyAUTOBOAT_TRX`/`_BOAT`). **Serials: transceiver=01C00B54, boat=02126CF1** (initially had them swapped — corrected after reading both boards). |
| 2026-06-10 | start/stop scripts rewritten for reliability. `stop_ihm.sh`: SIGTERM→SIGKILL escalation, frees port 5000 (kills the uvicorn reload worker that kept the webserver alive), `docker compose down --remove-orphans` + `docker rm -f` (mongo no longer survives via restart:unless-stopped), verifies each component. `start_ihm.sh`: runs stop first (clean slate), truncates logs, launches Python with `-u` (live logs). Tested end-to-end: stop → 0 procs/port free/mongo down; start → all up, transceiver auto-detected on ttyUSB1, data flowing. |
| 2026-06-10 | Hardware finding: the **boat's USB connection flaps** (kernel `cp210x ... status -19`, disconnect→reattach with a new ttyUSBn) — caused by the boat running on USB only with a dead battery (0.81 V) → brownouts drop the bus. Not a software issue; the serial_link reconnect-by-serial logic tolerates it, but stable telemetry needs the boat properly powered (LiPo). |
| 2026-06-10 | **Navigation upgraded to colleague's `Testautoboat` branch** (DeltaGod/PER_MEA_LYINCROYAP): new `navigation.h` (smarter initial tack via `nav_sideMovingTowardWaypoint`, passed-waypoint-plane arrival via `nav_hasPassedWaypoint`, corridor 20→100 m) + `NavigationConfig.h`/`NavigationSelector.h`/`oldNavigation.h` compile-time switch. Public entry points unchanged → drop-in; AutoController include retargeted to the selector. Only navigation ported — Facundo's AutoController/DroneApp/LoRaComm/ManualController kept. Compiles 28 %/7 %. |
| 2026-06-10 | **Manual propeller → BIDIRECTIONAL** (±100 %). ESC neutral moved 1000→**1500 µs** (ESC_NEUTRAL_US, =ESC_STOP_US): CH3 ratchet maps CH3_FULL(1100)=+100 % fwd / CH3_CENTER(1545)=stop (±40 µs deadband) / CH3_ZERO(1990)=−100 % rev. Coherence updates: ActuatorCommand default esc1Us 1000→1500, McpwmActuators init 1500 + write clamp lower bound → ESC_REVERSE_MIN_US(1000), AUTO_ESC_* shifted to forward half (1600/1700/1850). ⚠️ Reverse needs the ESC in bidirectional mode (EPRG-3 card) — not yet confirmed done. Compiles 28 %/7 %. |
| 2026-06-10 | Diagnosis (not a code bug): motor running **in pulses** at ~7 V is the ESC's **low-voltage cutoff** tripping under load (2S at 3.5 V/cell sags below LVC; worse if EPRG-3 cell-count is mis-set). Fix = full LiPo charge + program ESC cell count/LVC. Also: flashing the boat failed repeatedly — same brownout (chip stops mid-write, CP2104 re-enumerates); needs LiPo / manual BOOT-button entry. |

---

## 13. Notes on Key Algorithms

### ManualController — unified Manual mode (CH5 high)
One mode (`computeManual`) drives servos **and** propeller together. Entered only when CH5 > 1800.
- **CH2 → sail (binary):** center deadband ±35 µs. First frame initializes sail state from stick position to avoid snap on mode entry. Inside deadband: hold last state.
- **CH4 → rotor (positional):** CH4 measured range 1180–1790 µs maps to ROTOR_MIN_US–ROTOR_MAX_US (1417–1583 µs = ±90°). Deadband ±35 µs around center snaps to ROTOR_CENTER_US (1500). The Regatta ECO II holds its position.
- **CH3 → propeller (BIDIRECTIONAL ratchet, 2026-06-10):** `diff = CH3 − CH3_CENTER_US(1545)`. `|diff| ≤ ESC_CENTER_DEADBAND_US(40) → ESC_NEUTRAL_US(1500)`. Forward side (`diff<0`, toward CH3_FULL_US=1100): `frac=(center−CH3)/(center−1100)` → `esc1Us = 1500 + frac×(2000−1500)`. Reverse side (`diff>0`, toward CH3_ZERO_US=1990): `frac=(CH3−center)/(1990−center)` → `esc1Us = 1500 − frac×(1500−1000)`. Safety: **CH3 == 0 (lost) → ESC_NEUTRAL_US**. ⚠️ Reverse requires the ESC in bidirectional mode (EPRG-3). Replaces the old inverse 0–100 % throttle + 10 % low-deadzone.
- **Channel-loss guard:** if CH2 or CH4 read 0, `update()` resets sail state and returns safe neutral (ActuatorCommand defaults — esc1Us now 1500).

### Sail mode (CH5 middle) — inert
Does nothing: DroneApp outputs `ActuatorCommand{}` defaults (sail 1520, rotor 1500, **esc 1500 = neutral**) and resets the manual controller. Out-of-band CH5 values also fall back to Sail.

### ESC arming — REMOVED (2026-06-10)
The old "hold CH3 ≤1300 µs for 2 s" gesture was dropped: it is incompatible with the new inverse throttle (idle is now CH3 high ≈1990, not low) and was not requested. The 10 % low-deadzone + the actuator slew limiter are the safety. The ESC still arms via its own firmware (sees idle at power-on if the boat boots with CH3 at 0 %). To re-add a startup interlock, gate the throttle until CH3 has been seen at 0 % once after entering Manual.

### Future WinchTracker (Phase 6 prerequisite)
The Regatta ECO II has no position feedback. For autonomous mode a software estimator is needed:
- Track `rotorUs` deviations from 1500 µs over time
- Accumulate signed position estimate (speed × elapsed ms integration)
- Use to infer when sheet is near limits and when to reverse

---

## 14. Bench Test Procedure — Phase 1 (Manual RC)

Before water testing, verify each subsystem in order:

1. **AXP192 boot** — Power on, check serial for `[AXP]  OK`. If `WARN`, check I2C at GPIO21/22 for short.
2. **RC signal** — Power on RC transmitter first, then drone. Serial should show CH2–CH5 updating within ~2 s of RC receiver binding. All channels should read 1400–1600 µs at stick centers.
3. **Mode switching** — Move CH5 3-pos switch: verify `[FAILSAFE]` → `[SAIL    ]` (inert) → `[MANUAL  ]` transitions in serial output.
4. **Sail servo (Manual mode)** — Push CH2 stick past ±35 µs deadband. Verify sail servo snaps between two positions. Measure actual angle — adjust `SAIL_PLUS_US`/`SAIL_MINUS_US` in `Calibration.h` until physical ±10° is confirmed.
5. **Rotor winch (Manual mode)** — Move CH4. Verify winch rotates in both directions and stops cleanly at 1500 µs.
6. **Propeller (Manual mode, propeller disconnected)** — With CH3 high (≈1990) verify `esc1`=1000 (0 %). Lower CH3 toward 1100: `esc1` rises (inverse). Confirm that above ~90 % stick (≈10 % power) it snaps from 0 to active, i.e. below 10 % → `esc1`=1000.
7. **Sail-position inert check** — CH5 to middle (SAIL): verify all actuators hold neutral (sail=1520, rotor=1500, esc1=1000) regardless of sticks.
8. **Failsafe** — Turn off transmitter while in MANUAL mode. Within 100 ms all channels read 0 and mode switches to `[FAILSAFE]`; actuators go to safe defaults.

---

## 15. Known Hardware Issues & Conflicts

### GPIO23 — LoRa RST vs RC CH6
- T-Beam V1.1 hardware routes LoRa SX1276 RESET to **GPIO23**
- PCB V2 also routes **RC CH6** to GPIO23
- **Resolution (Phase 3):** LoRa driver uses RST=-1 (skip hardware reset). The SX1276 initializes reliably without it. RC CH6 ISR remains attached but CH6 is unused in any logic. No conflict in practice.

### ESC LiPo cell count (EPRG-3 card required)
- Pro-Tronik Black Fet ESCs cannot have cell count programmed from the transmitter
- **Must use the EPRG-3 programming card** before connecting a LiPo pack
- Connecting wrong-voltage pack without correct cell count setting risks ESC damage or incorrect LVC cutoff
- Do this before any powered ESC test

### Battery voltage divider drain (GPIO35) — RESOLVED in PCB V2
- The PCB V1 on-board voltage divider on GPIO35 used low-value resistors (~370 Ω total) → 20 mA continuous drain.
- **PCB V2 resolution:** New divider on GPIO36 uses R5=560kΩ / R6=120kΩ → drain ≈ 10.9 µA at 7.4 V (acceptable). BatteryAdc.cpp reads GPIO36 ADC1_CH0 with 11 dB attenuation. Do NOT read GPIO35 — it was replaced by GPIO36 and the old divider is harmful.

### LoRa RST pin — RESOLVED in PCB V2
- T-Beam V1.1 hardware: LoRa RST = GPIO23. Post_GPT incorrectly used GPIO14.
- PCB V1 shared GPIO23 with RC CH6, so RST was passed as -1 (skipped).
- PCB V2 removes CH6 entirely → GPIO23 is free → LoRa RST properly enabled.
- `LoRaRadio.cpp` now passes `RST=BoardConfig::LORA_RST_PIN` (GPIO23).

### GPS NEO-6M — saved flash config can disable NMEA output
- The NEO-6M has its own internal flash that survives power-off and battery removal.
- A previous session using Arduino IDE serial monitor, u-center, or any tool that sends CFG-PRT/CFG-MSG can save a config to that flash that disables NMEA output.
- **Symptom:** `chars` counter stuck at exactly 10 (= one 10-byte UBX ACK-ACK frame). GPS is powered and responds to UBX commands but produces zero NMEA sentences.
- **Diagnosis:** `chars=10` frozen vs chars climbing at ~180 bytes/s when healthy.
- **Fix in code (GpsUart::begin):** Sends UBX CFG-CFG factory reset on every boot (clearMask=0x1F, loadMask=0x1F, deviceMask=0x17) to wipe the saved flash config and reload ROM defaults. Followed by CFG-ANT for active antenna. This makes the GPS immune to stale flash configs across sessions.

### GPS NEO-6M — active external antenna requires software enable (CFG-ANT)
- Antenna used: **Taoglas ADFGP.25A** (active patch, u.FL connector, fits T-Beam directly).
- The NEO-6M antenna supervisor is OFF by default (`flags=0x0000`). Without enabling it:
  1. No DC bias voltage is provided through the coax → the Taoglas LNA has no power → receives nothing.
  2. The T-Beam RF switch (`ANT_FLAG` output of NEO-6M) stays on the internal ceramic patch → the u.FL connector is electrically bypassed even when physically connected.
- **Fix:** UBX CFG-ANT with `flags=0x001B` (svcs=1, scd=1, pdwnOnSCD=1, recovery=1) sent in GpsUart::begin() after the factory reset.
- **Result confirmed:** 7 satellites, hdop=2.2, fix in <60 s in open sky.
- **Note:** GPS does not work under roofs or heavy obstructions even with a good antenna. `chars` will increment (NMEA flowing) but `visible` will stay 0. Move to open sky before diagnosing antenna issues.

### GPIO32/33 — ESC outputs vs LoRa DIO1/DIO2 — RESOLVED
- T-Beam V1.1: GPIO32 = LoRa DIO1, GPIO33 = LoRa DIO2
- PCB V1 used GPIO32/33 for ESC1/ESC2. PCB V2 uses GPIO15 for the single ESC → GPIO32/33 are now free.
- LoRa library uses only DIO0 (GPIO26) for polling mode. DIO1/DIO2 not needed. No conflict in any PCB version.

---

## 16. Development Environment

### Arduino IDE
- **AppImage:** `/home/facundo/Applications/arduino-ide_2.3.8_Linux_64bit.AppImage`
- Used for: writing code, uploading firmware, serial monitor
- Board: `esp32:esp32:t-beam` (ESP32 core 3.3.8, already installed)
- Libraries installed (all in `~/.arduino15/` user libraries): AXP202X_Library 1.1.2, TinyGPSPlus 1.0.3, LoRa 0.8.0, ESP32Servo 3.2.0

### arduino-cli (command-line compiler)
- **Binary:** `/home/facundo/bin/arduino-cli` (v1.4.1)
- Used by Claude to verify compilation without opening the IDE
- Compile command:
  ```
  ~/bin/arduino-cli compile --fqbn esp32:esp32:t-beam /home/facundo/Proyectos/PER_Dron_a_voile/Informatica/Post-Claude/main
  ```
- Last successful compile (drone main): **368 KB flash (28%), 25 KB RAM (7%)**
- Last successful compile (transceiver): **328 KB flash (25%), 23 KB RAM (7%)**
- Upload: `~/bin/arduino-cli upload --fqbn esp32:esp32:t-beam --port /dev/ttyUSBx <sketch>` — if "Wrong boot mode (0xb)", retry once (auto-reset is flaky) or hold BOOT while resetting.

---

## 17. References

- T-Beam V1.1 hardware: https://github.com/Xinyuan-LilyGO/LilyGo-LoRa-Series
- AXP192 library: `Informatica/Arduino/libraries/AXP202X_Library/`
- TinyGPSPlus library: `Informatica/Arduino/libraries/TinyGPSPlus/`
- LoRa library: `Informatica/Arduino/libraries/LoRa/`
- ESP32Servo library (NOT to be used for main actuators): `Informatica/Arduino/libraries/ESP32Servo/`
- Post_GPT report (PDF): `Informatica/Post_GPT/Rapport de réécriture logicielle du drone de surface basé sur TTGO T-Beam.pdf`
- PCB schematic: `Electronica/Kicad_9_0/PCB_Electronica/PCB_Electronica_V2.kicad_sch`
