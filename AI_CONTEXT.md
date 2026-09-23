# Sail Drone (Dron à Voile) — Complete Project Context

> **Purpose of this file:** give any AI (or new developer) instant, complete context about this project. It is self-contained: hardware, firmware architecture, every design decision, every problem encountered and its solution, the ground-station software, build/flash procedures, and open work. If you are an AI assistant picking this project up, read this top to bottom first.

---

## 0. TL;DR (read this first)

- **What it is:** an autonomous **wind-powered surface drone** (a small sailing boat) built on a **LilyGO T-Beam V1.1** (ESP32). Its mission is **autonomous marine data acquisition**: sail to GPS waypoints, collect sensor data, and send telemetry over **LoRa 433 MHz** to a ground station.
- **It is a school research project (PER)** in France. Documentation exists in **French and Spanish**; code comments mix **English/French/Spanish**. Communicate with the author in **English**.
- **Propulsion is a rigid wing-profile sail** that freely weathervanes (~45° to the hull). Steering is done with **two actuators**: a binary ±10° aileron (chooses tack) and a multi-turn **winch "rudder"** (heading). A **single propeller/ESC** is the no-wind backup.
- **There is NO wind sensor and NO rudder/sail position feedback.** Wind direction is *estimated* from the GPS track. This is the single biggest limitation of the design.
- **Firmware status:** Manual control, GPS/navigation, and LoRa telemetry are **done and hardware-tested**. Wind estimation and autonomous sail navigation are **coded but never tested on water**. External sensors, data logging, and autonomous propulsion are **not implemented**.
- **The firmware was fully rewritten** ("Post-Claude" rewrite) from a defective first-group codebase. The #1 reason for the rewrite: the original used `ESP32Servo` (LEDC), which **breaks interrupt-based RC reading**. The rewrite uses **MCPWM** instead.
- **Two T-Beams are used:** one on the boat (the "drone"), one on the desk (the "transceiver"/ground radio). The transceiver bridges USB ↔ LoRa to a web UI ("IHM").
- **Hardware gotchas that bite everyone:** the T-Beam can only be powered from its battery/USB (not the header pins) → a USB cable was soldered to the PCB 5 V rail as a patch; powering on USB-only browns out (motor "pulses", flashing fails). **Use a charged LiPo.**

---

## 1. Project identity & people

| | |
|---|---|
| **Project** | PER — autonomous sailing surface drone for oceanographic data acquisition |
| **Board** | LilyGO T-Beam V1.1 (ESP32 WROVER + SX1276 LoRa + u-blox NEO-6M GPS + AXP192 PMIC) |
| **Language/target** | Arduino C++, ESP32 Arduino Core 3.x |
| **Original code author** | Mohamed EL JILY (first group — `boat.ino`, now considered defective/legacy) |
| **Current firmware author** | Facundo Arito (engineering student; designed the KiCad PCB V2; did the full firmware rewrite) |
| **Git remote** | `https://github.com/DeltaGod/PER_MEA_LYINCROYAP.git` (branch `main`) |
| **Test location** | Brest, France (≈ 48.36° N, −4.56° W) |
| **Languages in repo** | Reports in French & Spanish; code comments mixed EN/FR/ES |

---

## 2. Physical concept (how the boat actually works)

The boat is propelled by a **freely-rotating wing-profile sail** on a vertical axis. Because of its aerodynamic shape, the sail self-aligns so the wind always arrives ~45° from the hull centerline — **no active sail-angle control is needed** for basic propulsion.

Steering/heading uses **two actuators working together**:

| Actuator | Component | Role | Behaviour |
|---|---|---|---|
| **Sail aileron** | Futaba S3003 servo | Small flap on the sail trailing edge | **Binary ±10° only.** Chooses which side the sail pushes → selects the tack (port/starboard). Modeled in code as a sign: `sailState = +1` or `−1`. Only the two extreme positions have physical meaning. |
| **Rudder / "safran" winch** | Graupner Regatta ECO II (mod. 5176) | Primary heading (yaw) actuator | **Positional multi-turn winch servo:** PWM sets a *target shaft position*, not a speed. 1500 µs = mechanical center; full travel = 6 turns; holds position under load. |
| **Propeller** | Single ESC (Pro-Tronik Black Fet) | No-wind backup propulsion | One ESC only (the original dual-ESC differential thrust was removed in PCB V2). |

**No wind sensor.** Wind direction is **inferred from the GPS track** over time (see §10). There is also **no position feedback** on the winch rudder, so the firmware never truly knows the real rudder/sail angle — it only knows the commanded value.

**Control summary:**

| You want to… | You do… |
|---|---|
| Change which way the sail pushes (tack) | Toggle the aileron ±10° (RC channel CH2) |
| Change heading / yaw | Move the rudder winch (CH4) |
| Run the propeller (no wind) | CH3 throttle (bidirectional; manual mode only) |
| Switch operating mode | CH5 three-position switch |

---

## 3. Hardware

### 3.1 Main components

| Component | Details |
|---|---|
| **LilyGO T-Beam V1.1** | ESP32-D0WDQ6-V3 (WROVER), SX1276 LoRa (433 MHz here), u-blox NEO-6M GPS, AXP192 power-management IC, 18650/JST LiPo holder, USB-C |
| **Sail servo** | Futaba S3003 — center **1520 µs** (not 1500!), used ±10° binary |
| **Rudder winch** | Graupner Regatta ECO II 5176 — winch servo, 1500 µs center, 6 turns travel, idle 300–350 mA, **load up to 2.5–3.3 A** |
| **ESC** | Pro-Tronik Black Fet (BF20A/30A/45A). LiPo cell count + LVC **must be set with the EPRG-3 card** (cannot be set from the TX). Supports bidirectional mode. |
| **RC TX** | Pro-Tronik PTR-6A — 6 ch FHSS 2.4 GHz, PWM 1000–2000 µs, 50 Hz. **Throttle stick min is ≈1265 µs, not 1000.** |
| **RC RX** | Pro-Tronik R8X (8 ch), built-in failsafe |
| **GPS antenna** | Taoglas active, **u.FL** connector |
| **PCB** | Custom KiCad 9 interface board; V1 (defective) and V2 (current) |
| **Battery** | 2S LiPo (was running ~7 V; see LVC issue in §13). Group's fix: more capacity / higher voltage. |

### 3.2 Full pin map — PCB V2 (current, authoritative = `main/src/config/BoardConfig.h`)

| Signal | GPIO | Notes |
|---|---|---|
| RC CH2 (sail toggle) | 39 | SVN, input-only |
| RC CH3 (throttle) | 14 | freed (no longer shares I2C) |
| RC CH4 (rudder winch) | 13 | freed |
| RC CH5 (mode selector) | 4 | 3-position switch |
| Sail servo PWM out | 2 | via BC548 NPN level shifter |
| Rudder winch PWM out | 25 | via BC548 NPN level shifter |
| ESC1 PWM out | 15 | **single ESC** (ESC2 removed) |
| Battery ADC | 36 | SVP, ADC1_CH0, input-only; external divider R5=562k/R6=120k |
| LoRa SCK / MISO / MOSI / CS | 5 / 19 / 27 / 18 | SPI **HSPI** |
| **LoRa RST** | **23** | hardware reset (correct pin; LilyGO docs wrongly say 14). Free now that CH6 was removed. |
| LoRa IRQ (DIO0) | 26 | polling used |
| GPS RX / TX (Serial1) | 34 / 12 | 9600 baud |
| I2C SDA / SCL | 21 / 22 | AXP192 internal **+ expansion connector** (FREE for future sensors) |

**Do NOT use GPIO35** for anything: the T-Beam has a low-impedance (~370 Ω) divider permanently soldered to it that drains ~20 mA continuously. Battery sensing is on GPIO36 instead.

**Avoided conflicts (resolved in PCB V2):** GPIO21/22 are now dedicated to I2C (RC moved off them); GPIO23 is free for LoRa RST (CH6 removed); GPIO32/33 (LoRa DIO1/2) are unused by the LoRa lib (DIO0-only polling), so they don't conflict.

### 3.3 PCB V2 hardware notes

- **3.3 V → 5 V level shifting:** PWM outputs go through **NPN BC548** transistors (common-emitter). The transistor *inverts*, so MCPWM must use **`DUTY_MODE_0` (active-high) on BOTH channels** — using `DUTY_MODE_1` double-inverts and kills the signal (this was a real bug, see §13).
- **Battery divider:** R5 = 562 kΩ (high side; schematic labels it "560K" but the installed E96 part is **562 kΩ** — code correctly uses 562000.0f, do NOT "fix" to 560k), R6 = 120 kΩ → ratio (562k+120k)/120k = **5.683**, 11 dB ADC attenuation. Drain ≈ 10.9 µA at 7.4 V.
- **5 V rail:** Pololu 2866 step-down regulator from the battery.
- **I2C persistent:** `Wire.end()` was removed from AXP power init so the I2C bus stays live for future sensors on the expansion connector.

---

## 4. Repository layout

> **Note on the root folder:** the project content used to live under `Informatica/Post-Claude/`. It was **moved up** so the project root is now **`Informatica/`** directly (contains `.git`, `main/`, `transceiver/`, `IHM/`, `test/`, `docs/`, `CLAUDE.md`). The folder name may change again — treat the directory that contains `main/main.ino` as the **project root**. Older docs/memory may still say "Post-Claude".

```
<project-root>/                 (currently Informatica/, formerly Post-Claude/)
├── CLAUDE.md                   — long-form dev log / reference (authoritative history)
├── README.md                   — build/usage readme (somewhat stale vs current firmware)
├── AI_CONTEXT.md               — THIS FILE
├── main/
│   ├── main.ino                — entry point: DroneApp app; setup(){begin()} loop(){update()}
│   └── src/
│       ├── app/DroneApp.*      — orchestrator + cooperative scheduler
│       ├── config/             — BoardConfig.h (pins/thresholds), Calibration.h (µs values)
│       ├── core/Types.h        — ControlMode enum, RcFrame, ActuatorCommand, GpsPosition, Waypoint…
│       ├── control/            — ModeManager, ManualController, AutoController
│       ├── drivers/            — AxpPower, RcReceiver, McpwmActuators, GpsUart, LoRaRadio, BatteryAdc
│       ├── comm/LoRaComm.*     — JSON heartbeat builder + command dispatcher
│       └── navigation/         — Navigator, MissionPlan, MissionManager,
│                                 navigation.h, oldNavigation.h, NavigationConfig.h, NavigationSelector.h
├── transceiver/transceiver.ino — ground-station T-Beam sketch (USB ↔ LoRa bridge + RSSI injection)
├── IHM/                        — web ground station (FastAPI + MongoDB + Leaflet); start_ihm.sh / stop_ihm.sh
├── test/                       — native (host) unit tests — CURRENTLY BROKEN/STALE (see §15)
└── docs/                       — LaTeX reports (FR + ES) and PDFs
```

**Legacy directories to IGNORE (defective / reference only):**
- `Informatica/Arduino/boat/` — first group's original `boat.ino` (Mohamed EL JILY). Defective.
- `Informatica/Post_GPT/` — an earlier GPT-assisted rewrite; use only as architectural reference.

> Arduino IDE quirk: the IDE only compiles `.cpp` in the sketch folder or under a `src/` subdir. All firmware source lives under `main/src/...` and uses **relative includes** (`../sibling/Foo.h`).

---

## 5. Software architecture

**Style:** layered OOP, single-core **cooperative event loop** (no RTOS). `DroneApp::update()` runs every `loop()` and fires each task when its period elapses (no `delay()`, which would break RC interrupt timing).

```
main.ino
└── DroneApp (orchestrator)
      ├── DRIVERS
      │   ├── AxpPower        — AXP192 init: LDO2=LoRa, LDO3=GPS, DCDC1=3.3V, battery ADC on
      │   ├── RcReceiver      — 4-ch interrupt PWM (CH2/3/4/5), ISR-safe portMUX, 100 ms loss timeout
      │   ├── McpwmActuators  — MCPWM (NOT LEDC) for sail/rudder/ESC; slew-rate limited
      │   ├── GpsUart         — Serial1, TinyGPSPlus; sends UBX CFG-CFG + CFG-ANT at boot
      │   ├── LoRaRadio       — SX1276, SPI HSPI, 433 MHz, blocking TX, polling RX
      │   └── BatteryAdc      — GPIO36, divider 5.683, 11 dB
      ├── COMM
      │   └── LoRaComm        — JSON heartbeat TX (1 Hz) + command RX dispatcher
      ├── CONTROL
      │   ├── ModeManager     — CH5 → ControlMode {Failsafe, Sail, Manual, Automatic}
      │   ├── ManualController— unified manual: sail(CH2) + rudder(CH4) + bidirectional motor(CH3)
      │   └── AutoController  — wraps navigation.h; wind observation; maps nav rudder→winch µs
      └── NAVIGATION
          ├── Navigator       — haversine distance + bearing (stateless namespace)
          ├── MissionPlan     — up to 16 waypoints + mode (Linear/Circuit)
          └── MissionManager  — state machine: Idle→Running→Returning→Complete; emergencyReturn()
```

**Scheduler periods** (`main/src/app/DroneApp.h`):

| Task | Period |
|---|---|
| Control loop (RC read → mode → actuators) | **20 ms (50 Hz)** |
| Battery sample | 250 ms |
| Debug serial print | 200 ms (5 Hz) |
| LoRa heartbeat TX | **1000 ms (1 Hz)** |
| GPS drain / LoRa RX poll | every `loop()` (not rate-limited) |

---

## 6. Critical technical decisions (non-negotiable design rules)

1. **MCPWM, never LEDC.** The ESP32 LEDC peripheral (used by `ESP32Servo`/`analogWrite`) corrupts the interrupt-driven RC reading — channels read 0. MCPWM is independent of GPIO interrupts. *Never reintroduce LEDC/ESP32Servo for actuators.*
2. **Interrupt-driven RC, ISR-safe.** GPIO CHANGE interrupts capture `micros()` on edges; `portMUX` critical sections; valid pulse window 800–2200 µs; 100 ms timeout = channel lost (returns 0). ISRs in `IRAM_ATTR`.
3. **All actuator commands in microseconds**, never degrees — matches PWM hardware, fewer conversion bugs.
4. **Sail is binary** (+1/−1 → SAIL_PLUS_US/SAIL_MINUS_US), with a center deadband and memorized state.
5. **Slew-rate limiting** on the ESC output: max **30 µs per 20 ms tick** (≈1500 µs/s) to avoid mechanical jerks.
6. **BC548 needs `DUTY_MODE_0` on both PWM channels** (the transistor already inverts).
7. **AXP192 rails must be enabled in code** or GPS/LoRa get no power. `Wire.end()` removed → I2C stays live.
8. **GPS needs UBX init at boot:** CFG-CFG (factory reset, clears a flash config that can mute NMEA) + CFG-ANT (`flags=0x001B`, powers the active antenna's LNA and switches RF to the u.FL connector).

---

## 7. Control modes (CH5 selector)

`ModeManager::decode()` (authoritative — `main/src/control/ModeManager.cpp` + `BoardConfig.h`):

| CH5 pulse | Mode | Behaviour |
|---|---|---|
| **≤ 1250 µs** | **Automatic** | Follows the LoRa mission (autonomous navigation when a target is active and wind is known) |
| **1400–1600 µs** | **Sail** | **Inert** — all outputs neutral (`ActuatorCommand{}` defaults). Also the safe fallback for out-of-band CH5. |
| **> 1800 µs** | **Manual** | Unified manual control (sail + rudder + bidirectional motor) |
| **0 (signal lost)** | **Failsafe** | **Behaves like Automatic** — follows the mission autonomously |

> ⚠️ **Safety:** Failsafe = autonomous. If RC is lost, the boat tries to navigate by itself. Autonomous mode has **never been tested on water**, so the **first real trial must be in Manual**, in calm water, with a recovery plan.
>
> Note: which *physical* stick direction maps to low/high CH5 depends on the TX setup/channel reversing. The firmware numbers above are the truth; some hand-edited docs describe the stick direction differently.

Enum (`core/Types.h`): `ControlMode { Failsafe, Sail, Manual, Automatic }`. (Renamed from the older `ManualServo`/`ManualProp` split when the unified manual mode was introduced.)

---

## 8. Manual mode (detailed)

One function `ManualController::computeManual()` drives everything:

- **CH2 → sail (binary):** deadband ±35 µs around 1500 µs. First frame initializes the state from stick position (avoids a snap on entry). Inside the deadband the last state holds. Output = `SAIL_PLUS_US` (+10°) or `SAIL_MINUS_US` (−10°).
- **CH4 → rudder winch (positional):** measured stick travel `CH4_MIN_US=1180 … CH4_MAX_US=1790` maps linearly to `ROTOR_MIN_US=1417 … ROTOR_MAX_US=1583` (= ±90° physical). Center deadband snaps to `ROTOR_CENTER_US=1500`.
- **CH3 → propeller (BIDIRECTIONAL):** *ratchet* throttle (does not self-center). See below.

### 8.1 Bidirectional motor (added; the ESC neutral is now 1500 µs)

Originally the motor was 0–100 % forward only (unidirectional ESC, 1000 µs = stop). It was changed to **±100 %** using the ESC in **bidirectional mode** (neutral = 1500 µs):

| CH3 stick | Power | ESC µs |
|---|---|---|
| `CH3_FULL_US` = 1100 | +100 % forward | 2000 |
| `CH3_CENTER_US` = 1545 (±`ESC_CENTER_DEADBAND_US`=40) | 0 % stop | 1500 |
| `CH3_ZERO_US` = 1990 | −100 % reverse | 1000 |

To keep the whole firmware coherent, **neutral = 1500 µs is used everywhere** (`ESC_NEUTRAL_US`, also aliased as `ESC_STOP_US`): boot default, Failsafe, Sail mode, auto-stop. The write clamp's lower bound was dropped to `ESC_REVERSE_MIN_US=1000` to allow reverse. The default `ActuatorCommand.esc1Us` and the actuator init were changed from 1000→1500. Auto-mode throttle band (`AUTO_ESC_*`) was shifted into the forward half (1600/1700/1850).

> ⚠️ **Reverse only works physically if the ESC is programmed for bidirectional throttle with the EPRG-3 card.** Otherwise <1500 µs is ignored (forward-only). `CH3_CENTER_US=1545` is an assumed midpoint and needs bench verification.

---

## 9. Calibration constants (`main/src/config/Calibration.h`)

| Constant | Value | Notes |
|---|---|---|
| `SAIL_CENTER_US` | 1520 | Futaba S3003 center (not 1500) |
| `SAIL_PLUS_US` / `SAIL_MINUS_US` | 1575 / 1465 | ±10° — **need bench calibration** |
| `ROTOR_CENTER_US` | 1500 | rudder center |
| `ROTOR_MIN_US` / `ROTOR_MAX_US` | 1417 / 1583 | ±90° physical (manual). HW can do ±1080° but it's limited. |
| `ROTOR_RANGE_DEG` | 90 | physical half-travel mapping |
| `ROTOR_AUTO_RANGE_DEG` | 20 | auto mode uses only ±20° of winch (±90° was too aggressive) |
| `ESC_REVERSE_MIN_US` | 1000 | −100 % reverse (bidirectional) |
| `ESC_NEUTRAL_US` (= `ESC_STOP_US`) | 1500 | neutral / stop |
| `ESC_MAX_US` | 2000 | +100 % forward |
| `ESC_CENTER_DEADBAND_US` | 40 | throttle center deadband |
| `ESC_SLEW_US` | 30 µs/tick | slew limit |
| `AUTO_ESC_MIN/CRUISE/MAX_US` | 1600/1700/1850 | forward-half band for auto propulsion (untested) |
| `WIND_OBS_DISTANCE_M` | 30 | travel before wind estimate latches |
| `WIND_OBS_SMOOTH_ALPHA` | 0.1 | circular EMA factor for GPS course |
| Battery divider ratio | 5.683 | (562k+120k)/120k |
| `RC_DEADBAND_US` | 35 | stick deadband |
| CH5 thresholds | 1250 / 1400–1600 / 1800 | Auto / Sail / Manual |

(The old ESC arming gesture — hold throttle low 2 s — was **removed** when the bidirectional + unified-manual changes landed. `ESC_ARM_*` constants are dead.)

---

## 10. Navigation (autonomous)

### 10.1 Wind estimation (Phase 5 — coded, untested on water)

No wind sensor → the wind is observed by sailing a fixed tack:
1. A `wind-observation` LoRa command starts it. The boat fixes the sail at +10° (one tack).
2. The GPS course is smoothed with a circular EMA.
3. After traveling `WIND_OBS_DISTANCE_M` = 30 m, wind direction is latched as `smoothed_heading + 90°` (via `nav_handleWindObservation`).
4. Alternatively a `wind-command {value: deg}` LoRa command sets the wind direction manually (the reliable fallback).

The `wobs` field in the heartbeat reports observation progress (0–100 %) only while measuring. ⚠️ The +90° offset and 30 m threshold need field validation.

### 10.2 Sail navigation algorithm (Phase 6 — coded, untested on water, no unit tests)

`navigation.h` is a header-only algorithm **written by a teammate** (branch `Testautoboat` of `DeltaGod/PER_MEA_LYINCROYAP`) and integrated here. It is shared between firmware and a simulation. Public entry points:
- `nav_handleNavigationWithState(state, boatHeading, wptHeading, wptDist, windDir, sailAngle, rudderAngle, reachedDist, boatLat, boatLng, wptLat, wptLng, corridorHalfWidth) → NavResult`
- `nav_handleWindObservation(...) → NavResult`

What it does:
- **Upwind/downwind forbidden zones** with zigzag tacking (upwind forbidden ±45°, downwind ±20°).
- **Gybe (empannage) avoidance** via a small state machine (loop around through upwind).
- **Cross-track corridor** around the start→waypoint line (default half-width **100 m**).
- **Lofer/abattre** (luff up / bear away) for fine heading via a rudder integrator.
- Smarter initial tack selection (`nav_sideMovingTowardWaypoint` — picks the side that makes the most progress toward the waypoint).
- "Passed the waypoint plane within the corridor" arrival detection (`nav_hasPassedWaypoint`) in addition to the distance check.

**Selector mechanism:** `AutoController.h` includes `NavigationSelector.h`, which picks `navigation.h` or `oldNavigation.h` based on `#define USE_OLD_NAVIGATION` in `NavigationConfig.h` (default **0** = current algorithm). `oldNavigation.h` is the legacy fallback (kept, not compiled by default).

**`AutoController`** wraps the algorithm: it keeps persistent `sailAngle_`/`rudderAngle_` state (the lofer/abattre integrator needs it) and maps the nav rudder degrees **1:1 to physical winch degrees, clamped to ±`ROTOR_AUTO_RANGE_DEG` = 20°**. History: original mapping saturated; an early fix mapped ±20°→full ±90° which the author found far too aggressive ("demasiado elevado"); reverted to the gentle ±20° physical travel. Manual mode still uses the full ±90°.

**Missing for real autonomy:** a **WinchTracker** (the Regatta ECO II has no position feedback, so the firmware can't know the true rudder position — it's integrated open-loop). Autonomous propulsion (ESC in auto) is effectively unused.

---

## 11. LoRa protocol

**Radio:** SX1276, **433 MHz**, default SF7 / BW 125 kHz / CR 4/5. TX is **blocking** (`LoRa.endPacket()` ≈ 450 ms for a ~210 B packet at SF7 — blocks the loop once per second; acceptable for now, should go async before serious autonomous water tests). Parsed with `strstr()` on char buffers (no ArduinoJson).

### 11.1 Heartbeat (drone → ground, 1 Hz), current format

```json
{"origin":"boat","type":"info","message":{
  "mode":"standby|route-ready|navigate",
  "location":[48.36000,-4.56000],
  "servos":{"sail":10,"rudder":-12},
  "heading":270,"wind":180,"bat":7.42,
  "fix":1,"sat":7,"hdop":2.2,"rc":1,
  "wt":3,"wc":1,"wobs":42,"rssi":-104}}
```
- `servos.sail` = ±10° (binary); `servos.rudder` = winch degrees (manual ±90° / auto ±20°).
- `fix/sat/hdop` = GPS health; `rc`=1 if the receiver delivers pulses.
- `wt`/`wc` = waypoints total/current (flattened from the old `waypoints:{total,current}` to save bytes).
- `wobs` = wind-observation progress 0–100 %, **present only while measuring**.
- **`rssi` is NOT sent by the boat** — the *transceiver* appends `,"rssi":<packetRssi>` before the closing `}}` on each forwarded line (measured on the ground side over USB → doesn't count against the 255 B LoRa FIFO budget).
- **`control_mode` was removed** (100 % redundant with `mode`; the IHM derives it). Mode mapping: Failsafe/Manual/Sail→`standby`, Auto+Idle/Complete→`route-ready`, Auto+Running/Returning→`navigate`.
- Size ≈ 210 B (≈ 232 B worst case with `wobs`) — comfortably under the **255 B** SX1276 FIFO limit. Keep it small: integer heading, 5-decimal location.
- Measured RSSI −101…−106 dBm at close range (hardware-tested).

### 11.2 Commands (ground → drone)

```json
{"origin":"server","type":"command","message":{"navigate":true}}
{"origin":"server","type":"command","message":{"stop":true}}
{"origin":"server","type":"command","message":{"restart":true}}
{"origin":"server","type":"command","message":{"wind-observation":true}}
{"origin":"server","type":"command","message":{"wind-command":{"value":225}}}
{"origin":"server","type":"command","message":{"home":{"lat":48.36,"lon":-4.56}}}
{"origin":"server","type":"command","message":{"waypoints":{"number":2,"points":"48.36,-4.56,48.37,-4.55"}}}
```
All 7 uplink commands were hardware-verified.

### 11.3 Transceiver (ground T-Beam)

`transceiver/transceiver.ino` on a second T-Beam, USB 115200. Bridges USB serial ↔ LoRa. Accepts shorthand CLI (`navigate`, `stop`, `restart`, `wind-obs`, `wind <deg>`, `home <lat,lon>`, `wpt <lat,lon,...>`) and raw JSON passthrough, and **injects RSSI** into each forwarded heartbeat.

---

## 12. Ground station web UI (IHM)

Located in `IHM/`. Stack: **Python FastAPI** webserver + **MongoDB** (Docker, port 27017) + **`serial_link.py`** (reads/writes the transceiver serial port, uses MongoDB as a message bus) + a **Leaflet.js** map in the browser (port **5000**), polling `/api/messages` at 1 Hz.

**Python deps** (`IHM/requirements.txt`): `fastapi`, `uvicorn`, `pymongo[asyncio]`, `pyserial`, `python-dotenv`. **MongoDB** via `IHM/docker-compose.yml` (`mongo:7`).

**Features:** map with boat position; click-to-place waypoints + send route; health panel with traffic-lights (Link/GPS/Battery/RC/Control) + RSSI; wind compass; trajectory prediction (`nav.js` = exact JS replica of `navigation.h` + a kinematic simulator); wind-measurement progress bar; Sim/Réel toggle; endpoints `/api/start /stop /set-mode /reconnect /reset-transceiver /navigate /wind-observation /wind-command /send-route`.

**Transceiver auto-detection:** both T-Beams are Silicon Labs **CP2104** with unique serials. `config.py` resolves the transceiver port from `/dev/serial/by-id/` by serial number (no udev/root needed); `serial_link.py` re-resolves and reconnects on USB re-enumeration, opening with `dsrdtr/rtscts=False` so it doesn't reset the ESP32. **Serials: transceiver = `01C00B54`, boat = `02126CF1`.** (Optional udev rule in `IHM/99-autoboat.rules`.)

**Run (Linux):**
```bash
cd IHM
python3 -m venv .venv && .venv/bin/pip install -r requirements.txt   # once
./start_ihm.sh                 # autodetects the transceiver; opens http://localhost:5000
./start_ihm.sh /dev/ttyUSB0    # or force the port
./stop_ihm.sh                  # frees port 5000, brings MongoDB down
```
`start_ihm.sh` launches MongoDB (docker compose), `serial_link.py`, `webserver.py`, and opens the browser. Scripts are robust (TERM→KILL, frees the uvicorn worker on 5000, `docker compose down`, `python -u`).

**Run (Windows):** easiest via **WSL2 + Docker Desktop** (same scripts inside Ubuntu); attach the USB to WSL with **`usbipd-win`** (`usbipd bind/attach --wsl --busid <id>`). Native Windows alternative: Docker Desktop + Python, run `docker compose up`, `python serial_link.py`, `python webserver.py` manually, set `SERIAL_PORT_REAL=COMx` in `.env`.

> ⚠️ **EDUROAM does not work.** On the EDUROAM institutional WiFi the IHM fails (client isolation / firewall/proxy block Docker/MongoDB and local comms). **Workaround tested by the group: tether the laptop to a phone hotspot.**
>
> ⚠️ **Never open a serial monitor** (gtkterm, Arduino IDE monitor, pyserial) on the transceiver port **while the IHM is running** — two readers corrupt the JSON.

---

## 13. Problems encountered & solutions (the full list)

### 13.1 Resolved

| Problem | Cause | Solution |
|---|---|---|
| **RC channels read 0 when servos active** | `ESP32Servo` uses LEDC, which breaks GPIO-interrupt RC decoding | Rewrote actuators on **MCPWM**; never use LEDC |
| **RC pins clashed with I2C/AXP192** | Original used GPIO21/22 (I2C) and GPIO23 (LoRa RST) for RC | PCB V2 moved RC to GPIO39/14/13/4; CH6 removed |
| **~20 mA continuous drain** | GPIO35 has a low-impedance divider soldered on | Never read GPIO35; battery ADC on GPIO36 + high-impedance divider |
| **LoRa RST didn't work** | LilyGO docs say GPIO14; hardware is GPIO23 | Use GPIO23 |
| **Rudder servo dead** | `MCPWM_DUTY_MODE_1` on OPR_B double-inverted through BC548 | `DUTY_MODE_0` on both channels |
| **ESC never armed** | PTR-6A throttle min ≈1265 µs; arm threshold was 1050 | Raised to 1300 (then arming removed entirely with bidirectional rework) |
| **Stuck in Automatic at boot** | ModeManager read CH4 instead of CH5 | Read `frame.ch5` |
| **GPS silent (no NMEA)** | A saved CFG-PRT in NEO-6M flash disabled NMEA output | Send **UBX CFG-CFG** factory reset at boot |
| **GPS: 0 satellites with external antenna** | Antenna supervisor off → active antenna LNA unpowered, RF on internal antenna | Send **UBX CFG-ANT** `flags=0x001B` |
| **Battery read 17.9 V** | Divider resistors physically swapped (120k/562k) | Swap resistors back (code was correct) |
| **Serial port busy on flash** | A monitor/script held the port | `fuser /dev/ttyUSB0` + kill, or close the monitor |

### 13.2 Power topology — the big one (T-Beam can't be powered from header pins)

The T-Beam V1.1 **can only be powered reliably from its battery holder (JST) or USB**, **not** by injecting 5 V on the expansion header pins. But PCB V2 was designed to feed the T-Beam through the header `+5V` pin (Pololu output). That pin sits on the USB-VBUS path **after the protection diode** and **does not reliably reach the AXP192** (the PMIC that enables the 3.3 V / GPS / LoRa rails). So on header-pin power the AXP192 often **won't boot**, and under load the rail collapses (boat observed running at **~0.81 V**, a brownout), which also makes the CP2104 USB-serial chip re-enumerate (`cp210x ... status -19`).

**Quick patch adopted:** a USB-A→USB-C cable was cut; the **USB-A end (+5V/GND) soldered directly to the PCB 5 V rail**, USB-C plugged into the T-Beam — so PCB power enters via the T-Beam's *valid USB path* (VBUS→AXP192). It's an ugly patch. A proper PCB revision should route power to the correct T-Beam input (VBUS or the battery JST), not the header `+5V` pin.

### 13.3 Motor "pulses" — ESC low-voltage cutoff (NOT a code bug)

At ~7 V the motor runs in bursts: it spins a moment, cuts, and only restarts after going to 0 % then back. **Cause:** the ESC's **low-voltage cutoff (LVC)**. A 2S pack at 7 V is ~3.5 V/cell; under motor load it sags below the ESC cutoff (~3.0–3.2 V/cell), the ESC cuts to protect the pack, voltage recovers unloaded, the cutoff re-arms → pulsing. Much worse if the ESC cell-count is mis-programmed. **Solution:** a fully-charged pack and **program the cell count / LVC with the EPRG-3 card**; the group's documented fix was to **chain LiPos in series for higher voltage** (~15.4 V) and add battery capacity/space. No code change fixes LVC.

### 13.4 Flashing fails — same brownout

`arduino-cli upload` fails repeatedly (`chip stopped responding`, `No serial data received`) with the CP2104 re-enumerating mid-write — **only when the T-Beam is powered through the PCB**. The flash erase/write current spike browns out the ESP32. **Solution:** disconnect the T-Beam from the PCB, flash it, and (without unplugging USB) reseat it on the PCB. Or hold BOOT/IO0 + tap RST to force bootloader, and use a good USB cable.

---

## 14. Open problems & proposed improvements (future work)

| Open problem | Proposed improvement | Connector / note |
|---|---|---|
| **LoRa range terrible** (antenna inside the electronics box) | External **433 MHz antenna on the main mast**, high and clear of the box/metal/electronics | T-Beam SX1276 exposes **u.FL/IPEX**; external 433 antennas use **SMA** → need a **u.FL→SMA pigtail** (must be 433 MHz, not 2.4 GHz) |
| **GPS inconsistencies** (fix intermittent, few sats) | Separate **external active GPS antenna on the deck**, clear sky view, away from LoRa/electronics | NEO-6M input is **u.FL**; active antennas often **SMA** → need adapter; antenna must be **active** (supervisor already enabled in firmware) |
| **No wind direction & no sail-position sensing → real rudder position unknown** | **2 sensors** (wind direction + sail position) on the **I2C bus via the existing PCB expansion connector** (GPIO21/22). With both, the true rudder position can be derived and the control loop closed | Firmware already reserves I2C; needs a `SensorBus` driver in software |
| **Little battery space in the box** | Mechanical redesign for a larger/higher-capacity pack (keep voltage above the ESC LVC for the whole mission); account for new sensors + antenna connectors | related to §13.3 |
| **No rudder position feedback** | Software **WinchTracker** (integrate speed×time) or a real encoder | needed for trustworthy autonomous steering |
| **No external sensors / no data logging** | Implement Phase 4 (`SensorBus`, I2C) and Phase 7 (SPIFFS/SD logging) | this is the actual *purpose* of the drone |
| **Autonomous propulsion unused** | Wire the ESC into auto mode (PropulsionAutoMode) for no-wind legs | `AUTO_ESC_*` already forward-biased |
| **Unit tests broken** | Rewrite `test/test_runner.cpp` for current behavior | see §15 |

---

## 15. Build, flash, status

### 15.1 Build & flash (`arduino-cli`, FQBN `esp32:esp32:t-beam`)

```bash
# paths relative to the project root (folder containing main/)
~/bin/arduino-cli compile --fqbn esp32:esp32:t-beam main
~/bin/arduino-cli upload  --port /dev/ttyUSB0 --fqbn esp32:esp32:t-beam main          # boat
~/bin/arduino-cli upload  --port /dev/ttyUSB0 --fqbn esp32:esp32:t-beam transceiver   # ground radio
```
- **Libraries:** ESP32 Arduino Core 3.x, `AXP202X_Library` 1.1.2, `TinyGPSPlus` 1.0.3, `LoRa` 0.8.0.
- **Footprint:** drone firmware ≈ **28 % flash, 7 % RAM** (368 KB / 25 KB). Transceiver ≈ 23 % / 6 %.
- Identify boards by serial: `ls -l /dev/serial/by-id/` → boat `02126CF1`, transceiver `01C00B54`.
- **If flashing fails (brownout):** power from a charged LiPo, or BOOT/IO0 + RST, or the "disconnect from PCB, flash, reseat" trick (§13.4).
- Serial monitor at 115200 (deassert DTR/RTS to avoid holding the ESP32 in reset).

### 15.2 Boot serial signature (healthy)
```
[BAT]  OK  — R2=562k R1=120k, 11dB attenuation
[LORA] OK  — 433 MHz, TX heartbeat 1 Hz
[ACT]  OK  — sail=1520 rotor=1500 esc=1000
[GPS ] ... (NO FIX until satellites; LED TIMEPULSE blinks 1 Hz on fix)
```

### 15.3 Phase status

| Phase | Scope | Status |
|---|---|---|
| 1 — Manual RC | AXP192, RC interrupts, MCPWM, modes, unified manual + bidirectional motor | ✅ Coded & hardware-tested |
| 2 — GPS + Navigation | GpsUart, Navigator, MissionManager | ✅ Hardware-tested outdoors (fix acquired) |
| 3 — LoRa | Heartbeat + commands + transceiver | ✅ Hardware-tested (RSSI −101…−106 dBm) |
| 5 — Wind estimation | GPS-track + manual override | ✅ Coded, **untested on water** |
| 6 — Autonomous sailing | navigation.h (Testautoboat), AutoController | ✅ Coded, **untested on water, no unit tests** |
| 4 — Sensors (I2C) | SensorBus — the drone's actual data purpose | ❌ Not started |
| 7 — Logging / full mission | SPIFFS/SD, autonomous propulsion, WinchTracker | ❌ Not started |

**Tests are stale/broken:** `test/test_runner.cpp` predates the `main/src/` reorg, the single-ESC migration, and the Sail/Manual rename (references `esc2Us`, old thresholds, `isEscArmed()`). The historical "106/106 tests pass" milestone no longer builds. Firmware itself compiles/flashes fine.

---

## 16. Pre-trial checklist (before putting it on water)

1. **Charge the LiPo** and connect it to the battery holder (never power from header pins). More capacity/voltage helps the LVC issue.
2. **Program the ESC** with EPRG-3: cell count, LVC, and **bidirectional mode** (or reverse won't work).
3. **Bench-calibrate** `SAIL_PLUS_US`/`SAIL_MINUS_US` and verify `CH3_CENTER_US`.
4. Confirm **CH5 mode mapping** on your TX (Failsafe = autonomous!).
5. **First trial in Manual**, calm water, recovery plan (boat/line). Autonomous has never run on water.
6. Don't open a serial monitor on the transceiver while the IHM runs.

---

## 17. Glossary (the codebase mixes EN/FR/ES)

| Term | Meaning |
|---|---|
| **voile** (FR) / vela (ES) | sail |
| **safran** (FR) | rudder (the winch-driven steering) |
| **amure** (FR) / amura (ES) / tack | which side the boat sails relative to wind |
| **lofer / orzar** | luff up (turn toward the wind) |
| **abattre / arribar** | bear away (turn away from the wind) |
| **empannage / gybe** | turning the stern through the wind (avoided by the algorithm) |
| **près / upwind** | sailing toward the wind (forbidden zone ±45°) |
| **portant / downwind** | sailing away from the wind (forbidden zone ±20°) |
| **cross-track** | lateral deviation from the start→waypoint line (corridor) |
| **winch / treuil** | the multi-turn positional servo driving the rudder |
| **IHM** | Interface Homme-Machine = the ground-station web UI |
| **heartbeat** | the 1 Hz JSON telemetry packet boat→ground |

---

## 18. Document map (where to read more)

- **`CLAUDE.md`** (project root) — long-form chronological dev log + reference (most detailed history; some sections predate the latest changes).
- **`docs/informe_proyecto_es.tex` / `.pdf`** — full technical report in **Spanish** (includes a usage guide for the next group, Linux/Windows IHM setup, problems & improvements).
- **`docs/rapport_projet_fr.tex` / `.pdf`** — **French** translation of the Spanish report.
- **`docs/rapport_projet.tex`** — older French report (v2), partly superseded.
- **`README.md`** — build/usage; somewhat stale vs current firmware (still mentions dual ESC / old mode names in places).

> When docs and code disagree, **the code is authoritative.** Recent code changes (bidirectional motor, unified manual mode, navigation upgrade, moved folder layout) may not be reflected everywhere in prose.
