# Wind Observation Analysis — Discussion Summary

**Date:** 2026-09-09  
**Project:** PER MEA S9P-2026 — Autonomous Sailing Drone  
**Context:** Discussion between developer and AI assistant regarding wind estimation, angle relationships, and EMA smoothing implementation.

---

## 1. The "90° Phase" / Angle Relationship Question

### Original Question
> "If wind is -90° on the left with sail angle of -10°, the angle between them would be -80° on the motor side"

### Analysis

**Mathematically correct, but terminology is wrong.**

| Concept | Value | Meaning |
|---------|-------|---------|
| Relative wind (wind from port) | -90° | `nav_relativeAngle(boatHeading, windDir)` |
| Sail aileron deflection (port) | -10° | `NAV_SAIL_LEFT_DEG` |
| Difference (sail - wind) | **+80°** | Sail is 80° "inside" wind direction |
| Difference (wind - sail) | **-80°** | Wind is 80° "outside" sail angle |

**Critical clarification:** There is **no "motor side" angle**. The system has three independent actuators:

| Actuator | Function | Range | Controlled By |
|----------|----------|-------|---------------|
| **Sail aileron** | Binary tack selection | ±10° only | Navigation algorithm (wind-relative) |
| **Rudder winch** | Heading/yaw control | ±90° (manual) / ±110° (auto) | Navigation algorithm (waypoint-relative) |
| **Propeller (ESC)** | Thrust forward/reverse | Bidirectional | Manual mode only (CH3) |

The 80° difference between relative wind (-90°) and sail angle (-10°) is **expected physics** — the wing sail self-aligns at ~45° to hull, aileron adds ±10° to select tack. The rudder winch (what might be called "motor side") has no fixed angular relationship to either.

---

## 2. Implementation Status

### Sail Logic — IMPLEMENTED ✅
**File:** `main/src/navigation/navigation.h` (lines 280–281, 430–431)

```cpp
// Used in both direct navigation and lofer/abattre
result.sailAngle = (relativeWindDeg < 0) ? NAV_SAIL_LEFT_DEG : NAV_SAIL_RIGHT_DEG;
// NAV_SAIL_LEFT_DEG  = -10°
// NAV_SAIL_RIGHT_DEG = +10°
```

### Wind Observation (EMA Smoothing) — IMPLEMENTED ✅
**File:** `main/src/control/AutoController.cpp` (lines 99–144)

```cpp
// Circular EMA of GPS course during wind observation
float d = (float)nav_relativeAngle(smoothHeading_, pos.courseDeg);
smoothHeading_ = (float)nav_normalizeAngle(
    smoothHeading_ + Calibration::WIND_OBS_SMOOTH_ALPHA * d);  // alpha = 0.1
```

**Parameters (Calibration.h):**
```cpp
static constexpr float WIND_OBS_DISTANCE_M  = 30.0f;  // Min travel before estimate valid
static constexpr float WIND_OBS_SMOOTH_ALPHA = 0.1f;  // Circular EMA factor (10% new)
```

**Flow:**
1. `wind-observation` LoRa command → `AutoController::beginWindObservation()`
2. Sail fixed at +10° (starboard tack), rudder centered (1500 µs)
3. Each GPS fix: EMA smooth `courseDeg` into `smoothHeading_`
4. Distance tracked via raw GPS position (NOT smoothed): `Navigator::distanceM(obsStartLat, obsStartLon, pos.lat, pos.lon)`
5. At 30m: `observedWindDeg_ = smoothHeading + 90°` (wind comes from 90° to boat track)

---

## 3. Valid Concerns Raised About Smoothing

### Concern: "Lose actual coordinates due to smoothing"
**Answer: NO** — Position is NEVER smoothed. Only `courseDeg` (heading) is EMA-filtered. Distance calculation uses raw lat/lon:
```cpp
double dist = Navigator::distanceM(obsStartLat_, obsStartLon_, pos.lat, pos.lon);  // Raw!
```

### Concern: "Lose motor calibration due to normalization"
**Answer: NO** — Wind observation doesn't touch actuator calibration. During observation:
```cpp
cmd.sailUs  = Calibration::SAIL_PLUS_US;    // Fixed +10°
cmd.rotorUs = Calibration::ROTOR_CENTER_US; // Centered rudder
cmd.esc1Us  = 0;                            // ESC unused
```
Only output is `observedWindDeg_` (float, 0–360°), used later by navigation.

### Concern: "Blind spots + normalization = problems"
**Answer: PARTIAL** — Blind spots (GPS dropout) are handled correctly:
```cpp
if (!pos.valid) return cmd;  // Skips EMA update — good!
```
**But outliers are NOT rejected** — a single bad `courseDeg` pollutes the EMA.

---

## 4. Real Risks in Current Implementation

| Risk | Current Code | Impact |
|------|--------------|--------|
| **GPS course noise at low speed** | No speed filter on `courseDeg` | Noisy heading → slow convergence or biased estimate |
| **Alpha = 0.1 too slow** | Fixed 0.1 weight | ~10 GPS updates (10–20s at 1Hz) to converge; boat may have turned |
| **No outlier rejection** | Single bad `courseDeg` pollutes EMA | One GPS glitch = 10+ seconds to recover |
| **Circular normalization edge case** | `nav_relativeAngle` + `nav_normalizeAngle` | Generally robust (359°→1° handled correctly) |
| **Boat may not hold tack** | Sail fixed, rudder centered, but wind/current may turn boat | EMA tracks wrong heading if boat doesn't stabilize |

### Vulnerability Example
```cpp
// Line 118-120: EMA update with no guards
float d = (float)nav_relativeAngle(smoothHeading_, pos.courseDeg);
smoothHeading_ = (float)nav_normalizeAngle(
    smoothHeading_ + Calibration::WIND_OBS_SMOOTH_ALPHA * d);  // alpha = 0.1
```

**Scenario:** GPS glitch gives course = 180° when true heading = 0°
- `d = nav_relativeAngle(0, 180) = 180`
- `smoothHeading = 0 + 0.1 × 180 = 18°` (wrong!)
- Takes ~10 updates to recover

---

## 5. Recommended Improvements (Not Implemented — For Future Work)

```cpp
// 1. Speed gate — ignore course when barely moving
if (pos.speedKmph < 0.5f) {
    navMessage_ = "speed too low for course";
    return cmd;  // Don't update EMA
}

// 2. Outlier rejection — reject course jumps > 60° from smoothed
float d = nav_relativeAngle(smoothHeading_, pos.courseDeg);
if (std::abs(d) > 60.0f) {
    navMessage_ = "course outlier rejected";
    return cmd;
}

// 3. Adaptive alpha — faster convergence initially
float alpha = (windObsProgressPct_ < 50) ? 0.3f : 0.1f;
smoothHeading_ = nav_normalizeAngle(smoothHeading_ + alpha * d);
```

---

## 6. Key Files Reference

| File | Purpose |
|------|---------|
| `main/src/navigation/navigation.h` | Core navigation algorithm (header-only, shared with simulator) |
| `main/src/control/AutoController.h/.cpp` | Wraps navigation, maps rudder°→winch µs, handles wind observation |
| `main/src/config/Calibration.h` | All calibration constants (sail, rudder, ESC, wind obs params) |
| `main/src/navigation/Navigator.h/.cpp` | Haversine distance/bearing (stateless) |
| `main/src/navigation/MissionManager.h/.cpp` | Mission state machine |

---

## 7. Summary

| Topic | Status |
|-------|--------|
| Sail/wind angle relationship (80° difference) | **Correct physics, implemented correctly** |
| "Motor side" angle concept | **Misunderstanding — no such fixed relationship exists** |
| Wind observation EMA smoothing | **Implemented, but vulnerable to GPS noise/glitches** |
| Position smoothing | **NOT done — only heading is smoothed** |
| Actuator calibration coupling | **NONE — wind obs outputs only wind direction float** |
| Blind spot handling | **Correctly skips EMA update on invalid GPS** |
| Outlier rejection | **MISSING — major vulnerability** |
| Speed filtering | **MISSING — course noise at low speed not filtered** |

---

## 8. Position Smoothing — Why It's NOT Done (Simple Explanation)

### What "Position Smoothing" Would Mean
If we applied the same EMA filter to GPS latitude/longitude that we apply to heading:
```
smoothLat = smoothLat + alpha * (rawLat - smoothLat)
smoothLon = smoothLon + alpha * (rawLon - smoothLon)
```

### Why We DON'T Do This (Critical)

| Reason | Explanation |
|--------|-------------|
| **Distance would be wrong** | The wind observation needs to know when the boat has traveled **30 meters**. If you smooth position, the "smoothed position" lags behind the real position → calculated distance is **smaller than reality** → boat thinks it hasn't gone 30m when it actually has. |
| **GPS position is already "good enough"** | GPS accuracy is ~1–3 meters. The 30m threshold has 10x margin. No need to smooth. |
| **Heading is noisy, position is not** | `courseDeg` (heading) jumps wildly at low speed (GPS calculates heading from position delta). Position itself is stable. Only heading needs smoothing. |
| **Lag kills the estimate** | If position lags by even 2 seconds at 1 m/s, that's 2 meters of error. At 30m threshold, that's 7% error. For heading, lag is acceptable because we're averaging direction over time. |

### What Actually Happens in Code

```cpp
// Line 112-114: Raw position stored ONCE at start (anchor point)
obsStartLat_ = pos.lat;      // RAW - never smoothed
obsStartLon_ = pos.lon;      // RAW - never smoothed

// Line 118-120: ONLY heading is smoothed (EMA)
float d = nav_relativeAngle(smoothHeading_, pos.courseDeg);
smoothHeading_ = nav_normalizeAngle(smoothHeading_ + 0.1 * d);

// Line 123: Distance uses RAW current position vs RAW start position
double dist = Navigator::distanceM(obsStartLat_, obsStartLon_, pos.lat, pos.lon);
```

**Analogy:** You're walking 30 meters blindfolded.
- **Position smoothing** = someone secretly shortens your measuring tape → you think you walked less than you did.
- **Heading smoothing** = you average which way your feet point over many steps → gives you a stable "average direction" despite wobbling.

We want accurate distance (raw position) but stable direction (smoothed heading).

---

## 9. Action Items

- [ ] Review wind observation EMA smoothing implementation and parameters (tracked in TODO)
- [ ] Add speed gate (minimum 0.5 km/h) before using `courseDeg`
- [ ] Add outlier rejection (reject course jumps > 60° from smoothed heading)
- [ ] Consider adaptive alpha (0.3 initially, 0.1 after 50% distance)
- [ ] Validate `WIND_OBS_DISTANCE_M = 30m` and `+90°` offset with field testing
- [ ] Document that wind observation has **never been tested on water** (Phase 5 status)

---

*This document captures the technical discussion for future reference. No code changes were made.*