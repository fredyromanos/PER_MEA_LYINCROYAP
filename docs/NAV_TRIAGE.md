# Navigation Triage — "aligned → rudder 45°" and the sim/firmware reconciliation

**Date:** 2026-09 · **Scope:** diagnosis only — `navigation.h` was NOT modified.

## Symptom

When the desktop simulator was pointed at the **current firmware** `navigation.h`
(instead of its stale local copy), the simplest scenario (S1, single waypoint, wind
abeam) **failed to converge**: in the `direct: waypoint aligned` branch the boat
commanded `rudder = −45°` and sailed *away* from the waypoint (947 → 949 m), spinning
in place. The stale local nav gave `rudder = 0` and converged. This made it look like
the newer navigation was broken.

## Root cause — it is NOT a firmware bug

The rudder command is intentionally decomposed by the firmware itself
(`main/src/navigation/navigation.h`):

```
nav_rudderCommand(correction, relWind) = nav_rudderCompensation(relWind) + clamp(correction)
nav_rudderCompensation(relWind)        = relWind / 2          // standing feed-forward
nav_rudderCorrection(command, relWind) = clamp(command − relWind/2)   // pure steering
```

The `relWind/2` term is a **standing wind-compensation feed-forward**: the rudder offset
that counteracts the sail's yaw moment so the boat **holds course**. With the wind abeam
(relWind ≈ 90°) the aligned command is `0 + 90/2 = 45°` — this is *course-holding*, not a
turn. On the real boat it maps through `AutoController::rudderToUs` (0.92 µs/°) to a small
winch offset that cancels the sail yaw → the boat goes straight.

The **simulator** diverged because its kinematics model **no sail-induced yaw moment**
(`sim_environment.cpp::updateBoatDynamics` derives turn rate purely from rudder angle).
So the compensating 45° offset, with nothing to cancel, spun the boat. The bug was in the
**simulator's fidelity**, not the navigation logic.

## Fix applied (simulator only)

Drive the simulated turn from the firmware's own **steering correction**, not the raw
command — treating the compensation as course-holding:

```cpp
// Simulation/simulation/sim_boat.cpp (navigate branch)
double relWind  = nav_relativeAngle(state.heading, windDirection);
float  steerDeg = nav_rudderCorrection((float)rudderAngle, relWind);  // ±20°, firmware fn
environment.setServoAngles(clampedSail, steerDeg);
```

The raw command is still fed back to `nav_*` next tick (as the firmware does; its
lofer/abattre integrator depends on it). The wind-observation branch is unchanged
(it already commands rudder-center = straight, matching the firmware).

Also: `Simulation/navigation.h` is retired to a redirect and the sim now compiles against
the firmware header (single source of truth) — see `Simulation/simulation/Makefile`.

## Evidence

- `nav_rudderCorrection(45°, relWind=90°) = clamp(45 − 45) = 0` → aligned ⇒ no turn. ✓
- After the fix, **all six** simulator scenarios reach their waypoints against the current
  firmware nav (`All waypoints reached!` ×6, including the 2-waypoint S5), covering direct,
  VDB, lofer, abattre and downwind cases.

## Verdict & recommendation

- **`navigation.h` is correct on this point — no change required.** The aligned-45° rudder
  is the intended course-holding feed-forward, confirmed by the firmware's own
  correction/compensation decomposition.
- **Open bench question (flagged, not a code bug):** whether `relWind/2` is the *correct
  magnitude* to cancel the real boat's sail yaw is a physical-tuning question. Validate on
  the bench/water and adjust `nav_rudderCompensation` only with the nav author's (Testautoboat
  branch owner) approval. The reconciled simulator is now the right tool to explore this
  before water trials.
- Next: bench-calibrate `SAIL_PLUS/MINUS_US` & `CH3_CENTER_US`, confirm ESC bidirectional
  (EPRG-3), then a staged manual→auto on-water trial.
