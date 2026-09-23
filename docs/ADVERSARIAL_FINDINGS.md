# Adversarial Findings — where autonomous navigation breaks, and what must be measured

**Method.** An independent harness (`Simulation/adversarial/`) drives the **real firmware
`navigation.h`** through a boat model that injects the failure modes the "digital twin" omits,
each behind a parameter swept from 0 (twin's implicit assumption) outward. 200 random missions
per value (random start heading, wind direction, and waypoint bearing/distance 300–900 m).
Reproduce: `cd Simulation/adversarial && make sweep` (writes `adversarial.csv`).

**Read this as sensitivity, not verdict.** The disturbance *forms* (sail-yaw ∝ compensation·mismatch,
linear rudder authority, simple polar) are **illustrative, not measured**. The robust conclusions are
the **relative** drops from the same baseline; absolute percentages depend on model assumptions.

## Anti-fallacy controls (both passed)
- **Baseline control 6/6:** with all disturbances 0 and `yawMismatch=1`, the harness reproduces the
  twin's six scenarios (`make baseline`). It is **not** rigged to fail. (It actually *caught a harness
  bug* first — an uninitialised GPS course — which was fixed; that is the guardrail working.)
- **Discriminating power:** the sweeps clearly separate benign from breaking regimes (below), so the
  harness can fail — unlike the binary "reached the waypoint" it replaces.

## Results (%converged of 200 random missions)

| Disturbance | 0 (baseline) | → degradation | Severity |
|---|---|---|---|
| **Tidal current** | 73% | 0.25 m/s→**23%**, 0.5→20%, 1.0→10% | **CATASTROPHIC** |
| **Wind-estimate error** | 75% | 10°→58%, 20°→**39%**, 45°→2.5% (rudder thrashes) | **SEVERE** |
| **Sail-yaw compensation mismatch** | 78% | 0.75→44%, 0.5→13% ; 1.25→52%, 1.5→20% | **SEVERE** |
| **Open-loop winch slew (10–200°/s)** | 78% | 80–85% (no degradation) | negligible |
| **GPS noise (≤8 m / ≤8°)** | 72% | 65–82% (flat) | negligible |

### What this says
1. **Tidal current is almost certainly "the exact moment".** Adding just **0.25 m/s** of current
   cuts convergence by ~50 points. The mission is 5–30 m coastal water with strong tidal streams
   (often 0.5–2 m/s). The controller uses **GPS course-over-ground as heading**; under current,
   course ≠ where the sail points, and the tack/steer logic is fed a lie. The boat has no way to
   distinguish leeway/current from heading. **This is the single most likely on-water failure.**
2. **No wind sensor is a first-order risk.** Wind is *estimated* (GPS-track, +90°, single 30 m tack).
   A plausible 10–20° estimate error already halves convergence; 45° is fatal. Nothing in the current
   design bounds this error.
3. **The `relWind/2` compensation is an unvalidated magic number.** A ±25% mistune halves convergence.
   Its correct value depends on the real sail/rudder geometry, which has never been measured.
4. **Reassuringly, the winch slew rate and GPS noise are NOT the bottleneck** — effort should not go
   there first.
5. **Even at zero disturbance ~25% of random geometries did not converge in-model.** Caveat: the
   300 s "no-progress" stuck-detector can false-flag legitimately slow upwind VMG beats, so treat the
   *absolute* baseline cautiously. But it is a flag that the hand-picked 6 scenarios are not
   representative — a wider scenario set is warranted regardless.

## CANNOT be settled in software — required hardware measurements

The harness bounds fragility; it cannot close these unknowns. To make the sensitivity curves real,
measure on the bench/water:

1. **Winch µs → actual rudder-angle transfer function** (and its rate). The Regatta ECO II is
   open-loop multi-turn; map commanded `rotorUs` to physical rudder angle and slew rate. Feeds the
   `rudderToUs`/winch model — and decides whether a WinchTracker/position estimate is needed.
2. **Sail yaw moment vs relative wind angle** (with the aileron on each tack), to calibrate the
   `nav_rudderCompensation = relWind/2` term. This is the ±25%-sensitive magic number.
3. **Real wind-estimate accuracy** from a GPS-track run vs a handheld anemometer — quantify the
   actual error the +90°/30 m/EMA method produces, and map it onto the wind-error curve above.
4. **Speed polar** (boat speed vs wind angle/strength) — replaces the illustrative polar.
5. **On-site tidal current** at the test area (magnitude/direction vs tide phase) — the dominant
   disturbance; decide whether the controller must consume/estimate current (e.g. fuse heading from a
   compass/IMU instead of GPS course).

## Recommendation
Before any autonomous water trial: (a) add a **compass/IMU heading** source (or current estimation)
so control does not rely on GPS course under current — this is the highest-leverage change and directly
attacks the catastrophic sensitivity; (b) bench-measure items 1–2 to pin the winch and compensation;
(c) treat wind-estimate error as a first-class error budget. Do **not** invest first in winch-rate or
GPS-noise robustness — the data says they are not the bottleneck. `navigation.h` was not modified;
these are recommendations for its author (Testautoboat branch) plus a hardware/sensor decision.
