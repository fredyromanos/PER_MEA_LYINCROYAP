# Adversarial Findings — where autonomous navigation breaks, and what must be measured

**Method.** An independent harness (`adversarial/`) drives the **real firmware `navigation.h`**
through a boat model that injects the failure modes the "digital twin" omits, each behind a
parameter swept from 0 (twin's implicit assumption) outward. **200 random missions per value**
(random start heading, wind direction, and waypoint bearing/distance 300–900 m). Reproduce:
`cd adversarial && make sweep` (writes `adversarial.csv`).

**Provenance of the numbers below.** Regenerated from a fresh `make sweep` run on
2026-09-30, on top of commit `529b698` (`HEAD`) plus the following **uncommitted working-tree
changes** present at run time: `adversarial/adversarial_main.cpp` (the common-random-numbers
fix, see next paragraph — **plus a second, later edit the same day that extends the
`winchDegPerS` sweep range to `{…,5,2,1,0.5}`**, see the winch section below for why),
`main/src/navigation/navigation.h` (21+/15− lines — owned by the
firmware author, not audited here), and the `simulation/` current model added by this pass
(Task 1/2, does not affect this binary). The harness was run twice back-to-back; both
`adversarial.csv` outputs are **byte-for-byte identical**, confirming the harness is
deterministic for this checkout. **These numbers do not match the previously-committed
version of this document, nor a verification table quoted when this task was scoped** — both
predate one or more of the uncommitted changes above (most likely `navigation.h`, which has a
real 21+/15− diff, not just whitespace). This is expected and is exactly why the numbers were
regenerated from a fresh run rather than reused: with a harness this sensitive to the nav code
under test, only "numbers from a run against the code you actually have" are trustworthy.
**Do not treat this doc as stable across future changes to `navigation.h` or the boat
model — rerun the sweep.**

**RNG check.** `adversarial/adversarial_main.cpp`'s `sweep()` seeds
`std::mt19937 rng(0xC0FFEE ^ std::hash<std::string>{}(name))` — the seed depends only on the
sweep's **name**, not on the swept value `v`. Every point within one sweep therefore replays
the **same 200 missions** and differs only by the injected parameter (common random numbers).
This is present and correct in the current working tree. It matters: seeding on `v` (the
older behaviour) confounds the parameter's effect with mission-set variation between points,
which is one of the reasons the numbers below differ from the pre-fix document (see the GPS
noise section).

**Read this as sensitivity, not verdict.** The disturbance *forms* (sail-yaw ∝ compensation·mismatch,
linear rudder authority, simple polar) are **illustrative, not measured**. The robust conclusions are
the **relative** drops from the same baseline; absolute percentages depend on model assumptions.
**Every percentage below has a 95% binomial confidence interval of roughly ±5 to ±7 percentage
points at N=200** (exact CI given per row) — do not read a 1–3pp difference between two points
as a real effect.

## Anti-fallacy controls (both passed)
- **Baseline control 6/6:** with all disturbances 0 and `yawMismatch=1`, the harness reproduces the
  twin's six scenarios (`make baseline`) — reverified on this run: `6/6 converged`. It is **not**
  rigged to fail.
- **Discriminating power:** the sweeps clearly separate benign from breaking regimes (below), so the
  harness can fail — unlike the binary "reached the waypoint" it replaces.
- **Determinism control:** two consecutive `make sweep` runs produced byte-identical
  `adversarial.csv` — the numbers below are not sampling noise from run to run at fixed code.

## Results (%converged of N=200 random missions, ±95% binomial CI)

### Tidal current (`currentMps`) — CATASTROPHIC, and possibly UNDERSTATED
| current (m/s) | %converged | 95% CI |
|---|---|---|
| 0.00 (baseline) | 73.5% | [67.4, 79.6] |
| 0.25 | 28.0% | [21.8, 34.2] |
| 0.50 | 18.0% | [12.7, 23.3] |
| 1.00 | 7.5%  | [3.8, 11.2] |
| 1.50 | 3.5%  | [1.0, 6.0] |

The 0→0.25 drop (73.5%→28.0%, CIs non-overlapping) is the single largest effect in this
campaign, at the **lowest** current value swept. The stated mission area is 5–30 m coastal
water with tidal streams typically **0.5–2 m/s** — i.e. the *entire realistic range* sits at or
above the point where convergence has already collapsed to under 30%.

### Wind-estimate error (`windErrDeg`) — SEVERE
| error (deg) | %converged | 95% CI |
|---|---|---|
| 0 (baseline) | 75.5% | [69.5, 81.5] |
| 5  | 73.0% | [66.8, 79.2] |
| 10 | 59.0% | [52.2, 65.8] |
| 15 | 50.0% | [43.1, 56.9] |
| 20 | 45.5% | [38.6, 52.4] |
| 30 | 28.5% | [22.2, 34.8] |
| 45 | 3.5%  | [1.0, 6.0] |

0→20° is a non-overlapping-CI drop of 30 points. There is no wind sensor on the boat — this
error is not a rare edge case, it is the normal operating condition of the GPS-track wind
estimator.

### Sail-yaw compensation mismatch (`yawMismatch`) — SEVERE
| yawMismatch | %converged | 95% CI |
|---|---|---|
| 1.00 (twin assumption) | 82.5% | [77.2, 87.8] |
| 0.90 | 82.5% | [77.2, 87.8] |
| 0.75 | 58.0% | [51.2, 64.8] |
| 0.50 | 5.5%  | [2.3, 8.7] |
| 0.25 | 0.5%  | [-0.5, 1.5] |
| 0.00 | 0.5%  | [-0.5, 1.5] |
| 1.25 | 65.0% | [58.4, 71.6] |
| 1.50 | 18.0% | [12.7, 23.3] |

A ±25% mistune (0.75 or 1.25) already costs 17–24 points; ±50% is close to total failure in
either direction. `nav_rudderCompensation = relWind/2` is an unvalidated magic number.

### Open-loop winch slew rate (`winchDegPerS`) — NOT the bottleneck ≥~2 deg/s, COLLAPSES below ~1 deg/s
**Range extended 2026-09-30.** The previously-published sweep (`{1e9,200,100,50,20,10}`) was
perfectly flat, but it never tested anything slower than 10 deg/s — it stopped before the
cliff, if one existed. The Regatta ECO II's real slew rate has **never been measured** (it is
item 1 on the required-hardware-measurements list below), so "flat over 10–1e9 deg/s" was true
only over the range tested, not a statement about the hardware. The sweep was extended to
`{1e9,200,100,50,20,10,5,2,1,0.5}` and regenerated from a fresh `make sweep` run (numbers below
are from that run; see provenance note above).

| rate (deg/s) | %converged | 95% CI |
|---|---|---|
| 1e9 (instant) | 76.0% | [70.1, 81.9] |
| 200 | 76.0% | [70.1, 81.9] |
| 100 | 76.0% | [70.1, 81.9] |
| 50  | 76.0% | [70.1, 81.9] |
| 20  | 76.0% | [70.1, 81.9] |
| 10  | 76.0% | [70.1, 81.9] |
| 5   | 76.0% | [70.1, 81.9] |
| 2   | 74.5% | [68.5, 80.5] |
| 1   | 40.0% | [33.2, 46.8] |
| 0.5 | 0.0%  | [0.0, 0.0]\* |

\*The Wald interval degenerates to zero width at 0/200; treat the true upper bound as
bounded well below 5% (rule-of-thumb ≈3/N ≈ 1.5%), not as "exactly 0% with certainty".

**Flatness test, recomputed over the plateau `{1e9,200,100,50,20,10,5}` (7 points, all exactly
152/200):** chi-square = **0.00, df = 6, critical(α=0.05) = 12.59 — not significant** (bit-for-bit
flat, same as the old range, just now confirmed one point further down). **Extending the
plateau to include 2 deg/s (8 points, 152×7 + 149):** chi-square = **0.21, df = 7, critical =
14.07 — still not significant**: 2 deg/s is statistically indistinguishable from instant.
**The cliff is between 2 and 1 deg/s:** 74.5%→40.0%, non-overlapping 95% CIs
([68.5,80.5] vs [33.2,46.8]) — a real, large effect, not noise. By 0.5 deg/s convergence is
total failure (0/200): the rudder physically cannot move fast enough to steer at all.
**A chi-square over the full 10-point range is hugely significant (chi2 = 506.4, df = 9,
critical = 16.92)** — but that significance is entirely driven by the 1 and 0.5 deg/s points,
not by anything happening in the 1e9–5 deg/s plateau, which remains genuinely flat.

**Corrected conclusion:** winch slew rates at or above ~5 deg/s are not the bottleneck
(statistically flat all the way down to 2 deg/s, in fact); below ~2 deg/s convergence
collapses, reaching total failure by 0.5 deg/s. **The real winch's rudder-angle slew rate has
never been measured.** This is not "negligible" in general — it is negligible *if* the real
hardware clears roughly 5 deg/s, and catastrophic if it does not. The bench measurement (item 1
below) must confirm which side of that line the boat is actually on before this conclusion can
be trusted on the water.

**For calibration/context only — an ESTIMATE, not a measurement, and not reassurance:** the
firmware maps nav rudder command ±110° roughly 1:1 onto physical winch degrees, clamped to
`ROTOR_AUTO_MIN_US`/`ROTOR_AUTO_MAX_US` = 1399/1601 µs (`Calibration.h`) — i.e. ±101 µs out of
the winch's 1000 µs / 6-turn (2160°) full travel, about 0.6 turn. *If* the winch can traverse
its full ~6-turn range in on the order of ~10 s (an illustrative assumption, not a datasheet
citation or a bench result), covering ~0.6 turn would take on the order of ~1 s, i.e. a slew
rate of order 100 deg/s — comfortably above the ~5 deg/s cliff found above. This is a
back-of-envelope plausibility check, nothing more: it does not substitute for measuring the
actual winch under actual rudder load, and should not be read as evidence the hardware is fine.

### GPS noise (`gpsPosM`, course noise scaled with it) — recomputed post RNG-fix
| pos noise (m) | %converged | 95% CI |
|---|---|---|
| 0 (baseline) | 73.0% | [66.8, 79.2] |
| 1 | 73.0% | [66.8, 79.2] |
| 3 | 71.5% | [65.2, 77.8] |
| 5 | 69.5% | [63.1, 75.9] |
| 8 | 61.5% | [54.8, 68.2] |

**Flatness test:** counts 146/146/143/139/123 out of 200. Homogeneity chi-square across the 5
groups: **chi2 = 8.74, df = 4, critical(α=0.05) = 9.49 — technically not significant, but
close to the threshold (92% of critical) and, unlike the pre-fix run, now monotonically
decreasing** (more noise → lower convergence — the physically expected direction). The
pre-fix document's claim "GPS noise is negligible" was built on a run with the confounded RNG
(seeding on `v`), which gave chi2=18.83 (df=4, crit=9.49 — **not flat**) with a
physically-backwards shape (73.0→82.5→65.0, i.e. *more* noise apparently improving
convergence). That claim was **not supported** even on its own numbers.
**Honest statement for the post-fix run: this sweep cannot conclusively resolve GPS
sensitivity at N=200.** The trend is now in the right direction and borderline-significant,
but "borderline, right direction" is not the same claim as "flat" or "fine" — do not round
this down to "GPS noise is not a problem." A larger N (or a tighter per-mission pairing) would
be needed to confirm or rule out the mild monotonic drop.

## Digital twin now models current too (Task 1/2 of this pass)
The simulator (`simulation/`) previously had no current and no leeway: it integrated position
as pure `speed·sin/cos(heading)` and set the simulated GPS `courseDeg = heading + noise`, so
course-over-ground was *always* identical to heading — the one disturbance this harness finds
catastrophic could not even be represented in the twin. `simulation/sim_environment.cpp` now
adds `currentSpeedMs`/`currentDirDeg` (default **0.0 = no current**, so the twin's six
regression scenarios reproduce bit-for-bit unless current is explicitly switched on — reverified:
`cd simulation && make && ./boat_simulator` still prints `[NAV-REAL] All waypoints reached!`
six times) and computes a genuine velocity-over-ground = through-water velocity + current
vector; the simulated GPS course/speed-valid gate (mirroring the firmware's `HeadingGate`
threshold) is now driven by that ground vector, not by heading, so a boat nearly stationary
through the water but drifting on current now correctly shows a valid, decoupled GPS course —
exactly the case this harness's `currentMps` sweep is built to punish. The twin and the harness
no longer disagree about whether course-over-ground can decouple from heading. A standalone
check (`setCurrent(90°, 1.0 m/s)` on a boat headed due north at 2 m/s through the water)
confirmed course-over-ground and heading genuinely diverge by tens of degrees, in the direction
and magnitude vector addition predicts, and that a near-zero through-water speed with strong
current still reports a valid, current-direction-dominated GPS course. Running the twin's six
scenarios with this current enabled (0.25 and 0.5 m/s) did not, by itself, break convergence
for most of them at the one fixed test direction tried, but did materially slow scenario 3
(time-to-converge 340s→890s) and materially sped up others whose route happened to run with
the current — a direction-dependent effect, as expected. Pushing an adverse current on the
slowest-affected scenario (3) to 0.75–2.0 m/s did break convergence outright (closest approach
grew to 9.8–20.9 km instead of shrinking). Full table and commands in the task output, not
reproduced here since this document is about the adversarial harness, not the twin. **This is
a further reason current is possibly understated above**: a fixed, single-direction demonstration
on hand-picked scenarios is a much weaker test than this harness's random-direction sweep, and
even that weaker test could be made to fail — nothing in `navigation.h` changed to compensate
for current, so the underlying vulnerability is demonstrated, not mitigated.

### What this says
1. **Tidal current is almost certainly "the exact moment," and this run suggests the original
   estimate undersold it.** Convergence collapses by 45+ points at the *lowest* current value
   swept (0.25 m/s), inside the low end of the mission area's realistic 0.5–2 m/s tidal
   streams. The controller uses **GPS course-over-ground as heading**; under current, course ≠
   where the sail points, and the tack/steer logic is fed a lie. The boat has no way to
   distinguish leeway/current from heading. **This is the single most likely on-water
   failure**, and the twin can now reproduce the mechanism (see above), even though its own
   six hand-picked scenarios under one fixed current direction did not all fail outright —
   consistent with, not contradicting, "understated": a wider/random direction sweep on the
   twin would be expected to find failures the fixed-direction demo did not hit.
2. **No wind sensor is a first-order risk.** Wind is *estimated* (GPS-track, +90°, single 30 m
   tack). A plausible 10–20° estimate error already costs 15–30 points; 45° is fatal. Nothing
   in the current design bounds this error.
3. **The `relWind/2` compensation is an unvalidated magic number.** A ±25% mistune costs
   17–24 points; ±50% is close to total failure either direction. Its correct value depends on
   the real sail/rudder geometry, which has never been measured.
4. **Winch slew rate is not the bottleneck — but only if the real hardware clears ~5 deg/s,
   which is unmeasured.** The range was extended 2026-09-30 to find the cliff instead of
   stopping before it: flat and statistically indistinguishable from instant down to 2 deg/s
   (chi2=0.21, df=7, ns), then a real collapse between 2 and 1 deg/s (74.5%→40.0%,
   non-overlapping CIs), total failure by 0.5 deg/s. Effort should not go there first *provided*
   the bench measurement (item 1 below) confirms the winch clears the cliff — until then this is
   an open risk, not a closed one.
5. **GPS position/course noise: inconclusive, not "fine."** Post RNG-fix, the sweep is
   monotonic in the physically correct direction but borderline-significant (chi2=8.74 vs
   crit=9.49, df=4). The pre-fix "negligible" claim was never supported (its own chi2=18.83
   was significant, with a backwards shape) — it was an artifact of the confounded RNG, not a
   real finding. The correct statement today is that this sweep **cannot resolve** GPS
   sensitivity either way; it does not clear GPS noise as a non-issue.
6. **Even at zero disturbance ~25% of random geometries did not converge in-model.** Caveat: the
   300 s "no-progress" stuck-detector can false-flag legitimately slow upwind VMG beats, so treat the
   *absolute* baseline cautiously. But it is a flag that the hand-picked 6 scenarios are not
   representative — a wider scenario set is warranted regardless.

## Anti-fallacy note on illustrative forms
The disturbance forms in `adversarial/boat_model.hpp` (sail-yaw ∝ compensation·yawMismatch,
linear rudder authority, simple polar) are **illustrative, not measured** — see the file's own
header comment. The digital twin's own surge-dynamics force balance
(`simulation/sim_environment.cpp`, Stage 1) is similarly illustrative and tuned to match the
old kinematic top speed, not bench-measured. **The current/leeway model added to the twin in
this pass is illustrative too**: the leeway term reuses the gain and cap
(`LEEWAY_GAIN=0.040`, `MAX_LEEWAY_SPEED_MPS=0.35`) recorded in
`docs/SIMULATION_PHYSICS_REVIEW.md`'s Appendix as "salvaged, not measured," adapted to the
scalar true-wind relative angle already in that file rather than that appendix's full
apparent-wind vector (a further simplification, to keep the change minimal); it defaults to
0 (disabled) and nothing in this pass turned it on for the reported runs. None of these
illustrative forms should be read as validated physics — they exist to find sensitivity, not
to certify safety.

## CANNOT be settled in software — required hardware measurements

The harness bounds fragility; it cannot close these unknowns. To make the sensitivity curves real,
measure on the bench/water:

1. **Winch µs → actual rudder-angle transfer function, and its slew rate — now with a specific
   number to beat: ~5 deg/s.** The Regatta ECO II is open-loop multi-turn; map commanded
   `rotorUs` to physical rudder angle and slew rate. Feeds the `rudderToUs`/winch model — and
   decides whether a WinchTracker/position estimate is needed. The extended sweep (see Winch
   section above) shows convergence flat down to 2 deg/s and collapsing between 2 and 1 deg/s,
   with total failure by 0.5 deg/s — so this is no longer an open-ended "measure it someday"
   item, it is a pass/fail bench check: **confirm the real winch can slew the rudder at or
   above ~5 deg/s** (a safety margin above the 2 deg/s statistical plateau edge). If it cannot,
   winch rate moves from "deprioritized" back to a first-order risk. (Still lower priority to
   *schedule* than items 2–5 below in the sense that a plausible geometric estimate — see the
   winch section's ESTIMATE note — suggests the real winch is likely fast enough; but "likely"
   is not "confirmed", and this is now a cheap, well-defined bench check, not vague follow-up.)
2. **Sail yaw moment vs relative wind angle** (with the aileron on each tack), to calibrate the
   `nav_rudderCompensation = relWind/2` term. This is the ±25%-sensitive magic number.
3. **Real wind-estimate accuracy** from a GPS-track run vs a handheld anemometer — quantify the
   actual error the +90°/30 m/EMA method produces, and map it onto the wind-error curve above.
4. **Speed polar** (boat speed vs wind angle/strength) — replaces the illustrative polar.
5. **On-site tidal current** at the test area (magnitude/direction vs tide phase) — the dominant
   disturbance; decide whether the controller must consume/estimate current (e.g. fuse heading from a
   compass/IMU instead of GPS course). The twin can now simulate a chosen current for regression
   testing (`simulation/`, Task 1/2) but that is not a substitute for measuring the real thing.
6. **GPS course noise in practice** (NEO-6M, real antenna, real sea state) — the software sweep
   above could not conclusively resolve sensitivity even at N=200; a real-world characterization
   would settle it directly instead of waiting for a larger simulated N.

## Recommendation
Before any autonomous water trial: (a) add a **compass/IMU heading** source (or current estimation)
so control does not rely on GPS course under current — this is the highest-leverage change and directly
attacks the catastrophic sensitivity; (b) bench-measure items 2–3 above to pin the compensation and
wind-estimate error, which this run confirms are both severe; (c) treat wind-estimate error as a
first-class error budget. Do **not** invest first in winch-rate robustness **on the condition that
the bench measurement (hardware item 1) confirms the real winch clears ~5 deg/s of rudder-angle slew** —
the software sweep shows convergence flat down to 2 deg/s and collapsing below that, so *if* the
hardware clears ~5 deg/s the software risk really is closed, but that "if" is currently unverified.
Until the bench measurement lands, winch-rate robustness is deprioritized provisionally, not
retired — treat it as an open risk with a known, cheap test to close it, not a settled question.
Do not deprioritize GPS-noise work on the strength of this
sweep either — it is inconclusive, not cleared. `navigation.h` was not modified by this campaign;
these are recommendations for its author plus a hardware/sensor decision.
