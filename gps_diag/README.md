# GPS diagnostic (standalone)

Independent from `main/`: nothing here includes or modifies the drone firmware.
It answers one question with evidence: **what GPS data does the navigation really receive?**

## What is checked, and how

| # | Claim | Proven on PC (`host_test`) | Measured on board |
|---|---|---|---|
| 1 | Once a fix was obtained, `location.isValid()` never returns to false; only `age()` grows | tests 2a-2c | `stale_fix` |
| 2 | Empty course field (receiver stopped) is reported as a **valid** course: the last course seen while moving, frozen | tests 3a-3d | `stale_course` |
| 3 | Never moved since boot: valid course = 0.0 (fake north). The library leaves `newval` uninitialised; static storage makes it 0 | tests 4a-4b | `stale_course` |
| 4 | Satellites/HDOP are updated even without a fix | test 5 | `reception` |
| 5 | Receiver dynamic model after the firmware's CFG-CFG reset (boat should use 5 = sea) | - | `dyn_model` |
| 6 | Below which speed the NEO-6M stops giving a course | - | `course_vs_speed` |
| 7 | Heading from two positions 1 s apart (legacy AutoBoat) is noise at low speed | - | `pos_heading_noise` |
| 8 | AXP192 init really succeeds (previously only inferred) | - | `axp_init` |

The board sketch sends **byte-identical UBX packets** to `main/src/drivers/GpsUart.cpp`,
so it reproduces the drone's exact receiver configuration.

## 1. Test bank (PC, no hardware, ~1 min)

```bash
./host_test/run.sh          # exit 0 = everything passes
```

Compiles the **real, unmodified** project sources (current driver, legacy `Gps.cpp`, AXP and
TinyGPSPlus libraries, legacy distance code) under AddressSanitizer/UBSan, runs the IHM JavaScript
in node and the IHM Python route builder, and checks the real 2024 on-site log and project documents.
Needs `g++`, `python3`; `node` optional. A build failure fails the run.

| ID | Claim | Tested on |
|---|---|---|
| T01 | 79 on-site positions thousands of km off = exact fractions of the real position (average with empty slots) | 2024 field log |
| T02 / T02b | Stationary boat: transmitted heading is noise over 0-359 / constant fake north 0 | 2024 field log |
| T03 | 952 rows with no fix sent as position [0, 0] | 2024 field log |
| T04 / T04b | 183 rows lat/lon swapped (2024-12-09); isolated invalid values | 2024 field log |
| T14 | 2024-12-11 navigation: waypoint never reached, rudder at +/-20 limit | 2024 field log |
| T05a-i | **Current driver (fixed)**: silent receiver → invalid after 2 s, "FIX LOST" logged, reported no-fix / <5 sats / HDOP >3 → invalid, auto-recovery, last position kept (no [0,0]) | `main/` GpsUart.cpp |
| T06a-h | **Current driver (fixed)**: no fake north at boot, course discarded below 1 km/h, course and speed NOT frozen after stopping, one sample per sentence | `main/` GpsUart.cpp |
| T09a-e | **Current driver (fixed)**: CFG-CFG + CFG-ANT + CFG-NAV5 dynModel 5 (sea), checksums valid | `main/` GpsUart.cpp |
| tests 1-7 | TinyGPSPlus: sticky `isValid()`, stale fields, uninitialised `newval` | TinyGPSPlus source |
| T08a-a3 | Legacy: GPS power never commanded (AXP without `begin()`), library returns AXP_NOT_INIT | legacy + AXP source |
| T08b | Legacy: zero bytes sent to the GPS (no CFG-ANT) | legacy `boat/Gps.cpp` |
| T08c | Legacy: waypoint due north -> bearing 180 | legacy `boat/Gps.cpp` |
| T05L | Legacy: status stays "valid" after fix loss | legacy `boat/Gps.cpp` |
| L01-L05 | Legacy filters: <3 m updates discarded, >25 m jump frozen, history wiped every 30 s, stationary heading noise, north bias at start | legacy `boat/Gps.cpp` |
| T10 / T10b | Minutes: GPS receiver broken spring 2026; spec: GPS reliability "Inconnu" | meeting minutes, spec |
| T11 | 2026 trial RETEX contain no GPS mention | RETEX documents |
| T12 | `gps_explication.tex` claims fix-loss detection (contradicted by T05) | project doc |
| T13 | `ADVERSARIAL_FINDINGS.md` reaches the same GPS-course conclusion | project doc |
| R1 | Ruled out: 2024-12-11 "reversed heading" is not a code bug | 2024 field log |
| R2a-c | Ruled out: waypoint lat/lon order correct IHM JS -> IHM Python -> firmware | real code, all 3 links |
| R3 | Ruled out: legacy distance calculation correct (Brest-Paris 505 km) | legacy `test.cpp` |
| R4 / R5 / R6 | Ruled out: setupGPS loop terminates; archive = repo; original `GPS/Gps.cpp` bearing correct | legacy code, archive |

**Sensitivity was verified** by applying fixes to scratch copies of the code: an age check made the
old T05 fail, argument-order fix makes T08c fail, `axp.begin()` makes T08a fail, and a course guard
based on the raw RMC field made the old T06a/T06c fail. A guard based on `course.isUpdated()` + speed
does **not** work: empty fields still mark the course updated and the speed stays frozen, so the fix
reads the raw NMEA field. The driver was then fixed that way and T05/T06/T09 now check the fixed
behaviour; the logic above the driver (HeadingGate, AutoController) is covered by
`PER_MEA_LYINCROYAP/test/test_gps_logic.cpp` (ctest).

On the board, after the fix the diagnostic also sends CFG-NAV5, so `dyn_model` is expected to report
`NOT OBSERVED` (dynModel = 5, sea).

## 2. Flash the diagnostic

**This replaces the firmware on the connected board.** Reflash `main/` afterwards.

```bash
cd gps_diag_board
~/bin/arduino-cli compile --fqbn esp32:esp32:t-beam .
~/bin/arduino-cli upload  --port /dev/ttyUSB0 --fqbn esp32:esp32:t-beam .
```

## 3. Field protocol (outside, clear sky, LiPo connected, antenna on u.FL)

Start `./capture.sh` and keep it running for the whole sequence.

| Phase | Duration | Action | Feeds verdicts |
|---|---|---|---|
| A | 3 min | Board still, on the ground, until fix | 3, 4, 5, 7, 8 |
| B | 1 min | Walk in a **straight line** at normal pace | 6 |
| C | 1 min | Stop, stand still | 2, 6, 7 |
| D | 1 min | Walk very slowly (~1 km/h) | 6 |
| E | 30 s | **Unplug the antenna** (or wrap it in foil) | 1 |
| F | 1 min | Reconnect, wait | 1 |

Do not power-cycle during the sequence: a reboot resets the library state.

## 4. Analyse

```bash
python3 analyze_log.py logs/gps_diag_YYYYMMDD_HHMMSS.log
```

Each line gives `CONFIRMED`, `NOT OBSERVED` or `NOT TESTED` with the numbers behind it.
`NOT TESTED` means a phase was missing from the capture, never that the claim is false.

## Output format

`H` lines describe the columns. `C` = status every 10 s, `D` = one line per RMC (1 Hz),
`T` = receiver text messages (antenna status). Raw NMEA fields and TinyGPSPlus values
are side by side on every `D` line.
