#!/usr/bin/env python3
"""Analyse a gps_diag_board serial log and give a verdict for each GPS claim.

Usage:  python3 analyze_log.py capture.log
Standard library only. Each verdict is one of:
  CONFIRMED     the log contains direct evidence of the behaviour
  NOT OBSERVED  the right conditions occurred and the behaviour did not happen
  NOT TESTED    the log lacks the conditions needed to decide (see hint)
"""
import math
import statistics
import sys

D_FIELDS = ["ms", "rmcStatus", "rawSpeedKn", "rawCourse", "ggaQuality", "ggaSats", "ggaHdop",
            "tgLocValid", "tgLocAgeMs", "tgCourseValid", "tgCourseUpdated", "tgCourseDeg",
            "tgSpeedKmh", "tgSpeedAgeMs", "lat", "lon", "posHeadingDeg", "posStepM"]
C_FIELDS = ["ms", "axpOk", "dynModel", "nav5Polls", "antStatus", "chars", "badCRC", "rmcCount"]
DYN_MODELS = {0: "portable", 2: "stationary", 3: "pedestrian", 4: "automotive", 5: "sea",
              6: "airborne <1g", 7: "airborne <2g", 8: "airborne <4g"}
KN_TO_KMH = 1.852


def parse(lines):
    d_rows, c_rows, bad = [], [], 0
    for raw in lines:
        line = raw.strip()
        if line.startswith("D,"):
            parts = line.split(",")[1:]
            if len(parts) != len(D_FIELDS):
                bad += 1
                continue
            d_rows.append(dict(zip(D_FIELDS, parts)))
        elif line.startswith("C,"):
            parts = line.split(",")[1:]
            if len(parts) != len(C_FIELDS):
                bad += 1
                continue
            c_rows.append(dict(zip(C_FIELDS, parts)))
    return d_rows, c_rows, bad


def verdicts(d_rows, c_rows):
    out = []   # list of (key, verdict, detail)

    # 1. AXP192 power init
    if not c_rows:
        out.append(("axp_init", "NOT TESTED", "no C lines (log shorter than 10 s?)"))
    elif all(r["axpOk"] == "1" for r in c_rows):
        out.append(("axp_init", "CONFIRMED", "AXP192 begin OK on every status line"))
    else:
        out.append(("axp_init", "NOT OBSERVED", "AXP192 begin FAILED - GPS rail not set by code"))

    # 2. Dynamic model actually used after the firmware's CFG-CFG reset
    models = sorted({int(r["dynModel"]) for r in c_rows if r["dynModel"] != "-1"})
    if not models:
        out.append(("dyn_model", "NOT TESTED", "receiver never answered the CFG-NAV5 poll"))
    else:
        names = ", ".join(f"{m} ({DYN_MODELS.get(m, 'unknown')})" for m in models)
        verdict = "CONFIRMED" if models != [5] else "NOT OBSERVED"
        out.append(("dyn_model", verdict, f"dynModel = {names}; boat model 'sea' is 5"))

    # 3. Fix lost but location still reported valid
    seen_fix = False
    lost_rows = []
    for r in d_rows:
        if r["rmcStatus"] == "A":
            seen_fix = True
        elif seen_fix and r["rmcStatus"] == "V":
            lost_rows.append(r)
    if not lost_rows:
        out.append(("stale_fix", "NOT TESTED",
                    "no fix loss after a fix; cover/unplug the antenna ~30 s during the capture"))
    else:
        stale = [r for r in lost_rows if r["tgLocValid"] == "1"]
        if stale:
            max_age = max(int(r["tgLocAgeMs"]) for r in stale)
            out.append(("stale_fix", "CONFIRMED",
                        f"{len(stale)}/{len(lost_rows)} rows with RMC status V still report "
                        f"location valid; position age reached {max_age} ms"))
        else:
            out.append(("stale_fix", "NOT OBSERVED", "location invalidated when fix was lost"))

    # 4. Empty course field reported as a valid course
    fix_rows = [r for r in d_rows if r["rmcStatus"] == "A"]
    empty_course = [r for r in fix_rows if r["rawCourse"] == ""]
    if not empty_course:
        out.append(("stale_course", "NOT TESTED", "receiver never sent an empty course while fixed"))
    else:
        valid_empty = [r for r in empty_course if r["tgCourseValid"] == "1"]
        if valid_empty:
            values = sorted({r["tgCourseDeg"] for r in valid_empty})
            shown = ", ".join(values[:6]) + (" ..." if len(values) > 6 else "")
            kind = ("fake north 0.00" if values == ["0.00"]
                    else "frozen value(s) from earlier motion")
            out.append(("stale_course", "CONFIRMED",
                        f"{len(valid_empty)}/{len(empty_course)} rows: course field empty but "
                        f"TinyGPSPlus reports valid course = {shown} ({kind})"))
        else:
            out.append(("stale_course", "NOT OBSERVED", "empty course never reported valid"))

    # 5. Speed below which the receiver stops giving a course
    with_speed = [r for r in fix_rows if r["rawSpeedKn"] != ""]
    if not with_speed:
        out.append(("course_vs_speed", "NOT TESTED", "no raw speed values; walk with the board"))
    else:
        bins = [(0, 1), (1, 2), (2, 4), (4, 1e9)]
        parts = []
        for lo, hi in bins:
            rows = [r for r in fix_rows
                    if r["rawSpeedKn"] != "" and lo <= float(r["rawSpeedKn"]) * KN_TO_KMH < hi]
            if rows:
                n_course = sum(1 for r in rows if r["rawCourse"] != "")
                label = f"{lo}-{hi:g}" if hi < 1e9 else f">={lo}"
                parts.append(f"{label} km/h: course given {n_course}/{len(rows)}")
        moving = any(float(r["rawSpeedKn"]) * KN_TO_KMH >= 2 for r in with_speed)
        verdict = "CONFIRMED" if moving else "NOT TESTED"
        hint = "" if moving else " (never above 2 km/h - walk at normal pace to complete)"
        out.append(("course_vs_speed", verdict, "; ".join(parts) + hint))

    # 6. Heading from successive 1 s positions (legacy AutoBoat method) at low speed
    # Use the RAW speed field: TinyGPSPlus keeps the last speed when the field is empty,
    # so tgSpeedKmh can still show walking speed long after the board has stopped.
    def raw_kmh(r):
        return 0.0 if r["rawSpeedKn"] == "" else float(r["rawSpeedKn"]) * KN_TO_KMH
    slow = [r for r in fix_rows if float(r["posHeadingDeg"]) >= 0 and raw_kmh(r) < 1.0]
    if len(slow) < 10:
        out.append(("pos_heading_noise", "NOT TESTED", "fewer than 10 slow rows with a position step"))
    else:
        angles = [math.radians(float(r["posHeadingDeg"])) for r in slow]
        s = sum(math.sin(a) for a in angles) / len(angles)
        c = sum(math.cos(a) for a in angles) / len(angles)
        resultant = math.hypot(s, c)                      # 1 = all same direction, 0 = random
        spread = (math.degrees(math.sqrt(-2 * math.log(resultant)))
                  if resultant > 1e-9 else float("inf"))
        steps = statistics.median(float(r["posStepM"]) for r in slow)
        verdict = "CONFIRMED" if spread > 45 else "NOT OBSERVED"
        out.append(("pos_heading_noise", verdict,
                    f"{len(slow)} slow rows: circular spread {spread:.0f} deg, "
                    f"median step {steps:.2f} m between 1 s positions"))

    # 7. Reception summary (informational)
    if fix_rows:
        sats = [int(r["ggaSats"]) for r in fix_rows if r["ggaSats"].isdigit()]
        hdops = [float(r["ggaHdop"]) for r in fix_rows if r["ggaHdop"] not in ("",)]
        bad = max((int(r["badCRC"]) for r in c_rows), default=0)
        out.append(("reception", "INFO",
                    f"{len(fix_rows)}/{len(d_rows)} RMC with fix; sats {min(sats, default=0)}-"
                    f"{max(sats, default=0)}; HDOP {min(hdops, default=0):.2f}-"
                    f"{max(hdops, default=0):.2f}; bad checksums {bad}"))
    else:
        out.append(("reception", "INFO", f"no fix in {len(d_rows)} RMC rows"))
    return out


def main(argv):
    if len(argv) != 2:
        print(__doc__)
        return 2
    with open(argv[1], encoding="utf-8", errors="replace") as f:
        d_rows, c_rows, bad = parse(f)
    print(f"{len(d_rows)} D rows, {len(c_rows)} C rows, {bad} malformed lines ignored\n")
    for key, verdict, detail in verdicts(d_rows, c_rows):
        print(f"{key:18s} {verdict:13s} {detail}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
