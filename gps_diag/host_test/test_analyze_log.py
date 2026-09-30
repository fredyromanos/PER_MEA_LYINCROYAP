#!/usr/bin/env python3
"""Tests analyze_log.py verdicts on synthetic logs covering each case."""
import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import analyze_log as A

fails = 0
def check(ok, name):
    global fails
    print(f"[{'PASS' if ok else 'FAIL'}] {name}")
    fails += 0 if ok else 1

def D(ms, st, spd, crs, sats="07", hdop="1.20", lv="1", age="100", cv="1", cu="1",
      cdeg="0.00", kmh="0.00", phdg="-1.0", pstep="-1.00"):
    return (f"D,{ms},{st},{spd},{crs},1,{sats},{hdop},{lv},{age},{cv},{cu},{cdeg},{kmh},100,"
            f"48.3604150,-4.5666000,{phdg},{pstep}")
def C(axp="1", dyn="0", bad="0"):
    return f"C,10000,{axp},{dyn},1,OK,5000,{bad},10"

def run(lines):
    d, c, bad = A.parse(lines)
    return {k: (v, det) for k, v, det in A.verdicts(d, c)}, bad


def _run_scenarios():
    """Runs every check() scenario against analyze_log.py; returns the fail count.

    Kept as a plain function (not top-level module code) so importing this file
    — e.g. under pytest collection — has no side effects. Both the __main__
    entry point below and the pytest wrapper call this same logic.
    """
    global fails
    fails = 0

    # Scenario 1: static board, never moved, then fix lost.
    static = [C()] + [D(i * 1000, "A", "", "", cdeg="0.00") for i in range(20)]
    static += [D(20000 + i * 1000, "V", "", "", lv="1", age=str(1000 * (i + 1))) for i in range(5)]
    v, _ = run(static)
    check(v["axp_init"][0] == "CONFIRMED", "axp ok detected")
    check(v["dyn_model"][0] == "CONFIRMED" and "portable" in v["dyn_model"][1], "portable model reported")
    check(v["stale_fix"][0] == "CONFIRMED" and "5000 ms" in v["stale_fix"][1], "stale fix confirmed with age")
    check(v["stale_course"][0] == "CONFIRMED" and "fake north" in v["stale_course"][1], "fake north detected")
    check(v["course_vs_speed"][0] == "NOT TESTED", "no speed data -> not tested")

    # Scenario 2: moving then stopping -> frozen course, no fix loss.
    move = [C(dyn="5")]
    move += [D(i * 1000, "A", "2.00", "87.50", cdeg="87.50", kmh="3.70", phdg="88.0", pstep="1.9")
             for i in range(10)]
    # tgSpeedKmh stays 3.70 while stopped: TinyGPSPlus freezes it (host test 3d).
    move += [D(10000 + i * 1000, "A", "0.05", "", cdeg="87.50", kmh="3.70",
               phdg=str((i * 97) % 360), pstep="0.4") for i in range(12)]
    v, _ = run(move)
    check(v["dyn_model"][0] == "NOT OBSERVED", "sea model -> not observed")
    check(v["stale_fix"][0] == "NOT TESTED", "no fix loss -> not tested")
    check(v["stale_course"][0] == "CONFIRMED" and "frozen" in v["stale_course"][1], "frozen course detected")
    check(v["course_vs_speed"][0] == "CONFIRMED" and "0-1 km/h: course given 0/12" in v["course_vs_speed"][1]
          and "2-4 km/h: course given 10/10" in v["course_vs_speed"][1],
          "course vs speed bins")
    check(v["pos_heading_noise"][0] == "CONFIRMED" and "12 slow rows" in v["pos_heading_noise"][1],
          "slow rows found via RAW speed despite frozen tgSpeedKmh; noise confirmed")

    # Scenario 3: behaviour absent -> NOT OBSERVED, never a false CONFIRMED.
    clean = [C(axp="0")]
    clean += [D(i * 1000, "A", "0.05", "", cv="0") for i in range(5)]
    clean += [D(5000 + i * 1000, "V", "", "", lv="0") for i in range(3)]
    clean += [D(9000 + i * 1000, "A", "0.05", "", cv="0", kmh="0.09", phdg="45.0", pstep="0.3") for i in range(12)]
    v, _ = run(clean)
    check(v["axp_init"][0] == "NOT OBSERVED", "axp failure reported")
    check(v["stale_fix"][0] == "NOT OBSERVED", "location invalidated -> not observed")
    check(v["stale_course"][0] == "NOT OBSERVED", "course not valid when empty -> not observed")
    check(v["pos_heading_noise"][0] == "NOT OBSERVED", "consistent headings -> not observed")

    # Robustness: malformed and foreign lines are ignored, not crashing.
    v, bad = run(["garbage", "D,1,2,3", "[GPS ] lat=48", "T,1,$GPTXT", C(), "C,1"])
    check(bad == 2 and v["reception"][0] == "INFO", "malformed lines counted and skipped")

    # Empty log never claims anything.
    v, _ = run([])
    check(all(verdict in ("NOT TESTED", "INFO") for verdict, _ in v.values()), "empty log -> nothing confirmed")

    return fails


def test_analyze_log_scenarios():
    """pytest entry point: runs the exact same scenarios as direct execution."""
    n = _run_scenarios()
    assert n == 0, f"{n} analyze_log check(s) failed (see stdout for which)"


if __name__ == "__main__":
    n = _run_scenarios()
    print(f"\n{'0 failed' if not n else f'{n} FAILED'}")
    sys.exit(1 if n else 0)
