#!/usr/bin/env python3
"""Evidence tests on REAL on-site telemetry: all_messages.log (12 trial days, Oct-Dec 2024).
Claims #1-#4 (what went wrong on site) and ruled-out check R1."""
import json
import math
import re
import statistics
import sys
import unittest
from fractions import Fraction

from paths import FIELD_LOG

LINE = re.compile(r"(\d{4}-\d{2}-\d{2}) (\d{2}):(\d{2}):(\d{2}), (\{.*\})\s*$")


def load():
    rows = []
    for line in open(FIELD_LOG, encoding="utf-8", errors="replace"):
        m = LINE.match(line)
        if not m:
            continue
        try:
            msg = json.loads(m.group(5)).get("message")
        except json.JSONDecodeError:
            continue
        if not isinstance(msg, dict):
            continue
        loc = msg.get("location")
        if not (isinstance(loc, list) and len(loc) == 2):
            continue
        t = int(m.group(2)) * 3600 + int(m.group(3)) * 60 + int(m.group(4))
        rows.append({"day": m.group(1), "t": t, "lat": loc[0], "lon": loc[1],
                     "heading": msg.get("heading"), "mode": msg.get("mode"),
                     "servos": msg.get("servos") or {}, "wp": msg.get("waypoints") or {}})
    return rows


def in_brest(r):
    return 48.0 < r["lat"] < 48.7 and -4.8 < r["lon"] < -4.3


def dist_m(a, b):
    la1, lo1, la2, lo2 = map(math.radians, (a["lat"], a["lon"], b["lat"], b["lon"]))
    return 6371000 * math.hypot((lo2 - lo1) * math.cos((la1 + la2) / 2), la2 - la1)


def bearing(a, b):
    la1, lo1, la2, lo2 = map(math.radians, (a["lat"], a["lon"], b["lat"], b["lon"]))
    y = math.sin(lo2 - lo1) * math.cos(la2)
    x = math.cos(la1) * math.sin(la2) - math.sin(la1) * math.cos(la2) * math.cos(lo2 - lo1)
    return (math.degrees(math.atan2(y, x)) + 360) % 360


# The real on-site telemetry log lives in a sibling tree (AutoBoat_VN-1/Python)
# that is not part of every checkout (this one included). Loading it is a
# top-level side effect, so it must not run at import time — pytest's import
# machinery would otherwise turn a missing file into a collection error for
# the whole module. Skip cleanly instead of failing when the data is absent.
_SKIP_REASON = (
    None if FIELD_LOG.exists()
    else f"field log not found: {FIELD_LOG} (sibling tree AutoBoat_VN-1 absent from this checkout)"
)

if _SKIP_REASON:
    if "pytest" in sys.modules:
        import pytest
        pytest.skip(_SKIP_REASON, allow_module_level=True)
    else:
        print(f"SKIPPED: {_SKIP_REASON}")
        sys.exit(0)

ROWS = load()


class FieldLog(unittest.TestCase):
    def test_T00_log_loaded(self):
        """T00 control: the field log parses into ~27 000 positioned telemetry rows over 12 days"""
        self.assertGreater(len(ROWS), 27000)
        self.assertEqual(len({r["day"] for r in ROWS}), 12)

    def test_T01_zero_average_ramp(self):
        """T01 79 positions thousands of km away are exact fractions of the real position (average with empty slots)"""
        ramp, fractions = [], {}
        for i, r in enumerate(ROWS):
            if 0 < r["lat"] < 47.5 and -4.6 < r["lon"] < 0:
                ref = next((q for q in ROWS[i + 1:i + 15] if in_brest(q)), None)
                self.assertIsNotNone(ref, f"no real position after {r}")
                rla, rlo = r["lat"] / ref["lat"], r["lon"] / ref["lon"]
                frac = Fraction(rla).limit_denominator(15)
                self.assertLess(abs(rla - rlo), 0.002, f"lat/lon ratios differ at {r}")
                self.assertLess(abs(float(frac) - rla), 0.002, f"not a simple fraction at {r}")
                ramp.append(r)
                fractions[str(frac)] = fractions.get(str(frac), 0) + 1
        self.assertEqual(len(ramp), 79)
        self.assertEqual(set(fractions), {"1/5", "2/5", "3/5", "4/5", "1/3", "14/15"})
        worst = max(dist_m(r, {"lat": 48.3604, "lon": -4.5656}) for r in ramp) / 1000
        self.assertGreater(worst, 4000, "the ramp puts the boat thousands of km away")

    def test_T02_stationary_heading_noise(self):
        """T02 boat stationary (median scatter <10 m): transmitted heading is noise across 0-359 deg"""
        checked = 0
        for day in sorted({r["day"] for r in ROWS}):
            st = [r for r in ROWS if r["day"] == day and r["mode"] == "setup-ready" and in_brest(r)
                  and isinstance(r["heading"], (int, float)) and r["heading"] != 999]
            if len(st) < 50:
                continue
            c = {"lat": statistics.median(r["lat"] for r in st), "lon": statistics.median(r["lon"] for r in st)}
            if statistics.median(dist_m(c, r) for r in st) >= 10:
                continue
            hs = {r["heading"] for r in st}
            if len(hs) == 1:          # 2024-11-06: constant heading 0, see test_T02b
                continue
            checked += 1
            self.assertGreaterEqual(len(hs), 50, f"{day}: only {len(hs)} heading values")
            self.assertGreater(max(hs) - min(hs), 300, f"{day}: heading span too small")
        self.assertGreaterEqual(checked, 4)

    def test_T02b_constant_fake_north(self):
        """T02b 2024-11-06: 302 stationary rows, heading constant 0 (fake north), identical position"""
        day = [r for r in ROWS if r["day"] == "2024-11-06"]
        self.assertEqual(len(day), 302)
        self.assertEqual({r["heading"] for r in day}, {0})
        self.assertEqual(len({(r["lat"], r["lon"]) for r in day}), 1)

    def test_T03_zero_positions(self):
        """T03 no-fix sent as a real position: 952 rows at [0.0, 0.0]"""
        self.assertEqual(sum(1 for r in ROWS if r["lat"] == 0 and r["lon"] == 0), 952)

    def test_T04_swapped_lat_lon(self):
        """T04 183 rows on 2024-12-09 have latitude and longitude swapped"""
        sw = [r for r in ROWS if -4.8 < r["lat"] < -4.3 and 48.0 < r["lon"] < 48.7]
        self.assertEqual(len(sw), 183)
        self.assertEqual({r["day"] for r in sw}, {"2024-12-09"})

    def test_T04b_isolated_invalid(self):
        """T04b isolated invalid positions remain (sign lost on longitude, near-zero values): cause not in log"""
        other = [r for r in ROWS if not in_brest(r) and not (r["lat"] == 0 and r["lon"] == 0)
                 and not (0 < r["lat"] < 47.5 and -4.6 < r["lon"] < 0)
                 and not (-4.8 < r["lat"] < -4.3 and 48.0 < r["lon"] < 48.7)]
        self.assertTrue(any(r["lon"] > 4.3 and 48 < r["lat"] < 48.7 for r in other), "sign-lost longitude")
        self.assertGreaterEqual(len(other), 3)

    def test_T14_waypoint_never_reached_rudder_saturated(self):
        """T14 2024-12-11 navigation: waypoint index stays 0 and rudder at its +/-20 limit most of the time"""
        nav = [r for r in ROWS if r["day"] == "2024-12-11" and r["mode"] == "navigate" and in_brest(r)]
        self.assertEqual(len(nav), 599)
        self.assertEqual({r["wp"].get("current") for r in nav}, {0})
        saturated = sum(1 for r in nav if abs(r["servos"].get("rudder", 0)) == 20)
        self.assertGreaterEqual(saturated, 400)

    def test_R1_reversed_heading_not_a_bug(self):
        """R1 ruled out: 2024-12-11 'reversed heading' - same code matches track to <5 deg at 16:24; reversed part at >6 km/h"""
        nav = [r for r in ROWS if r["day"] == "2024-12-11" and r["mode"] == "navigate" and in_brest(r)
               and isinstance(r["heading"], (int, float))]

        def segment(t0, t1):
            s = [r for r in nav if t0 <= r["t"] <= t1]
            errs, speeds = [], []
            for i, a in enumerate(s):
                j = next((k for k in range(i + 1, len(s)) if s[k]["t"] - a["t"] >= 10), None)
                if j is None or dist_m(a, s[j]) < 5:
                    continue
                b = s[j]
                speeds.append(dist_m(a, b) / (b["t"] - a["t"]) * 3.6)
                errs.append(abs((s[(i + j) // 2]["heading"] - bearing(a, b) + 180) % 360 - 180))
            return errs, speeds

        good_err, _ = segment(16 * 3600 + 24 * 60, 16 * 3600 + 26 * 60)
        rev_err, rev_speed = segment(15 * 3600 + 45 * 60, 15 * 3600 + 55 * 60)
        self.assertLess(statistics.median(good_err), 5)
        self.assertGreater(statistics.median(rev_err), 135)
        self.assertGreater(statistics.median(rev_speed), 6)


if __name__ == "__main__":
    unittest.main(verbosity=2)
