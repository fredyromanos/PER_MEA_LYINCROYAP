#!/usr/bin/env python3
"""
Fetch the real forecast wind (Windy) for a point and print shell exports for the
simulator:

    eval "$(python3 fetch_wind.py 48.34 -4.52)" && ./boat_simulator 1

Prints `export SIM_WIND_DIR=<deg from>` and `export SIM_WIND_SPEED=<m/s>` on stdout.
Errors go to stderr with a non-zero exit code (nothing is exported, so the simulator
falls back to each scenario's own wind).

Reuses IHM/app/windy.py (the same module behind GET /api/wind-forecast); the API key
comes from WINDY_API_KEY or IHM/.env and is never printed. Needs outbound network.
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "IHM", "app"))
import windy  # noqa: E402


def main(argv):
    if len(argv) != 3:
        print("usage: fetch_wind.py LAT LON", file=sys.stderr)
        return 2
    try:
        lat, lon = float(argv[1]), float(argv[2])
    except ValueError:
        print("fetch_wind.py: LAT/LON must be numbers", file=sys.stderr)
        return 2
    res = windy.get_wind_forecast(lat, lon)
    if "error" in res:
        print(f"fetch_wind.py: {res['error']}", file=sys.stderr)
        return 1
    print(f"export SIM_WIND_DIR={res['direction_deg']}")
    print(f"export SIM_WIND_SPEED={res['speed_ms']}")
    print(f"# Windy {res['model']} forecast at {lat},{lon}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
