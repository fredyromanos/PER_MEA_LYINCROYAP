#!/usr/bin/env python3
"""
LoRa uplink test — verifies all drone commands end-to-end.

Drone  : /dev/ttyUSB0  (main firmware with DEBUG_MODE=1)
GND    : /dev/ttyUSB1  (transceiver sketch)

For each command the script fires 5 bursts at 200 ms spacing so that
at least one falls inside the drone's 550 ms RX window between 450 ms
heartbeat transmissions.  A command is PASS if the expected debug string
appears on the drone serial within CMD_TIMEOUT seconds.

Usage:  python3 lora_uplink_test.py [drone_port [gnd_port]]
"""

import serial
import threading
import time
import queue
import sys

DRONE_PORT  = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"
GND_PORT    = sys.argv[2] if len(sys.argv) > 2 else "/dev/ttyUSB1"
BAUD        = 115200
BOOT_WAIT   = 4.0   # seconds after DTR-reset for both boards to boot
BURST_COUNT = 5     # how many times to send each command
BURST_GAP   = 0.22  # seconds between bursts (< 550 ms RX window)
CMD_TIMEOUT = 4.0   # seconds to wait for drone echo after last burst

# (label, gnd_shorthand_command, expected_string_in_drone_serial)
TESTS = [
    ("navigate",     "navigate",              "[LORA] CMD: navigate"),
    ("stop",         "stop",                  "[LORA] CMD: stop"),
    ("home",         "home 48.380000,-4.490000", "[LORA] CMD: home"),
    ("waypoints",    "wpt 48.38,-4.49,48.39,-4.50", "[LORA] CMD: waypoints"),
    ("wind-command", "wind 225",              "[LORA] CMD: wind-command"),
    ("wind-obs",     "wind-obs",              "[LORA] CMD: wind-observation"),
    # restart last — it reboots the drone
    ("restart",      "restart",               "=== SeaDrone boot ==="),
]

# ------------------------------------------------------------------
drone_q: queue.Queue = queue.Queue()
_t0 = time.time()

def ts() -> str:
    return f"[{time.time() - _t0:6.2f}s]"

def drone_reader(ser: serial.Serial) -> None:
    """Background thread: pushes every drone serial line into drone_q."""
    while True:
        try:
            raw = ser.readline()
        except Exception as e:
            drone_q.put(f"[READER-ERR] {e}")
            break
        if raw:
            line = raw.decode("utf-8", errors="replace").rstrip()
            drone_q.put(line)
            # Live print every drone line with timestamp to stderr for diagnostics
            import sys as _sys
            _sys.stderr.write(f"{ts()} LIVE| {line}\n")
            _sys.stderr.flush()

def drain(timeout: float = 0.3) -> list[str]:
    """Return all lines available in drone_q within timeout seconds."""
    lines, deadline = [], time.time() + timeout
    while True:
        remaining = deadline - time.time()
        if remaining <= 0:
            break
        try:
            lines.append(drone_q.get(timeout=remaining))
        except queue.Empty:
            break
    return lines

def wait_for(keyword: str, timeout: float = CMD_TIMEOUT) -> tuple[bool, list[str]]:
    """Block until a drone line contains keyword, or timeout expires."""
    collected, deadline = [], time.time() + timeout
    while True:
        remaining = deadline - time.time()
        if remaining <= 0:
            break
        try:
            line = drone_q.get(timeout=remaining)
            collected.append(line)
            if keyword in line:
                return True, collected
        except queue.Empty:
            break
    return False, collected

# ------------------------------------------------------------------
def main() -> None:
    print("=== LoRa Uplink Test ===")
    print(f"  Drone : {DRONE_PORT}   GND : {GND_PORT}")
    print(f"  Burst : {BURST_COUNT}× per command, {BURST_GAP*1000:.0f} ms apart")
    print()

    try:
        drone_ser = serial.Serial(DRONE_PORT, BAUD, timeout=1)
    except serial.SerialException as e:
        sys.exit(f"Cannot open drone port: {e}")

    try:
        gnd_ser = serial.Serial(GND_PORT, BAUD, timeout=1)
    except serial.SerialException as e:
        drone_ser.close()
        sys.exit(f"Cannot open GND port: {e}")

    # Explicit DTR reset — works regardless of previous port state
    for ser in (drone_ser, gnd_ser):
        ser.setDTR(False)
        time.sleep(0.05)
        ser.setDTR(True)
        ser.reset_input_buffer()

    threading.Thread(target=drone_reader, args=(drone_ser,), daemon=True).start()

    print(f"{ts()} Ports open — reset issued, waiting {BOOT_WAIT:.0f}s for boot...")
    time.sleep(BOOT_WAIT)

    # Drain boot output
    boot = drain(0.5)
    gnd_boot = []
    while gnd_ser.in_waiting:
        line = gnd_ser.readline()
        if line:
            gnd_boot.append(line.decode("utf-8", errors="replace").rstrip())

    print(f"{ts()} Boot: {len(boot)} drone lines, {len(gnd_boot)} GND lines")
    for l in boot:
        print(f"       DRONE| {l}")
    for l in gnd_boot:
        print(f"       GND  | {l}")

    # Confirm drone heartbeat
    print(f"\n{ts()} Waiting for first drone heartbeat...")
    alive, hb = wait_for("[LORA] tx=", timeout=3.5)
    for l in hb:
        print(f"       DRONE| {l}")
    if not alive:
        print(f"WARNING: no heartbeat seen — check [LORA] init at boot")
    else:
        print(f"{ts()} Drone alive ✓")

    # ------------------------------------------------------------------
    results: list[tuple[str, bool]] = []

    for label, cmd, keyword in TESTS:
        print(f"\n{ts()} ── TEST: {label!r} ──")
        print(f"       CMD  | {cmd!r}  ×{BURST_COUNT}")

        # Fire bursts
        for i in range(BURST_COUNT):
            gnd_ser.write((cmd + "\n").encode())
            gnd_ser.flush()
            time.sleep(BURST_GAP)

        # Collect any GND echo lines
        time.sleep(0.1)
        while gnd_ser.in_waiting:
            line = gnd_ser.readline()
            if line:
                print(f"       GND  | {line.decode('utf-8', errors='replace').rstrip()}")

        # Wait for drone response
        ok, resp = wait_for(keyword, timeout=CMD_TIMEOUT)
        for l in resp:
            print(f"       DRONE| {l}")

        status = "PASS ✓" if ok else "FAIL ✗"
        print(f"       → {status}")
        results.append((label, ok))

        # For restart, wait for drone to reboot before continuing
        if label == "restart" and ok:
            print(f"{ts()} Waiting for drone reboot...")
            time.sleep(4.0)
            drain(0.5)

        time.sleep(1.0)

    # ------------------------------------------------------------------
    passed = sum(ok for _, ok in results)
    print(f"\n{ts()} ══════════════════════════════")
    print(f"{ts()} Results: {passed}/{len(results)} PASSED")
    for label, ok in results:
        print(f"       {'✓' if ok else '✗'} {label}")
    print(f"{ts()} ══════════════════════════════")

    drone_ser.close()
    gnd_ser.close()
    sys.exit(0 if passed == len(results) else 1)

if __name__ == "__main__":
    main()
