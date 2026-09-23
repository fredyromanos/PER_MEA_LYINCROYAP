#!/usr/bin/env python3
"""
Monitor a single T-Beam serial port with timestamps.
Usage: python3 monitor_drone.py /dev/ttyUSB0
Press Ctrl-C to quit.
"""

import sys
import serial
import time

BAUD = 115200

def main():
    if len(sys.argv) < 2:
        print("Usage: monitor_drone.py <port>")
        print("Example: monitor_drone.py /dev/ttyUSB0")
        sys.exit(1)

    port = sys.argv[1]
    start = time.time()

    print(f"Monitoring {port} at {BAUD} baud  (Ctrl-C to stop)")
    print("---")

    with serial.Serial(port, BAUD, timeout=1) as s:
        while True:
            line = s.readline()
            if line:
                try:
                    text = line.decode("utf-8", errors="replace").rstrip()
                except Exception:
                    text = repr(line)
                ts = time.time() - start
                print(f"[{ts:7.2f}] {text}")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopped.")
    except serial.SerialException as e:
        print(f"ERROR: {e}")
        sys.exit(1)
