#!/usr/bin/env python3
"""
Monitor two T-Beam serial ports simultaneously.
Usage: python3 monitor_both.py /dev/ttyUSB0 /dev/ttyUSB1
Press Ctrl-C to quit.
"""

import sys
import threading
import serial
import time

BAUD = 115200

def read_port(port, label, color_code):
    prefix = f"\033[{color_code}m[{label}]\033[0m "
    try:
        with serial.Serial(port, BAUD, timeout=1) as s:
            print(f"{prefix}Opened {port}")
            while True:
                line = s.readline()
                if line:
                    try:
                        text = line.decode("utf-8", errors="replace").rstrip()
                    except Exception:
                        text = repr(line)
                    print(f"{prefix}{text}")
    except serial.SerialException as e:
        print(f"{prefix}ERROR: {e}")

def main():
    if len(sys.argv) < 3:
        print("Usage: monitor_both.py <port_tx> <port_rx>")
        print("Example: monitor_both.py /dev/ttyUSB0 /dev/ttyUSB1")
        sys.exit(1)

    port_tx = sys.argv[1]
    port_rx = sys.argv[2]

    print(f"Monitoring TX={port_tx}  RX={port_rx}  (Ctrl-C to stop)")
    print("---")

    t1 = threading.Thread(target=read_port, args=(port_tx, "TX", "33"), daemon=True)
    t2 = threading.Thread(target=read_port, args=(port_rx, "RX", "36"), daemon=True)
    t1.start()
    t2.start()

    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nStopped.")

if __name__ == "__main__":
    main()
