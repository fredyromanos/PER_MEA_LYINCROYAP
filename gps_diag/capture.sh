#!/bin/sh
# Records the diagnostic output to a timestamped log while showing it live.
# Usage: ./capture.sh [port]        (default /dev/ttyUSB0) - stop with Ctrl+C
set -e
PORT="${1:-/dev/ttyUSB0}"
LOG="$(dirname "$0")/logs/gps_diag_$(date +%Y%m%d_%H%M%S).log"
mkdir -p "$(dirname "$LOG")"
# -hupcl/clocal: do not toggle DTR/RTS, which would hold the ESP32 in reset
stty -F "$PORT" 115200 raw -echo -hupcl clocal
echo "Logging $PORT -> $LOG  (Ctrl+C to stop, then: python3 analyze_log.py $LOG)"
cat "$PORT" | tee "$LOG"
