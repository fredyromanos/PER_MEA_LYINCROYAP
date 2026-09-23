#!/bin/sh
# GPS test bank: compiles the REAL project sources on the PC and runs every check.
# Usage: ./run.sh            (from anywhere)   Exit code 0 = all tests pass.
# Needs: g++, python3; node optional (R2a skipped without it).
# Override locations: TINYGPS_DIR, AXP_DIR, S9P_ROOT.
cd "$(dirname "$0")" || exit 2
LIB="$HOME/Arduino/libraries"
TG="${TINYGPS_DIR:-$LIB/TinyGPSPlus/src}"
AXP="${AXP_DIR:-$LIB/AXP202X_Library/src}"
ROOT="${S9P_ROOT:-$(cd ../../../.. && pwd)}"
MAIN="$ROOT/03_Developpement_logiciel/PER_MEA_LYINCROYAP/main/src"
VN1="$ROOT/03_Developpement_logiciel/Informatique/AutoBoat_VN-1/Arduino"
DIST="$ROOT/03_Developpement_logiciel/Informatique/Calcul dist IHM/test.cpp"
OUT="$(mktemp -d)"; trap 'rm -rf "$OUT"' EXIT
CXX="g++ -std=c++17 -g -w -fsanitize=address,undefined -fno-sanitize-recover=all -DARDUINO=100 -Istub"
FAILED=""

run() {   # run <label> <command...>
    label="$1"; shift
    echo; echo "================ $label"
    if "$@"; then :; else FAILED="$FAILED\n  - $label"; fi
}
build() { $CXX "$@" 2>"$OUT/build.log" || { cat "$OUT/build.log"; return 1; }; }

if build test_tinygps.cpp "$TG/TinyGPS++.cpp" -I"$TG" -o "$OUT/tinygps"; then
    run "TinyGPSPlus library behaviour (#5 #6 #7)" "$OUT/tinygps"
else FAILED="$FAILED\n  - BUILD FAILED: TinyGPSPlus library behaviour (#5 #6 #7)"; fi

if build test_diag_util.cpp "$TG/TinyGPS++.cpp" -I"$TG" -o "$OUT/diag"; then
    run "gps_diag_board parsing helpers" "$OUT/diag"
else FAILED="$FAILED\n  - BUILD FAILED: gps_diag_board parsing helpers"; fi

if build test_main_driver.cpp "$MAIN/drivers/GpsUart.cpp" "$TG/TinyGPS++.cpp" stub/arduino_stub.cpp \
      -I"$TG" -I"$MAIN" -o "$OUT/main"; then
    run "CURRENT firmware driver main/ (#5 #6 #9)" "$OUT/main"
else FAILED="$FAILED\n  - BUILD FAILED: CURRENT firmware driver main/ (#5 #6 #9)"; fi

if build test_legacy_boat.cpp "$VN1/boat/Gps.cpp" "$AXP/axp20x.cpp" "$TG/TinyGPS++.cpp" stub/arduino_stub.cpp \
         -I"$TG" -I"$AXP" -I"$VN1/boat" -o "$OUT/legacy"; then
    for s in power bearing loss gate jump reset noise startbias; do
        run "LEGACY boat/Gps.cpp: $s (#8, filters)" "$OUT/legacy" "$s"
    done
else FAILED="$FAILED\n  - legacy build"; fi

if build test_legacy_gps_orig.cpp "$VN1/GPS/Gps.cpp" "$AXP/axp20x.cpp" "$TG/TinyGPS++.cpp" stub/arduino_stub.cpp \
      -I"$TG" -I"$AXP" -I"$VN1/GPS" -o "$OUT/orig"; then
    run "LEGACY original GPS/Gps.cpp (control R6)" "$OUT/orig"
else FAILED="$FAILED\n  - BUILD FAILED: LEGACY original GPS/Gps.cpp (control R6)"; fi

if $CXX -Dmain=legacy_test_main -c "$DIST" -o "$OUT/dist.o" 2>"$OUT/build.log" &&
   build test_legacy_distance.cpp "$OUT/dist.o" -o "$OUT/dist"; then
    run "LEGACY distance calculation (ruled out R3)" "$OUT/dist"
else FAILED="$FAILED\n  - BUILD FAILED: LEGACY distance calculation (ruled out R3)"; fi

run "Field log analyzer (gps_diag)" python3 test_analyze_log.py
run "REAL on-site telemetry 2024 (#1-#4, R1)" python3 test_field_log.py
run "Documents + IHM/firmware consistency (#10-#13, R2 R4 R5)" python3 test_docs_and_repo.py

echo; echo "================================================================"
if [ -z "$FAILED" ]; then echo "ALL TEST GROUPS PASSED"; exit 0; fi
printf "FAILED GROUPS:%b\n" "$FAILED"; exit 1
