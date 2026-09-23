// Tests the REAL legacy code Informatique/AutoBoat_VN-1/Arduino/boat/Gps.cpp (compiled unmodified,
// identical to the handover archive) together with the real AXP202X library.
// The legacy code keeps global state, so each scenario runs in a fresh process: ./bin <scenario>
#include "test_common.h"
#include "Gps.hpp"
#include "Wire.h"
#include <cmath>
#include <set>

static constexpr double LAT = 48.360415, LON = -4.566613;
static double dLat(double m) { return m / 111320.0; }
static double dLon(double m) { return m / (111320.0 * std::cos(LAT * PI / 180.0)); }

static GPS boat;   // global, like `GPS gpsBoat;` in boat.ino

// Feeds one fix and runs one legacy update (the legacy code only updates once per >1000 ms).
static void step(double lat, double lon, const char* spd = "", const char* crs = "") {
    fake_millis += 1001;
    Serial1.feed(gga('1', lat, lon, "07") + rmc('A', lat, lon, spd, crs));
    boat.upDatePosition();
}
static void stepLost() {
    fake_millis += 1001;
    Serial1.feed(nmea("GPGGA,101530.00,,,,,0,00,99.99,,,,,,") + nmea("GPRMC,101530.00,V,,,,,,,160926,,,N"));
    boat.upDatePosition();
}

static int powerAndAntenna() {
    fake_millis = 0;
    boat.init();
    check(Wire.transmissions == 0 && Wire.bytesWritten == 0,
          "T08a", "legacy GPS::init() sends ZERO I2C bytes: AXP192 LDO3 (GPS power) never commanded");
    check(Serial1.begun && Serial1.tx.empty(),
          "T08b", "legacy GPS::init() writes ZERO bytes to the GPS: no CFG-ANT, active antenna unpowered");

    AXP20X_Class axp;   // exactly what legacy init() does: no begin()
    check(axp.setPowerOutPut(AXP192_LDO3, AXP202_ON) == AXP_NOT_INIT &&
          axp.setLDO3Voltage(3300) == AXP_NOT_INIT,
          "T08a2", "real AXP library refuses both calls with AXP_NOT_INIT when begin() was not called");
    const int before = Wire.transmissions;
    AXP20X_Class probe;
    probe.begin(Wire, AXP192_SLAVE_ADDRESS);
    check(Wire.transmissions > before,
          "T08a3", "control: with begin(Wire) the library DOES talk I2C (stub counts traffic)");
    return finish();
}

static int bearingReversed() {
    fake_millis = 0; boat.init();
    for (int i = 0; i < 3; ++i) step(LAT, LON);
    const double wLat = LAT + dLat(100), wLon = LON;            // waypoint 100 m due NORTH
    boat.computeDirectPath(wLat, wLon);                         // as handleNavigation() does
    const double h = boat.getHeading(), d = boat.getDist();
    char msg[160];
    std::snprintf(msg, sizeof msg,
        "waypoint 100 m NORTH -> legacy getHeading() = %.1f deg (true bearing 0): reversed", h);
    check(std::fabs(h - 180.0) < 1.0, "T08c", msg);
    check(std::fabs(d - 100.0) < 1.0, "T08c2", "distance is correct (100 m): only the direction is wrong");
    return finish();
}

static int fixLoss() {
    fake_millis = 0; boat.init();
    for (int i = 0; i < 3; ++i) step(LAT, LON);
    const int stBefore = boat.getStatus();
    for (int i = 0; i < 30; ++i) stepLost();
    check(stBefore == 2, "T05L0", "legacy status 2 (location valid) with fix");
    check(boat.getStatus() == 2, "T05L", "30 s after fix loss legacy status is STILL 2 - loss undetected");
    return finish();
}

static int threeMetreGate() {
    fake_millis = 0; boat.init();
    step(LAT, LON);
    step(LAT, LON);
    const double start = boat.getLat();
    step(LAT + dLat(1), LON);                                   // boat moves 1 m/s north
    step(LAT + dLat(2), LON);
    const double moved2 = (boat.getLat() - start) * 111320.0;  // metres
    char msg[160];
    std::snprintf(msg, sizeof msg,
        "boat moved 2 m north: legacy position moved %.4f m (<3 m updates discarded)", moved2);
    check(std::fabs(moved2) < 0.01, "L01", msg);
    step(LAT + dLat(3.2), LON);                                 // 3.2 m from the recorded point
    const double moved3 = (boat.getLat() - start) * 111320.0;
    std::snprintf(msg, sizeof msg,
        "control: at 3.2 m the legacy position does move (%.2f m) -> the gate, not a frozen test", moved3);
    check(moved3 > 0.3, "L01c", msg);
    return finish();
}

static int jumpRejected() {
    fake_millis = 0; boat.init();
    step(LAT, LON); step(LAT, LON);
    const double bLat = LAT + dLat(40);                          // real position now 40 m further
    int stuck = 0;
    for (int i = 0; i < 10; ++i) {
        step(bLat, LON);
        if (std::fabs(boat.getLat() - LAT) < dLat(1)) ++stuck;
    }
    char msg[160];
    std::snprintf(msg, sizeof msg,
        "boat really 40 m away: legacy position stays at the old point for %d/10 updates (>25 m rejected)", stuck);
    check(stuck == 10, "L02", msg);
    return finish();
}

static int historyReset() {
    fake_millis = 0; boat.init();
    int resetAt = -1;
    for (int i = 1; i <= 40 && resetAt < 0; ++i) {
        Serial.text.clear();
        step(LAT, LON);
        if (Serial.text.find("initialisation de l'historique") != std::string::npos) resetAt = i;
    }
    char msg[160];
    std::snprintf(msg, sizeof msg, "legacy position history wiped after %d updates (every ~30 s)", resetAt);
    check(resetAt > 0 && resetAt <= 31, "L03", msg);
    return finish();
}

static int stationaryHeadingNoise() {
    fake_millis = 0; boat.init();
    // Deterministic pseudo-random jitter within +/-4 m: typical NEO-6M scatter (we measured 3-5 m).
    uint32_t seed = 12345;
    auto rnd = [&]() { seed = seed * 1103515245u + 12345u; return ((seed >> 8) % 2001) / 1000.0 - 1.0; };
    std::set<int> headings;
    for (int i = 0; i < 120; ++i) {
        step(LAT + dLat(4 * rnd()), LON + dLon(4 * rnd()));
        if (i >= 10) headings.insert((int)std::lround(boat.getSmoothHeading()) % 360);
    }
    char msg[160];
    std::snprintf(msg, sizeof msg,
        "boat NOT moving, GPS jitter +/-4 m: legacy heading took %zu different values over 110 s",
        headings.size());
    check(headings.size() >= 20, "L04", msg);
    return finish();
}

static int headingStartBias() {
    fake_millis = 0; boat.init();
    // Boat moving steadily EAST (true heading 90) at 5 m/s from the first fix.
    double lon = LON;
    step(LAT, lon);
    std::vector<double> h;
    for (int i = 0; i < 12; ++i) { lon += dLon(5); step(LAT, lon); h.push_back(boat.getSmoothHeading()); }
    char msg[200];
    std::snprintf(msg, sizeof msg,
        "moving due EAST (90): first legacy headings %.0f, %.0f -> converge to %.0f only after history fills",
        h[0], h[1], h.back());
    check(std::fabs(h[0] - 90.0) > 20.0 && std::fabs(h.back() - 90.0) < 5.0, "L05", msg);
    return finish();
}

int main(int argc, char** argv) {
    std::string s = argc > 1 ? argv[1] : "";
    if (s == "power")      return powerAndAntenna();
    if (s == "bearing")    return bearingReversed();
    if (s == "loss")       return fixLoss();
    if (s == "gate")       return threeMetreGate();
    if (s == "jump")       return jumpRejected();
    if (s == "reset")      return historyReset();
    if (s == "noise")      return stationaryHeadingNoise();
    if (s == "startbias")  return headingStartBias();
    std::printf("usage: %s power|bearing|loss|gate|jump|reset|noise|startbias\n", argv[0]);
    return 2;
}
