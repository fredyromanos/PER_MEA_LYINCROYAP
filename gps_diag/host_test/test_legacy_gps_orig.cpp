// Control for T08c: the ORIGINAL legacy driver AutoBoat_VN-1/Arduino/GPS/Gps.cpp (compiled unmodified)
// gets the waypoint bearing right, so the 180 deg reversal was introduced in boat/Gps.cpp.
#include "test_common.h"
#include "Gps.hpp"
#include <cmath>

static constexpr double LAT = 48.360415, LON = -4.566613;
static GPS boat;

int main() {
    fake_millis = 0;
    boat.init();
    for (int i = 0; i < 3; ++i) {
        fake_millis += 1001;
        Serial1.feed(gga('1', LAT, LON, "07") + rmc('A', LAT, LON, "", ""));
        boat.upDatePosition();
    }
    boat.computeDirectPath(LAT + 100.0 / 111320.0, LON);   // waypoint 100 m due north
    char msg[160];
    std::snprintf(msg, sizeof msg, "original GPS/Gps.cpp: waypoint 100 m NORTH -> getCap() = %.1f deg",
                  boat.getCap());
    check(boat.getStatus() == 2 && std::fabs(boat.getCap()) < 1.0, "R6", msg);
    return finish();
}
