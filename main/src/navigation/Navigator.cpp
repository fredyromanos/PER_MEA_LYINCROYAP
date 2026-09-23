#include "Navigator.h"
#include <math.h>

static constexpr double EARTH_R_M = 6371000.0;
static constexpr double DEG2RAD   = M_PI / 180.0;

float Navigator::distanceM(double lat1, double lon1, double lat2, double lon2) {
    const double dLat = (lat2 - lat1) * DEG2RAD;
    const double dLon = (lon2 - lon1) * DEG2RAD;
    const double a    = sin(dLat/2)*sin(dLat/2) +
                        cos(lat1*DEG2RAD)*cos(lat2*DEG2RAD)*
                        sin(dLon/2)*sin(dLon/2);
    return (float)(EARTH_R_M * 2.0 * atan2(sqrt(a), sqrt(1.0 - a)));
}

float Navigator::bearingDeg(double lat1, double lon1, double lat2, double lon2) {
    const double dLon = (lon2 - lon1) * DEG2RAD;
    const double y    = sin(dLon) * cos(lat2 * DEG2RAD);
    const double x    = cos(lat1 * DEG2RAD) * sin(lat2 * DEG2RAD) -
                        sin(lat1 * DEG2RAD) * cos(lat2 * DEG2RAD) * cos(dLon);
    return (float)fmod(atan2(y, x) / DEG2RAD + 360.0, 360.0);
}
