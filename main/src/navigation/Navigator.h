#pragma once
#include <Arduino.h>

namespace Navigator {
    // Distance in meters between two GPS coordinates (haversine formula)
    float distanceM(double lat1, double lon1, double lat2, double lon2);
    // Bearing in degrees [0, 360) from point 1 toward point 2
    float bearingDeg(double lat1, double lon1, double lat2, double lon2);
}
