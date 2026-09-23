#pragma once
#include <Arduino.h>

// Minimal subset of the firmware core/Types.h needed by the GPS test.

struct GpsPosition {
    bool     valid      = false;
    double   lat        = 0.0;
    double   lon        = 0.0;
    float    speedKmph  = 0.0f;
    float    courseDeg  = 0.0f;
    uint8_t  satellites = 0;
    float    hdop       = 99.9f;
    uint32_t ageMs      = 0xFFFFFFFFUL;  // millis since last valid fix
};
