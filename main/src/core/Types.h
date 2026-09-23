#pragma once
#include <Arduino.h>

enum class ControlMode : uint8_t {
    Failsafe  = 0,  // RC signal absent or invalid — follows mission (auto fallback)
    Sail      = 1,  // CH5 middle position — inert: all actuators to safe neutral
    Manual    = 2,  // CH5 high — full manual: CH2 sail, CH4 rotor, CH3 propeller (inverse)
    Automatic = 3   // GPS + waypoints autonomous sailing
};

// Raw RC pulse widths in microseconds. Value 0 means channel signal is lost.
struct RcFrame {
    uint16_t ch2 = 0;
    uint16_t ch3 = 0;
    uint16_t ch4 = 0;
    uint16_t ch5 = 0;
};

// Target pulse widths for all actuators in microseconds.
// Default values are safe neutral positions.
struct ActuatorCommand {
    uint16_t sailUs  = 1520;  // Futaba S3003 center
    uint16_t rotorUs = 1500;  // Regatta ECO II stopped
    uint16_t esc1Us  = 1500;  // neutre / arrêt (ESC bidirectionnel)
};

struct GpsPosition {
    bool     valid       = false;         // fix present, fresh, enough sats, acceptable HDOP
    double   lat         = 0.0;
    double   lon         = 0.0;
    float    speedKmph   = 0.0f;          // 0 when the receiver sends no speed
    bool     speedValid  = false;         // raw RMC speed field present
    float    courseDeg   = 0.0f;          // meaningful ONLY when courseValid
    bool     courseValid = false;         // raw course present and boat fast enough
    uint32_t courseSeq   = 0;             // +1 on each new valid course sample
    uint8_t  satellites  = 0;
    float    hdop        = 99.9f;
    uint32_t ageMs       = 0xFFFFFFFFUL;  // millis since last position fix
};

struct Waypoint {
    double lat     = 0.0;
    double lon     = 0.0;
    float  radiusM = 10.0f;
};

enum class MissionMode : uint8_t {
    Linear  = 0,  // visit waypoints in order, then return to home point
    Circuit = 1   // loop through waypoints indefinitely
};

enum class MissionState : uint8_t {
    Idle      = 0,  // no mission running
    Running   = 1,  // navigating toward waypoints
    Returning = 2,  // heading to home point (linear complete or emergency)
    Complete  = 3   // arrived at home after linear mission
};
