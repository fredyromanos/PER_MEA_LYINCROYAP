#pragma once
#include <cstdio>
#include <cstdint>
#include <cstddef>

// Heartbeat JSON formatting, kept free of Arduino/radio dependencies so the
// 255-byte LoRa budget can be checked on a PC (test/test_gps_logic.cpp).
//
//   hv = heading source: 0 none (do not trust "heading"), 1 GPS course, 2 held
//   fa = seconds since the last position fix (capped at 99)
static constexpr size_t HEARTBEAT_LORA_MAX_BYTES = 255;

struct HeartbeatFields {
    const char* mode      = "standby";
    double      lat       = 0.0;
    double      lon       = 0.0;
    int         sailDeg   = 0;
    int         rudderDeg = 0;
    float       heading   = 0.0f;
    uint8_t     headingSrc = 0;
    uint8_t     fixAgeS   = 99;
    float       windDeg   = 0.0f;
    float       batVolts  = 0.0f;
    bool        gpsFix    = false;
    uint8_t     sats      = 0;
    float       hdop      = 99.9f;
    bool        rcOk      = false;
    uint8_t     wptTotal  = 0;
    uint8_t     wptCur    = 0;
    bool        windObs   = false;   // "wobs" only present while measuring
    uint8_t     windObsPct = 0;
};

// Returns the number of characters that were (or would have been) written.
inline int formatHeartbeat(char* buf, size_t size, const HeartbeatFields& f) {
    char wobsFrag[16] = "";
    if (f.windObs) {
        std::snprintf(wobsFrag, sizeof(wobsFrag), ",\"wobs\":%u", (unsigned)f.windObsPct);
    }
    return std::snprintf(buf, size,
        "{\"origin\":\"boat\",\"type\":\"info\",\"message\":{"
        "\"mode\":\"%s\","
        "\"location\":[%.5f,%.5f],"
        "\"servos\":{\"sail\":%d,\"rudder\":%d},"
        "\"heading\":%.0f,\"hv\":%u,"
        "\"wind\":%.0f,"
        "\"bat\":%.2f,"
        "\"fix\":%d,\"fa\":%u,\"sat\":%u,\"hdop\":%.1f,\"rc\":%d,"
        "\"wt\":%u,\"wc\":%u"
        "%s"
        "}}",
        f.mode, f.lat, f.lon, f.sailDeg, f.rudderDeg,
        f.heading, (unsigned)f.headingSrc,
        f.windDeg, f.batVolts,
        f.gpsFix ? 1 : 0, (unsigned)f.fixAgeS, (unsigned)f.sats, f.hdop, f.rcOk ? 1 : 0,
        (unsigned)f.wptTotal, (unsigned)f.wptCur, wobsFrag);
}
