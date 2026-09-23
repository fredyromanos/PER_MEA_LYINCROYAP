#pragma once
#include <Arduino.h>
#include <TinyGPSPlus.h>
#include "../core/Types.h"

// Reads NMEA sentences from Serial1 (u-blox NEO-6M on T-Beam GPIO34/12).
// AXP192 LDO3 must be enabled before begin() — AxpPower::begin() does this.
// Call update() on every loop() iteration to drain the serial buffer.
//
// Validity rules (TinyGPSPlus alone is not enough — proven in gps_diag/host_test):
//   - location.isValid() never returns to false once a fix was seen, so the fix
//     is only valid while fresh (GPS_FIX_MAX_AGE_MS) with enough sats and HDOP.
//   - an empty NMEA field keeps the previous value and is still marked valid,
//     so speed and course are read from the RAW RMC fields.
class GpsUart {
public:
    void begin();
    void update();
    const GpsPosition& position() const { return pos_; }

    uint32_t    charsProcessed()   const { return gps_.charsProcessed(); }
    uint32_t    sentencesWithFix() const { return gps_.sentencesWithFix(); }
    uint32_t    failedChecksums()  const { return gps_.failedChecksum(); }
    const char* lastLine()         const { return completedLine_; }
    uint8_t     satsInView();

    // Fletcher-8 checksum of a UBX frame body (class..payload). Public for tests.
    static void ubxChecksum(const uint8_t* body, size_t len, uint8_t& ckA, uint8_t& ckB);

private:
    TinyGPSPlus   gps_;
    TinyGPSCustom gsvTotal_;   // total satellites in view from $GPGSV field 3
    TinyGPSCustom rmcSpeed_;   // raw $GPRMC field 7 (knots) — empty when stopped
    TinyGPSCustom rmcCourse_;  // raw $GPRMC field 8 (deg)   — empty when stopped
    GpsPosition   pos_;
    char          lineBuf_[96]       = {};
    char          completedLine_[96] = {};
    uint8_t       lineLen_           = 0;
    bool          prevValid_         = false;  // tracks fix state for change detection
    bool          rmcCourseOk_       = false;  // last RMC carried a usable course

    void sendNav5SeaModel();
};
