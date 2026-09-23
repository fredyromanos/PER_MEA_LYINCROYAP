#pragma once
#include <Arduino.h>
#include <TinyGPSPlus.h>
#include "Types.h"

// Reads NMEA sentences from Serial1 (u-blox NEO-6M on T-Beam GPIO34/12).
// AXP192 LDO3 must be enabled before begin() — AxpPower::begin() does this.
// Call update() on every loop() iteration to drain the serial buffer.
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

private:
    TinyGPSPlus   gps_;
    TinyGPSCustom gsvTotal_;   // total satellites in view from $GPGSV field 3
    GpsPosition   pos_;
    char          lineBuf_[96]       = {};
    char          completedLine_[96] = {};
    uint8_t       lineLen_           = 0;
    bool          prevValid_         = false;  // tracks fix state for change detection
};
