#include "GpsUart.h"
#include "../config/BoardConfig.h"
#include "../config/Calibration.h"
#include "../config/DebugConfig.h"

static constexpr float KMPH_PER_KNOT = 1.852f;

void GpsUart::ubxChecksum(const uint8_t* body, size_t len, uint8_t& ckA, uint8_t& ckB) {
    ckA = ckB = 0;
    for (size_t i = 0; i < len; ++i) {
        ckA = (uint8_t)(ckA + body[i]);
        ckB = (uint8_t)(ckB + ckA);
    }
}

// UBX-CFG-NAV5 (class 0x06 id 0x24, 36-byte payload): dynamic model 5 = "sea".
// mask=0x0001 applies ONLY the dynModel field; every other setting is left alone.
// The CFG-CFG reset in begin() reloads navConf from ROM (= "portable"), so this
// must be sent after it on every boot.
void GpsUart::sendNav5SeaModel() {
    uint8_t frame[2 + 4 + 36 + 2] = {0xB5, 0x62, 0x06, 0x24, 36, 0x00};
    uint8_t* payload = frame + 6;
    payload[0] = 0x01;   // mask LE: dyn
    payload[1] = 0x00;
    payload[2] = 5;      // dynModel: sea
    ubxChecksum(frame + 2, 4 + 36, frame[42], frame[43]);
    Serial1.write(frame, sizeof(frame));
}

void GpsUart::begin() {
    gsvTotal_.begin(gps_, "GPGSV", 3);  // field 3 = total satellites in view
    rmcSpeed_.begin(gps_, "GPRMC", 7);
    rmcCourse_.begin(gps_, "GPRMC", 8);
    Serial1.begin(BoardConfig::GPS_BAUD_RATE, SERIAL_8N1,
                  BoardConfig::GPS_RX_PIN, BoardConfig::GPS_TX_PIN);
    DBG_GPS("Serial1 started %u baud RX=GPIO%d TX=GPIO%d",
        (unsigned)BoardConfig::GPS_BAUD_RATE,
        BoardConfig::GPS_RX_PIN, BoardConfig::GPS_TX_PIN);

    // Give GPS time to finish power-on before sending UBX commands.
    delay(500);

    // Factory-reset GPS to ROM defaults (UBX-CFG-CFG, class=0x06 id=0x09).
    // A previously-saved flash config may have disabled NMEA output entirely
    // (e.g. from a u-center CFG-PRT session). Symptom: GPS ACKs UBX commands
    // but chars counter stays at 10 (= one ACK frame) with zero NMEA output.
    // clearMask=0x1F  → clear ioPort+msgConf+infMsg+navConf+rxmConf.
    // loadMask=0x1F   → reload those sections from ROM (factory defaults).
    // deviceMask=0x17 → target BBR+Flash+EEPROM on the GPS module.
    // After reset: UART1 = 9600 baud, NMEA sentences enabled, ant supervisor off.
    static const uint8_t kCfgCfgReset[] = {
        0xB5, 0x62,                      // UBX sync
        0x06, 0x09,                      // class=CFG, id=CFG
        0x0D, 0x00,                      // payload length = 13
        0x1F, 0x00, 0x00, 0x00,         // clearMask LE
        0x00, 0x00, 0x00, 0x00,         // saveMask  LE
        0x1F, 0x00, 0x00, 0x00,         // loadMask  LE
        0x17,                            // deviceMask (BBR|Flash|EEPROM)
        0x71, 0xFE                       // Fletcher checksum
    };
    Serial1.write(kCfgCfgReset, sizeof(kCfgCfgReset));
    DBG_GPS("CFG-CFG sent — factory reset to ROM defaults");

    // Wait for GPS to apply reset and restart its engine.
    delay(1500);

    // Discard any partial data accumulated during restart.
    while (Serial1.available()) Serial1.read();

    // Enable active external antenna (Taoglas ADFGP.25A or similar).
    // Factory-reset GPS has antenna supervisor OFF → no DC bias on the coax
    // → active LNA is unpowered → zero signal. Also, the T-Beam RF switch
    // (driven by NEO-6M ANT_FLAG) stays on the internal ceramic patch.
    // UBX-CFG-ANT flags=0x001B: svcs=1 (supply ctrl + RF switch),
    // scd=1 (short detect), pdwnOnSCD=1, recovery=1.
    static const uint8_t kCfgAnt[] = {
        0xB5, 0x62,
        0x06, 0x13,
        0x04, 0x00,
        0x1B, 0x00,   // flags LE: svcs|scd|pdwnOnSCD|recovery
        0x0F, 0x00,   // pins  LE: default NEO-6M pin mapping
        0x47, 0x57    // Fletcher checksum
    };
    Serial1.write(kCfgAnt, sizeof(kCfgAnt));
    delay(100);
    DBG_GPS("CFG-ANT sent — active antenna supervisor + bias enabled");

    sendNav5SeaModel();
    delay(100);
    DBG_GPS("CFG-NAV5 sent — dynamic model 5 (sea)");
}

uint8_t GpsUart::satsInView() {
    return gsvTotal_.isValid() ? (uint8_t)atoi(gsvTotal_.value()) : 0;
}

void GpsUart::update() {
    while (Serial1.available()) {
        char c = Serial1.read();
        gps_.encode(c);

        // Capture last complete NMEA sentence into completedLine_
        if (c == '\n') {
            lineBuf_[lineLen_] = '\0';
            memcpy(completedLine_, lineBuf_, lineLen_ + 1);
            lineLen_ = 0;
        } else if (c != '\r' && lineLen_ < sizeof(lineBuf_) - 1) {
            lineBuf_[lineLen_++] = c;
        }
    }

    // location.isValid() is sticky: freshness, satellites and HDOP decide the fix.
    const uint32_t age  = gps_.location.isValid() ? gps_.location.age() : 0xFFFFFFFFUL;
    const uint8_t  sats = gps_.satellites.isValid() ? (uint8_t)gps_.satellites.value() : 0;
    const float    hdop = gps_.hdop.isValid() ? (float)gps_.hdop.hdop() : 99.9f;
    const bool nowValid = gps_.location.isValid()
                       && age  <  Calibration::GPS_FIX_MAX_AGE_MS
                       && sats >= Calibration::GPS_MIN_SATS
                       && hdop <= Calibration::GPS_MAX_HDOP;

    // Log GPS fix transitions
    if (nowValid && !prevValid_) {
        DBG_GPS("FIX ACQUIRED  lat=%.6f lon=%.6f sats=%u hdop=%.1f",
            gps_.location.lat(), gps_.location.lng(), (unsigned)sats, hdop);
    } else if (!nowValid && prevValid_) {
        DBG_GPS("FIX LOST  age=%lums sats=%u hdop=%.1f", (unsigned long)age, (unsigned)sats, hdop);
    }
    prevValid_ = nowValid;

    // One new RMC sentence = one speed/course sample, read from the RAW fields:
    // TinyGPSPlus keeps the previous speed/course when the receiver blanks them.
    if (rmcSpeed_.isUpdated() || rmcCourse_.isUpdated()) {
        const char* spd = rmcSpeed_.value();    // value() clears the updated flag
        const char* crs = rmcCourse_.value();
        const bool speedPresent = spd[0] != '\0';
        pos_.speedKmph  = speedPresent ? (float)atof(spd) * KMPH_PER_KNOT : 0.0f;
        pos_.speedValid = speedPresent;
        rmcCourseOk_ = nowValid && speedPresent && crs[0] != '\0'
                    && pos_.speedKmph >= Calibration::GPS_COURSE_MIN_SPEED_KMPH;
        if (rmcCourseOk_) {
            pos_.courseDeg = (float)atof(crs);
            pos_.courseSeq++;
        }
    }

    pos_.valid       = nowValid;
    // Keep the last known position when invalid (never report [0,0]); consumers check valid.
    if (gps_.location.isValid()) {
        pos_.lat = gps_.location.lat();
        pos_.lon = gps_.location.lng();
    }
    pos_.speedValid  = pos_.speedValid && nowValid;
    pos_.courseValid = rmcCourseOk_ && nowValid;
    pos_.satellites  = sats;
    pos_.hdop        = hdop;
    pos_.ageMs       = age;
}
