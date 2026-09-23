#include "GpsUart.h"
#include "BoardConfig.h"
#include "DebugConfig.h"

void GpsUart::begin() {
    gsvTotal_.begin(gps_, "GPGSV", 3);  // field 3 = total satellites in view
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

    const bool nowValid = gps_.location.isValid();

    // Log GPS fix transitions
    if (nowValid && !prevValid_) {
        DBG_GPS("FIX ACQUIRED  lat=%.6f lon=%.6f sats=%u hdop=%.1f",
            gps_.location.lat(), gps_.location.lng(),
            gps_.satellites.isValid() ? (unsigned)gps_.satellites.value() : 0u,
            gps_.hdop.isValid() ? (float)gps_.hdop.hdop() : 99.9f);
    } else if (!nowValid && prevValid_) {
        DBG_GPS("FIX LOST");
    }
    prevValid_ = nowValid;

    pos_.valid      = nowValid;
    pos_.lat        = nowValid ? gps_.location.lat()           : 0.0;
    pos_.lon        = nowValid ? gps_.location.lng()           : 0.0;
    pos_.speedKmph  = gps_.speed.isValid()      ? (float)gps_.speed.kmph()    : 0.0f;
    pos_.courseDeg  = gps_.course.isValid()     ? (float)gps_.course.deg()    : 0.0f;
    pos_.satellites = gps_.satellites.isValid() ? (uint8_t)gps_.satellites.value() : 0;
    pos_.hdop       = gps_.hdop.isValid()       ? (float)gps_.hdop.hdop()     : 99.9f;
    pos_.ageMs      = gps_.location.isValid()   ? gps_.location.age()         : 0xFFFFFFFFUL;
}
