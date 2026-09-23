/**
 * GPS diagnostic — LilyGO T-Beam V1.1 (standalone, no dependency on main/).
 *
 * Purpose: verify on real hardware how the NEO-6M + TinyGPSPlus pair behaves,
 * by printing the RAW NMEA fields next to what TinyGPSPlus reports.
 *
 * The GPS is configured with EXACTLY the same UBX packets as the drone
 * firmware (main/src/drivers/GpsUart.cpp), so what is seen here is what the
 * navigation code receives.
 *
 * Output (115200 baud), one line per event:
 *   C,...  config/status, repeated every 10 s (so a late serial attach sees it)
 *   D,...  one line per received $GxRMC sentence (1 Hz)
 *   T,...  $GxTXT from the receiver (antenna status)
 * Field layout is printed by the H lines at boot and every 10 s.
 */

#include <Wire.h>
#include <axp20x.h>
#include <TinyGPSPlus.h>
#include "diag_util.h"

// ---- Board pins (identical to main/src/config/BoardConfig.h) ----
static constexpr int      I2C_SDA  = 21;
static constexpr int      I2C_SCL  = 22;
static constexpr int      GPS_RX   = 34;
static constexpr int      GPS_TX   = 12;
static constexpr uint32_t GPS_BAUD = 9600;

static AXP20X_Class axp;
static TinyGPSPlus  gps;          // static storage, same as the firmware

static bool     axpOk        = false;
static uint32_t nav5Polls    = 0;
static char     antStatus[16] = "none";

// ---- Raw NMEA capture ----
static char     line[100];
static uint8_t  lineLen = 0;
static char     ggaQuality[4] = "", ggaSats[4] = "", ggaHdop[8] = "";
static uint32_t rmcCount = 0;

// ---- Heading from successive positions (legacy AutoBoat method) ----
static bool   havePrev = false;
static double prevLat = 0.0, prevLon = 0.0;

static diag::UbxNav5Reader ubx;   // extracts dynModel from the CFG-NAV5 answer

static void pollNav5() {
    uint8_t msg[8];
    diag::buildNav5Poll(msg);
    Serial1.write(msg, sizeof msg);
    ++nav5Polls;
}

static void printHeader() {
    Serial.println("H,C,ms,axpOk,dynModel,nav5Polls,antStatus,chars,badCRC,rmcCount");
    Serial.println("H,D,ms,rmcStatus,rawSpeedKn,rawCourse,ggaQuality,ggaSats,ggaHdop,"
                   "tgLocValid,tgLocAgeMs,tgCourseValid,tgCourseUpdated,tgCourseDeg,"
                   "tgSpeedKmh,tgSpeedAgeMs,lat,lon,posHeadingDeg,posStepM");
}

static void printConfig() {
    Serial.printf("C,%lu,%d,%d,%lu,%s,%lu,%lu,%lu\n",
        (unsigned long)millis(), axpOk ? 1 : 0, ubx.dynModel, (unsigned long)nav5Polls,
        antStatus, (unsigned long)gps.charsProcessed(),
        (unsigned long)gps.failedChecksum(), (unsigned long)rmcCount);
}

// Called once per complete RMC line, AFTER TinyGPSPlus has committed it.
static void onRmc(const char* s) {
    char status[4], spd[12], crs[12];
    diag::nmeaField(s, 2, status, sizeof status);
    diag::nmeaField(s, 7, spd, sizeof spd);
    diag::nmeaField(s, 8, crs, sizeof crs);
    ++rmcCount;

    // Order matters: isUpdated() must be read BEFORE deg(), which clears it.
    const bool crsValid   = gps.course.isValid();
    const bool crsUpdated = gps.course.isUpdated();
    const double crsDeg   = gps.course.deg();

    const bool   locValid = gps.location.isValid();
    const double lat      = locValid ? gps.location.lat() : 0.0;
    const double lon      = locValid ? gps.location.lng() : 0.0;

    // Heading from the previous position, only on sentences that carry a fix.
    double posHdg = -1.0, posStep = -1.0;
    if (status[0] == 'A' && locValid) {
        if (havePrev) {
            posStep = TinyGPSPlus::distanceBetween(prevLat, prevLon, lat, lon);
            posHdg  = TinyGPSPlus::courseTo(prevLat, prevLon, lat, lon);
        }
        prevLat = lat; prevLon = lon; havePrev = true;
    }

    Serial.printf("D,%lu,%s,%s,%s,%s,%s,%s,%d,%lu,%d,%d,%.2f,%.2f,%lu,%.7f,%.7f,%.1f,%.2f\n",
        (unsigned long)millis(), status, spd, crs, ggaQuality, ggaSats, ggaHdop,
        locValid ? 1 : 0, (unsigned long)gps.location.age(),
        crsValid ? 1 : 0, crsUpdated ? 1 : 0, crsDeg,
        gps.speed.kmph(), (unsigned long)gps.speed.age(),
        lat, lon, posHdg, posStep);
}

static void onLine(const char* s) {
    if (!diag::nmeaChecksumOk(s)) return;
    if (diag::isType(s, "GGA")) {
        diag::nmeaField(s, 6, ggaQuality, sizeof ggaQuality);
        diag::nmeaField(s, 7, ggaSats, sizeof ggaSats);
        diag::nmeaField(s, 8, ggaHdop, sizeof ggaHdop);
    } else if (diag::isType(s, "RMC")) {
        onRmc(s);
    } else if (diag::isType(s, "TXT")) {
        const char* a = strstr(s, "ANTSTATUS=");
        if (a) {
            size_t i = 0;
            for (a += 10; *a && *a != '*' && i + 1 < sizeof antStatus; ++a) antStatus[i++] = *a;
            antStatus[i] = '\0';
        }
        Serial.printf("T,%lu,%s\n", (unsigned long)millis(), s);
    }
}

// -------------------------------------------------------------------- setup

void setup() {
    Serial.begin(115200);
    delay(1500);
    Serial.println("\n# GPS diagnostic (standalone) - T-Beam V1.1");

    Wire.begin(I2C_SDA, I2C_SCL);
    axpOk = (axp.begin(Wire, AXP192_SLAVE_ADDRESS) == AXP_PASS);
    if (axpOk) {
        axp.setLDO3Voltage(3300);
        axp.setPowerOutPut(AXP192_LDO3, AXP202_ON);   // GPS
        axp.setDCDC1Voltage(3300);
        axp.setPowerOutPut(AXP192_DCDC1, AXP202_ON);  // 3.3 V rail
    }
    Serial.printf("# AXP192 begin: %s\n", axpOk ? "OK" : "FAIL");

    Serial1.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);
    delay(500);

    // Same packets, same order, same delays as main/src/drivers/GpsUart.cpp.
    static const uint8_t kCfgCfgReset[] = {
        0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00,
        0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F, 0x00, 0x00, 0x00,
        0x17, 0x71, 0xFE};
    Serial1.write(kCfgCfgReset, sizeof kCfgCfgReset);
    delay(1500);
    while (Serial1.available()) Serial1.read();

    static const uint8_t kCfgAnt[] = {
        0xB5, 0x62, 0x06, 0x13, 0x04, 0x00, 0x1B, 0x00, 0x0F, 0x00, 0x47, 0x57};
    Serial1.write(kCfgAnt, sizeof kCfgAnt);
    delay(100);

    // CFG-NAV5, mask 0x0001 (dynModel only), dynModel 5 = sea — as GpsUart::sendNav5SeaModel().
    uint8_t nav5[44] = {0xB5, 0x62, 0x06, 0x24, 36, 0x00};
    nav5[6] = 0x01;
    nav5[8] = 5;
    diag::ubxChecksum(nav5 + 2, 40, nav5[42], nav5[43]);
    Serial1.write(nav5, sizeof nav5);
    delay(100);
    Serial.println("# UBX CFG-CFG + CFG-ANT + CFG-NAV5(sea) sent (identical to firmware)");

    pollNav5();
    printHeader();
}

// --------------------------------------------------------------------- loop

void loop() {
    while (Serial1.available()) {
        const uint8_t c = (uint8_t)Serial1.read();
        ubx.feed(c);
        gps.encode((char)c);

        if (c == '\n') {
            line[lineLen] = '\0';
            if (lineLen > 0 && line[0] == '$') onLine(line);
            lineLen = 0;
        } else if (c != '\r') {
            if (c == '$') lineLen = 0;                 // resync on sentence start
            if (lineLen < sizeof line - 1) line[lineLen++] = (char)c;
        }
    }

    static uint32_t lastCfg = 0;
    if (millis() - lastCfg >= 10000) {
        lastCfg = millis();
        if (ubx.dynModel < 0) pollNav5();                  // retry until answered
        printHeader();
        printConfig();
    }
}
