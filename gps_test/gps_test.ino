/**
 * GPS Standalone Test for LilyGO T-Beam V1.1
 * Tests u-blox NEO-6M with external active antenna.
 *
 * Uses the SAME power init (AxpPower) and GPS driver (GpsUart) as the real
 * firmware, so a fix here means the firmware GPS path is good — and no fix
 * here isolates the fault to hardware/antenna/sky, not the firmware.
 *
 * Requires: LiPo battery connected, external antenna on u.FL, clear sky view.
 */

#include "AxpPower.h"
#include "GpsUart.h"

GpsUart gps;

void printGpsStatus();

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 2000) {}  // Wait for USB serial

    Serial.println("\n=== GPS Standalone Test ===");
    Serial.println("Board: LilyGO T-Beam V1.1");
    Serial.println("GPS: u-blox NEO-6M @ 9600 baud");
    Serial.println("Pins: RX=GPIO34, TX=GPIO12");
    Serial.println("Antenna: External active (u.FL)");
    Serial.println();

    // Enable AXP192 rails (LDO3 = GPS power) exactly like the firmware.
    if (!AxpPower::begin()) {
        Serial.println("[FATAL] AXP192 init failed — GPS has no power. Check I2C/battery.");
    }

    // Initialize GPS UART + send UBX config (CFG-CFG reset, CFG-ANT).
    gps.begin();
}

void loop() {
    // Drain Serial1 buffer every loop (critical!)
    gps.update();

    // Print status every 200ms
    static uint32_t lastPrint = 0;
    if (millis() - lastPrint >= 200) {
        lastPrint = millis();
        printGpsStatus();
    }
}

void printGpsStatus() {
    const auto& pos = gps.position();

    Serial.print("[GPS] ");

    if (pos.valid) {
        Serial.printf("lat=%.6f lon=%.6f spd=%.1fkm/h hdg=%.1f° sat=%u hdop=%.1f age=%lums\n",
            pos.lat, pos.lon, pos.speedKmph, pos.courseDeg,
            pos.satellites, pos.hdop, pos.ageMs);
    } else {
        Serial.printf("NO FIX  visible=%u  chars=%lu  badCRC=%lu\n",
            gps.satsInView(), gps.charsProcessed(), gps.failedChecksums());

        const char* last = gps.lastLine();
        if (last && strlen(last) > 0) {
            Serial.printf("        last: %.80s\n", last);
        }
    }
}
