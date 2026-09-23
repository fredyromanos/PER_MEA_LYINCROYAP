// Tests the REAL current firmware driver main/src/drivers/GpsUart.cpp (compiled unmodified).
// Checks the FIXED behaviour of the three defects found by this bank:
//   #5 fix loss must be detected, #6 course/speed must not be fake or frozen,
//   #9 the receiver must be put in the "sea" dynamic model.
#include "test_common.h"
#include "drivers/GpsUart.h"
#include "config/Calibration.h"
#include <cmath>

static constexpr double LAT = 48.360415, LON = -4.566613;   // position measured on our board

static void resetPorts() { Serial1.rx.clear(); Serial1.tx.clear(); Serial.text.clear(); }
static void feedAndUpdate(GpsUart& g, const std::string& s) { Serial1.feed(s); g.update(); }
static std::string ggaHdop(double lat, double lon, const char* sats, const char* hdop) {
    return nmea("GPGGA,101500.00," + nmeaLat(lat) + "," + nmeaLon(lon) + ",1," + sats + "," + hdop +
                ",40.0,M,50.0,M,,");
}
static const std::string kNoFix = nmea("GPGGA,101530.00,,,,,0,00,99.99,,,,,,") +
                                  nmea("GPRMC,101530.00,V,,,,,,,160926,,,N");

// Static storage, exactly like `static DroneApp app;` in main.ino.
static GpsUart gCfg, gSilent, gNoFix, gBoot, gMove, gQual;

int main() {
    // ---------- #9 : what begin() sends to the receiver ----------
    resetPorts(); fake_millis = 0;
    gCfg.begin();
    struct Frame { uint8_t cls, id; uint16_t len; bool ckOk; std::vector<uint8_t> payload; };
    std::vector<Frame> frames;
    const auto& tx = Serial1.tx;
    for (size_t i = 0; i + 8 <= tx.size();) {
        if (tx[i] != 0xB5 || tx[i + 1] != 0x62) { ++i; continue; }
        Frame f{tx[i + 2], tx[i + 3], (uint16_t)(tx[i + 4] | tx[i + 5] << 8), false, {}};
        size_t end = i + 6 + f.len;
        if (end + 2 > tx.size()) break;
        uint8_t a = 0, b = 0;
        for (size_t k = i + 2; k < end; ++k) { a += tx[k]; b += a; }   // independent Fletcher
        f.ckOk = (a == tx[end] && b == tx[end + 1]);
        f.payload.assign(tx.begin() + i + 6, tx.begin() + end);
        frames.push_back(f);
        i = end + 2;
    }
    check(frames.size() == 3, "T09a", "driver sends 3 UBX frames at boot");
    check(frames.size() == 3 && frames[0].cls == 0x06 && frames[0].id == 0x09 && frames[0].ckOk,
          "T09b", "frame 1 = CFG-CFG with valid checksum (unchanged)");
    check(frames.size() == 3 && frames[1].cls == 0x06 && frames[1].id == 0x13 && frames[1].ckOk,
          "T09c", "frame 2 = CFG-ANT with valid checksum (unchanged)");
    check(frames.size() == 3 && frames[2].cls == 0x06 && frames[2].id == 0x24 && frames[2].len == 36 &&
          frames[2].ckOk,
          "T09d", "frame 3 = CFG-NAV5, 36-byte payload, valid checksum");
    check(frames.size() == 3 && frames[2].payload[0] == 0x01 && frames[2].payload[1] == 0x00 &&
          frames[2].payload[2] == 5,
          "T09e", "CFG-NAV5 applies ONLY dynModel (mask 0x0001) = 5 (sea), after the CFG-CFG reset");

    // ---------- #5a : receiver goes SILENT -> fix lost by age ----------
    resetPorts(); fake_millis = 10000;
    gSilent.begin(); resetPorts();
    feedAndUpdate(gSilent, ggaHdop(LAT, LON, "07", "1.20") + rmc('A', LAT, LON, "0.10", "45.00"));
    const bool validAtFix = gSilent.position().valid;
    const unsigned long tFix = fake_millis;
    fake_millis = tFix + 1000; gSilent.update();
    const bool validAt1s = gSilent.position().valid;
    fake_millis = tFix + Calibration::GPS_FIX_MAX_AGE_MS; gSilent.update();
    check(validAtFix, "T05a", "valid position once fix acquired (7 sats, HDOP 1.2)");
    check(validAt1s, "T05b", "1 s without data: still valid (within GPS_FIX_MAX_AGE_MS)");
    check(!gSilent.position().valid, "T05c", "receiver silent 2 s: position().valid becomes FALSE");
    check(Serial.text.find("FIX ACQUIRED") != std::string::npos,
          "T05d0", "control: driver logs ARE captured (FIX ACQUIRED present)");
    check(Serial.text.find("FIX LOST") != std::string::npos, "T05d", "'FIX LOST' is logged");
    check(std::fabs(gSilent.position().lat - LAT) < 1e-5,
          "T05e", "last position kept for telemetry (never [0,0]) while flagged invalid");
    fake_millis += 1000;
    feedAndUpdate(gSilent, ggaHdop(LAT, LON, "07", "1.20") + rmc('A', LAT, LON, "0.10", "45.00"));
    check(gSilent.position().valid, "T05f", "fix returns -> valid again by itself");

    // ---------- #5b : receiver REPORTS no fix -> lost immediately ----------
    resetPorts(); fake_millis = 30000;
    gNoFix.begin(); resetPorts();
    feedAndUpdate(gNoFix, ggaHdop(LAT, LON, "07", "1.20") + rmc('A', LAT, LON, "0.10", "45.00"));
    fake_millis += 1000;
    feedAndUpdate(gNoFix, kNoFix);
    check(!gNoFix.position().valid, "T05g", "receiver sends 'no fix' (0 sats): invalid on the next update");

    // ---------- #5c : poor-quality fix rejected ----------
    resetPorts(); fake_millis = 40000;
    gQual.begin(); resetPorts();
    feedAndUpdate(gQual, ggaHdop(LAT, LON, "04", "1.20") + rmc('A', LAT, LON, "0.10", "45.00"));
    check(!gQual.position().valid, "T05h", "4 satellites (< GPS_MIN_SATS) -> not valid");
    fake_millis += 1000;
    feedAndUpdate(gQual, ggaHdop(LAT, LON, "08", "5.00") + rmc('A', LAT, LON, "0.10", "45.00"));
    check(!gQual.position().valid, "T05i", "HDOP 5.0 (> GPS_MAX_HDOP) -> not valid");

    // ---------- #6a : never moved since boot -> no course, no speed ----------
    resetPorts(); fake_millis = 50000;
    gBoot.begin(); resetPorts();
    feedAndUpdate(gBoot, ggaHdop(LAT, LON, "07", "1.20") + rmc('A', LAT, LON, "", ""));
    const GpsPosition& pb = gBoot.position();
    check(pb.valid && !pb.courseValid && pb.courseSeq == 0,
          "T06a", "stationary since boot, empty course field -> courseValid FALSE (no fake north)");
    check(!pb.speedValid && pb.speedKmph == 0.0f, "T06b", "empty speed field -> speed 0, speedValid false");

    // ---------- #6b : moved then stopped -> nothing frozen ----------
    resetPorts(); fake_millis = 90000;
    gMove.begin(); resetPorts();
    feedAndUpdate(gMove, ggaHdop(LAT, LON, "07", "1.20") + rmc('A', LAT, LON, "5.00", "123.40"));
    const GpsPosition moving = gMove.position();
    for (int i = 0; i < 50; ++i) gMove.update();                      // 50 Hz loop, no new sentence
    const uint32_t seqAfterRereads = gMove.position().courseSeq;
    fake_millis += 1000;
    feedAndUpdate(gMove, ggaHdop(LAT, LON, "07", "1.20") + rmc('A', LAT, LON, "0.30", "250.00"));
    const GpsPosition slow = gMove.position();
    fake_millis += 1000;
    feedAndUpdate(gMove, ggaHdop(LAT, LON, "07", "1.20") + rmc('A', LAT, LON, "", ""));   // stopped
    const GpsPosition& stopped = gMove.position();
    check(moving.courseValid && std::fabs(moving.courseDeg - 123.4f) < 0.01f && moving.courseSeq == 1,
          "T06c", "moving 5 kn: course 123.4 valid, one sample");
    check(std::fabs(moving.speedKmph - 9.26f) < 0.01f && moving.speedValid, "T06d", "moving: speed 9.26 km/h valid");
    check(seqAfterRereads == 1, "T06e", "50 updates without a new sentence do not create new samples");
    check(!slow.courseValid && slow.courseSeq == 1,
          "T06f", "0.3 kn (below GPS_COURSE_MIN_SPEED_KMPH): course present but discarded");
    check(!stopped.courseValid, "T06g", "stopped (empty field): courseValid FALSE - not frozen at 123.4");
    check(stopped.speedKmph == 0.0f && !stopped.speedValid, "T06h", "stopped: speed 0 - not frozen at 9.26");

    return finish();
}
