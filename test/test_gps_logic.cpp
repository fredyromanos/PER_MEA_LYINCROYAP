// ─────────────────────────────────────────────────────────────────────────────
// GPS-only heading logic tests (no compass on the boat).
//
// Covers:
//   • HeadingGate      confirmation, speed hysteresis, hold, fix loss, duplicate samples
//   • AutoController   gps-lost neutral, heading acquisition push, timeout, resume,
//                      arrival without heading, navigation on a confirmed heading
//   • computeAutoPropulsionUs  frozen speed ignored, no heading → no heading branch
//   • Wind observation new-sample-only EMA, seeding, min samples, timeout
//   • Heartbeat        hv/fa fields, worst case within the 255-byte LoRa budget
// ─────────────────────────────────────────────────────────────────────────────
#include <cstdio>
#include <cstdint>
#include <cmath>
#include <cstring>

#include "control/HeadingGate.h"
#include "control/AutoController.h"
#include "comm/Heartbeat.h"
#include "config/Calibration.h"
#include "core/Types.h"

static int g_pass = 0;
static int g_fail = 0;

#define CHECK(label, expr)                                              \
    do {                                                                \
        if (expr) { printf("  PASS  %s\n", label); g_pass++; }          \
        else      { printf("  FAIL  %s  (line %d)\n", label, __LINE__); g_fail++; } \
    } while (0)

namespace C = Calibration;

static constexpr double LAT = 48.360415, LON = -4.566613;   // measured on the boat
static double northOf(double metres) { return LAT + metres / 111320.0; }

// A GPS sample as the driver publishes it.
static GpsPosition fix(double lat, float speedKmph, float courseDeg, uint32_t seq,
                       bool courseValid = true, bool speedValid = true) {
    GpsPosition p;
    p.valid = true;  p.lat = lat;  p.lon = LON;
    p.speedKmph = speedKmph;  p.speedValid = speedValid;
    p.courseDeg = courseDeg;  p.courseValid = courseValid;  p.courseSeq = seq;
    p.satellites = 8;  p.hdop = 1.2f;  p.ageMs = 100;
    return p;
}
static GpsPosition noFix(uint32_t seq) { GpsPosition p; p.courseSeq = seq; return p; }

static bool isNeutral(const ActuatorCommand& c) {
    ActuatorCommand n{};
    return c.sailUs == n.sailUs && c.rotorUs == n.rotorUs && c.esc1Us == n.esc1Us;
}

// ── HeadingGate ──────────────────────────────────────────────────────────────
static void testHeadingGate() {
    printf("\n[HeadingGate]\n");
    {
        HeadingGate g;
        CHECK("G1 no fix -> no heading", !g.update(noFix(0), 0).valid);
    }
    {
        HeadingGate g;
        g.update(fix(LAT, 2.0f, 90, 1), 1000);
        g.update(fix(LAT, 2.0f, 90, 2), 2000);
        CHECK("G2a two samples -> not yet trusted", !g.state().valid);
        const auto& s = g.update(fix(LAT, 2.0f, 91, 3), 3000);
        CHECK("G2b third consecutive sample -> heading valid from GPS",
              s.valid && s.source == HeadingGate::Gps && std::fabs(s.deg - 91) < 0.01f);
    }
    {
        HeadingGate g;
        g.update(fix(LAT, 2.0f, 90, 1), 1000);
        g.update(fix(LAT, 2.0f, 90, 2), 2000);
        g.update(fix(LAT, 1.2f, 90, 3), 3000);   // below ON speed before confirmation
        g.update(fix(LAT, 2.0f, 90, 4), 4000);
        CHECK("G3 slow sample during confirmation restarts the count", !g.state().valid);
    }
    {
        HeadingGate g;
        for (uint32_t i = 1; i <= 3; ++i) g.update(fix(LAT, 2.0f, 90, i), i * 1000);
        const auto& s = g.update(fix(LAT, 1.2f, 100, 4), 4000);   // between OFF and ON
        CHECK("G4 once trusted, 1.2 km/h keeps refreshing (hysteresis)",
              s.valid && s.source == HeadingGate::Gps && std::fabs(s.deg - 100) < 0.01f);
    }
    {
        HeadingGate g;
        for (uint32_t i = 1; i <= 3; ++i) g.update(fix(LAT, 2.0f, 90, i), i * 1000);
        // Samples stop (boat slows below GPS_COURSE_MIN_SPEED: courseValid false, seq frozen).
        GpsPosition slow = fix(LAT, 0.3f, 90, 3, false);
        CHECK("G5a 1.5 s after last sample -> still GPS",
              g.update(slow, 3000 + C::HEADING_SAMPLE_GAP_MS).source == HeadingGate::Gps);
        CHECK("G5b then HELD during HEADING_HOLD_MS",
              g.update(slow, 3000 + C::HEADING_SAMPLE_GAP_MS + 1000).source == HeadingGate::Held);
        CHECK("G5c after the hold -> no heading",
              !g.update(slow, 3000 + C::HEADING_SAMPLE_GAP_MS + C::HEADING_HOLD_MS + 1).valid);
        g.update(fix(LAT, 2.0f, 90, 4), 9000);
        CHECK("G5d after losing it, one sample is not enough again", !g.state().valid);
    }
    {
        HeadingGate g;
        g.update(fix(LAT, 2.0f, 90, 1), 1000);
        g.update(fix(LAT, 2.0f, 90, 2), 2000);
        g.update(fix(LAT, 2.0f, 90, 3), 2000 + C::HEADING_SAMPLE_GAP_MS + 500);   // gap
        CHECK("G6 gap between samples during confirmation restarts the count", !g.state().valid);
    }
    {
        HeadingGate g;
        for (uint32_t i = 1; i <= 3; ++i) g.update(fix(LAT, 2.0f, 90, i), i * 1000);
        CHECK("G7a fix lost -> no heading immediately", !g.update(noFix(3), 3100).valid);
        g.update(fix(LAT, 2.0f, 90, 4), 4000);
        CHECK("G7b fix back -> must re-confirm", !g.state().valid);
    }
    {
        HeadingGate g;
        for (uint32_t t = 0; t < 150; ++t) g.update(fix(LAT, 2.0f, 90, 1), 1000 + t * 20);  // 50 Hz, one sample
        CHECK("G8 the same 1 Hz sample read 150 times at 50 Hz counts once", !g.state().valid);
    }
}

// ── AutoController ───────────────────────────────────────────────────────────
static void testAutoController() {
    printf("\n[AutoController]\n");
    Waypoint wp;  wp.lat = northOf(200);  wp.lon = LON;  wp.radiusM = 10;
    const float wind = 270.0f;   // from the west: waypoint north is reachable

    {
        AutoController ac;
        ActuatorCommand c = ac.compute(wind, noFix(0), wp, 1000);
        CHECK("A1 fix invalid -> neutral command", isNeutral(c));
        CHECK("A1b mode gps-lost", std::strcmp(ac.navMode(), "gps-lost") == 0);
    }
    {
        AutoController ac;
        ActuatorCommand c = ac.compute(wind, fix(LAT, 0.0f, 0, 0, false, false), wp, 1000);
        CHECK("A2 no heading -> rudder centred", c.rotorUs == C::ROTOR_CENTER_US);
        CHECK("A2b no heading -> propeller cruise push", c.esc1Us == C::AUTO_ESC_CRUISE_US);
        // The acquire-heading fallback picks its tack from the documented initial
        // default NAV_SAIL_RIGHT_DEG (navigation.h) while the heading is still
        // unknown (sailAngle_ has not been set by real navigation yet).
        // AutoController::sailToUs maps sailAngleDeg >= 0 -> SAIL_PLUS_US, so the
        // one CORRECT tack here is SAIL_PLUS_US -- not "either tack", which let a
        // flipped-default mutant (NAV_SAIL_RIGHT_DEG -> NAV_SAIL_LEFT_DEG) survive.
        // (NAV_SAIL_RIGHT_DEG is a plain `static const double`, not constexpr, so
        // this can't be a static_assert -- verified by inspection: 10.0 >= 0.)
        CHECK("A2c no heading -> defaults to the documented initial tack (SAIL_PLUS_US)",
              c.sailUs == C::SAIL_PLUS_US);
        CHECK("A2d mode acquire-heading", std::strcmp(ac.navMode(), "acquire-heading") == 0);
        c = ac.compute(wind, fix(LAT, 0.0f, 0, 0, false, false), wp, 1000 + C::HEADING_ACQUIRE_TIMEOUT_MS + 1);
        CHECK("A3 still no heading after the timeout -> neutral", isNeutral(c));
        CHECK("A3b mode heading-timeout", std::strcmp(ac.navMode(), "heading-timeout") == 0);
    }
    {
        AutoController ac;
        ac.compute(wind, fix(LAT, 0.0f, 0, 0, false, false), wp, 1000);
        ActuatorCommand c = ac.compute(wind, noFix(0), wp, 2000);
        CHECK("A6a mid-mission fix loss -> neutral", isNeutral(c));
        c = ac.compute(wind, fix(LAT, 0.0f, 0, 0, false, false), wp, 3000);
        CHECK("A6b fix back -> resumes by itself (acquisition push)",
              c.esc1Us == C::AUTO_ESC_CRUISE_US && std::strcmp(ac.navMode(), "acquire-heading") == 0);
    }
    {
        AutoController ac;
        Waypoint here = wp;  here.lat = northOf(5);
        ActuatorCommand c = ac.compute(wind, fix(LAT, 0.0f, 0, 0, false, false), here, 1000);
        CHECK("A5 inside waypoint radius without heading -> neutral, reached",
              isNeutral(c) && std::strcmp(ac.navMode(), "reached") == 0);
    }
    {
        AutoController ac;
        ActuatorCommand c{};
        for (uint32_t i = 1; i <= 3; ++i)
            c = ac.compute(wind, fix(northOf(i * 1.0), 3.0f, 0.0f, i), wp, i * 1000);
        CHECK("A4 heading confirmed -> navigation runs (not acquiring, not lost)",
              ac.heading().valid &&
              std::strcmp(ac.navMode(), "acquire-heading") != 0 &&
              std::strcmp(ac.navMode(), "gps-lost") != 0);
        CHECK("A4b sailing at 3 km/h (> target) -> propeller stopped by propulsion logic",
              c.esc1Us == C::ESC_STOP_US);
    }
}

// ── Propulsion ───────────────────────────────────────────────────────────────
static void testPropulsion() {
    printf("\n[computeAutoPropulsionUs]\n");
    HeadingGate::State none;
    HeadingGate::State northH{true, 0.0f, HeadingGate::Gps};
    // Frozen speed from the library (9.26 km/h) but the receiver sends NO speed field.
    GpsPosition frozen = fix(LAT, 9.26f, 0, 0, false, false);
    CHECK("P1 stale speed not trusted -> boat considered stopped -> cruise push",
          AutoController::computeAutoPropulsionUs(frozen, none, 200, 0, 10) == C::AUTO_ESC_CRUISE_US);
    GpsPosition slowWrong = fix(LAT, 1.0f, 180, 1, true, true);
    CHECK("P2a heading valid and 180 deg off -> minimum thrust",
          AutoController::computeAutoPropulsionUs(slowWrong, HeadingGate::State{true, 180.0f, HeadingGate::Gps},
                                                  200, 0, 10) == C::AUTO_ESC_MIN_US);
    // P2b pins the ACTUAL propulsion gain (documented as ESC_STOP_US + speedError
    // * 180 us per km/h below target) instead of only excluding one sentinel
    // value. A gain blow-up mutant (e.g. *180.0f -> *5000.0f, a 28x change) still
    // lands outside AUTO_ESC_MIN_US and would have survived the old assertion.
    uint16_t p2b = AutoController::computeAutoPropulsionUs(slowWrong, none, 200, 0, 10);
    float expectedP2b = C::ESC_STOP_US + (C::AUTO_PROP_TARGET_SPEED_KMPH - 1.0f) * 180.0f;  // 1500+1*180=1680
    CHECK("P2b no heading, 1.0 km/h below target -> exact propulsion gain (1680 us)",
          std::fabs((float)p2b - expectedP2b) < 1.0f);

    // Second case: 0.1 km/h below target. The raw gain formula (1500+0.1*180=1518)
    // falls BELOW the documented autonomous floor AUTO_ESC_MIN_US (1600), so the
    // real code clamps it back up. A blown-up gain would instead push this case
    // to the opposite end of the envelope (clamped at AUTO_ESC_MAX_US=1850), so
    // this independently catches the same mutant from the other clamp direction.
    GpsPosition nearTarget = fix(LAT, 1.9f, 180, 2, true, true);
    uint16_t p2c = AutoController::computeAutoPropulsionUs(nearTarget, none, 200, 0, 10);
    CHECK("P2c no heading, 0.1 km/h below target -> clamped to the AUTO_ESC_MIN_US floor",
          p2c == C::AUTO_ESC_MIN_US);
    (void)northH;
}

// ── Wind observation ─────────────────────────────────────────────────────────
static void testWindObservation() {
    printf("\n[Wind observation]\n");
    {
        AutoController ac;
        ac.beginWindObservation();
        ac.observeWind(fix(LAT, 0.0f, 0, 0, false), 0);                 // anchor at start, no sample
        uint32_t t = 0;
        // 40 m travelled (distance condition met), but only ONE course sample, read 100× at 50 Hz.
        for (int k = 0; k < 100; ++k) { t += 20; ac.observeWind(fix(northOf(40), 2.0f, 200, 1), t); }
        CHECK("W1 one 1 Hz sample read 100 times -> not complete (needs 10 distinct)",
              !ac.windObsComplete());
        CHECK("W1b it counts as 1 sample (progress 10 %)", ac.windObsProgressPct() == 10);
    }
    {
        AutoController ac;
        ac.beginWindObservation();
        ActuatorCommand c = ac.observeWind(fix(LAT, 0.0f, 0, 0, false), 0);    // anchor, no sample
        CHECK("W0 observation never uses the propeller", c.esc1Us == ActuatorCommand{}.esc1Us);
        uint32_t seq = 0, t = 0;
        for (int k = 1; k <= 12; ++k) {
            t += 1000;
            ac.observeWind(fix(northOf(3.0 * k), 2.0f, 200, ++seq), t);
        }
        CHECK("W2 seeded with the first real course (200), not 0 -> wind = 290",
              ac.windObsComplete() && std::fabs(ac.observedWindDeg() - 290.0f) < 0.5f);
    }
    {
        AutoController ac;
        ac.beginWindObservation();
        ac.observeWind(fix(LAT, 0.0f, 0, 0, false), 0);
        uint32_t t = 0;
        for (uint32_t s = 1; s <= 5; ++s) { t += 1000; ac.observeWind(fix(northOf(50), 2.0f, 200, s), t); }
        CHECK("W3 distance reached but only 5 samples -> not complete", !ac.windObsComplete());
        CHECK("W3b progress limited by samples (50 %)", ac.windObsProgressPct() == 50);
    }
    {
        AutoController ac;
        ac.beginWindObservation();
        ac.observeWind(noFix(0), 0);
        ac.observeWind(noFix(0), C::WIND_OBS_TIMEOUT_MS + 1);
        CHECK("W4 no fix for the whole window -> observation failed",
              ac.windObsFailed() && !ac.windObsComplete());
    }
}

// ── Heartbeat ────────────────────────────────────────────────────────────────
static void testHeartbeat() {
    printf("\n[Heartbeat]\n");
    HeartbeatFields f;
    f.mode = "route-ready";  f.lat = -48.36041;  f.lon = -104.56661;
    f.sailDeg = -10;  f.rudderDeg = -110;  f.heading = 359;  f.headingSrc = 2;  f.fixAgeS = 99;
    f.windDeg = 359;  f.batVolts = 16.8f;  f.gpsFix = true;  f.sats = 12;  f.hdop = 99.9f;
    f.rcOk = true;  f.wptTotal = 16;  f.wptCur = 16;  f.windObs = true;  f.windObsPct = 100;
    char buf[320];
    const int len = formatHeartbeat(buf, sizeof buf, f);
    printf("  worst case: %d bytes\n", len);
    CHECK("H1 worst-case heartbeat fits the 255-byte LoRa packet",
          len > 0 && (size_t)len <= HEARTBEAT_LORA_MAX_BYTES);
    CHECK("H2 carries heading source and fix age",
          std::strstr(buf, "\"hv\":2,") && std::strstr(buf, "\"fa\":99,"));
    CHECK("H3 keeps the existing fields the IHM reads",
          std::strstr(buf, "\"fix\":1,") && std::strstr(buf, "\"wt\":16,\"wc\":16") &&
          std::strstr(buf, "\"wobs\":100}}"));
}

int main() {
    testHeadingGate();
    testAutoController();
    testPropulsion();
    testWindObservation();
    testHeartbeat();
    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
