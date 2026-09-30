// ─────────────────────────────────────────────────────────────────────────────
// Navigation-core unit tests — the highest-risk, previously-zero-coverage brain.
//
// Covers:
//   • Navigator::distanceM / bearingDeg          (haversine geodesy)
//   • navigation.h pure helpers                  (angle math, clamps, geometry)
//   • nav_handleNavigationWithState branch matrix (direct/upwind/downwind/lofer/reached)
//   • nav_handleWindObservation                  (wind-acquire geometry + threshold)
//   • MissionManager state machine               (Idle→Running→Returning→Complete, Circuit)
//
// navigation.h is header-only and hardware-independent — included directly.
// ─────────────────────────────────────────────────────────────────────────────
#include <cstdio>
#include <cstdint>
#include <cmath>
#include <cstring>
#include <random>
#include <algorithm>
#include <chrono>

#include "navigation/navigation.h"
#include "navigation/Navigator.h"
#include "navigation/MissionManager.h"
#include "core/Types.h"

// ── tiny harness ─────────────────────────────────────────────────────────────
static int g_pass = 0;
static int g_fail = 0;

#define CHECK(label, expr)                                              \
    do {                                                                \
        if (expr) { printf("  PASS  %s\n", label); g_pass++; }          \
        else      { printf("  FAIL  %s  (line %d)\n", label, __LINE__); g_fail++; } \
    } while (0)

#define CHECK_NEAR(label, got, want, tol)                               \
    do {                                                                \
        double _g=(got), _w=(want), _t=(tol);                           \
        if (std::fabs(_g-_w) <= _t) { printf("  PASS  %s  (%.4f)\n", label, _g); g_pass++; } \
        else { printf("  FAIL  %s  got=%.4f want=%.4f tol=%.4f  (line %d)\n", \
                      label, _g, _w, _t, __LINE__); g_fail++; }          \
    } while (0)

#define CHECK_STR(label, got, want)                                     \
    do {                                                                \
        const char* _g=(got); const char* _w=(want);                    \
        if (_g && std::strcmp(_g,_w)==0) { printf("  PASS  %s  (\"%s\")\n", label, _g); g_pass++; } \
        else { printf("  FAIL  %s  got=\"%s\" want=\"%s\"  (line %d)\n",  \
                      label, _g?_g:"(null)", _w, __LINE__); g_fail++; }  \
    } while (0)

static void section(const char* name) { printf("\n── %s\n", name); }

// ═════════════════════════════════════════════════════════════════════════════
// Navigator — haversine distance + initial bearing
// ═════════════════════════════════════════════════════════════════════════════
void test_Navigator() {
    section("Navigator::distanceM / bearingDeg");

    CHECK_NEAR("same point → 0 m", Navigator::distanceM(48.34,-4.52,48.34,-4.52), 0.0, 0.01);
    // 1° of longitude at the equator ≈ 111195 m (R=6371000)
    CHECK_NEAR("(0,0)->(0,1) ≈ 111195 m", Navigator::distanceM(0,0,0,1), 111195.0, 60.0);
    // 1° of latitude anywhere ≈ 111195 m
    CHECK_NEAR("(0,0)->(1,0) ≈ 111195 m", Navigator::distanceM(0,0,1,0), 111195.0, 60.0);

    CHECK_NEAR("bearing east  → 90",  Navigator::bearingDeg(0,0,0,1),   90.0, 0.5);
    CHECK_NEAR("bearing north → 0",   Navigator::bearingDeg(0,0,1,0),    0.0, 0.5);
    CHECK_NEAR("bearing south → 180", Navigator::bearingDeg(0,0,-1,0), 180.0, 0.5);
    CHECK_NEAR("bearing west  → 270", Navigator::bearingDeg(0,0,0,-1), 270.0, 0.5);
}

// ═════════════════════════════════════════════════════════════════════════════
// navigation.h pure helpers
// ═════════════════════════════════════════════════════════════════════════════
void test_NavHelpers() {
    section("navigation.h — angle & geometry helpers");

    // relativeAngle wraps to (-180,180]
    CHECK_NEAR("relAngle(10,350) → -20",  nav_relativeAngle(10,350),  -20.0, 1e-9);
    CHECK_NEAR("relAngle(350,10) → +20",  nav_relativeAngle(350,10),   20.0, 1e-9);
    CHECK_NEAR("relAngle(0,180) → 180",   nav_relativeAngle(0,180),   180.0, 1e-9);

    // normalizeAngle → [0,360)
    CHECK_NEAR("normalize(-10) → 350", nav_normalizeAngle(-10), 350.0, 1e-9);
    CHECK_NEAR("normalize(370) → 10",  nav_normalizeAngle(370),  10.0, 1e-9);
    CHECK_NEAR("normalize(720) → 0",   nav_normalizeAngle(720),   0.0, 1e-9);

    // oppositeAngle
    CHECK_NEAR("opposite(10) → 190",  nav_oppositeAngle(10),  190.0, 1e-9);
    CHECK_NEAR("opposite(200) → 20",  nav_oppositeAngle(200),  20.0, 1e-9);

    // clampRudder → ±20 (NAV_RUDDER_CORRECTION_LIMIT_DEG)
    CHECK_NEAR("clampRudder(30) → 20",   nav_clampRudder(30.f),  20.0, 1e-6);
    CHECK_NEAR("clampRudder(-30) → -20", nav_clampRudder(-30.f),-20.0, 1e-6);
    CHECK_NEAR("clampRudder(12) → 12",   nav_clampRudder(12.f),  12.0, 1e-6);

    // rudderCommand: comp=relWind/2 + clamp(correction), then clamp ±110
    CHECK_NEAR("rudderCommand(100,180) → +110 (clamped)", nav_rudderCommand(100.f,180.0), 110.0, 1e-4);
    CHECK_NEAR("rudderCommand(-100,-180) → -110 (clamped)",nav_rudderCommand(-100.f,-180.0),-110.0,1e-4);
    CHECK_NEAR("rudderCommand(0,0) → 0",                   nav_rudderCommand(0.f,0.0),      0.0, 1e-6);

    // isBetweenOnShortestTurn
    CHECK("isBetween(45; start0,end90) → true",  nav_isBetweenOnShortestTurn(45,0,90));
    CHECK("isBetween(135; start0,end90) → false",!nav_isBetweenOnShortestTurn(135,0,90));

    // hasPassedWaypoint: path (0,0)->(0,0.001) (east ~111 m)
    CHECK("boat at start → not passed",
          !nav_hasPassedWaypoint(0,0, 0,0.001, 0,0));
    CHECK("boat beyond target → passed",
          nav_hasPassedWaypoint(0,0, 0,0.001, 0,0.002));

    // crossTrackError: path east-bound; boat north of path → nonzero, on path → ~0
    CHECK_NEAR("on-path cross-track ≈ 0",
               nav_crossTrackErrorMeters(0,0, 0,0.001, 0,0.0005), 0.0, 0.5);
    CHECK("off-path cross-track nonzero",
          std::fabs(nav_crossTrackErrorMeters(0,0, 0,0.001, 0.0002,0.0005)) > 1.0);
}

// ═════════════════════════════════════════════════════════════════════════════
// Angle-wrap boundaries — nav_normalizeAngle/nav_relativeAngle/nav_oppositeAngle
// were just rewritten from loop-based to fmod-based wrapping. This is where this
// codebase's angle bugs live, and nothing above pins the boundaries or the
// large-input performance that motivated the rewrite (a corrupted radio payload
// used to be able to stall the control loop for seconds on the ESP32 — see the
// historical note below). All expected values here were independently computed
// (Python math.fmod, matching C++ fmod semantics) and cross-checked against the
// documented ranges: normalizeAngle -> [0,360), relativeAngle -> (-180,180].
// ═════════════════════════════════════════════════════════════════════════════
void test_AngleWrapBoundaries() {
    section("angle wrap — exact boundaries, crossing, negatives, huge inputs");

    // ── exact boundaries ────────────────────────────────────────────────────
    CHECK_NEAR("normalize(0) → 0",     nav_normalizeAngle(0),     0.0, 1e-9);
    CHECK_NEAR("normalize(360) → 0",   nav_normalizeAngle(360),   0.0, 1e-9);
    CHECK_NEAR("normalize(180) → 180", nav_normalizeAngle(180), 180.0, 1e-9);
    CHECK_NEAR("normalize(-180) → 180",nav_normalizeAngle(-180),180.0, 1e-9);

    // Preserve the EXISTING convention at exactly +-180: relativeAngle's documented
    // range is (-180,180] (open at -180, closed at +180) -- pin what the code
    // actually does, not a preference. Both +180 and -180 as the raw difference
    // must land on the SAME output, +180 (never -180).
    CHECK_NEAR("relAngle(0,180) → 180 (upper bound, closed)",  nav_relativeAngle(0,180),  180.0, 1e-9);
    CHECK_NEAR("relAngle(0,-180) → 180 (lower bound wraps to +180, never -180)",
               nav_relativeAngle(0,-180), 180.0, 1e-9);
    CHECK_NEAR("relAngle(0,0) → 0",    nav_relativeAngle(0,0),     0.0, 1e-9);
    CHECK_NEAR("relAngle(0,360) → 0",  nav_relativeAngle(0,360),   0.0, 1e-9);

    CHECK_NEAR("opposite(0) → 180",    nav_oppositeAngle(0),   180.0, 1e-9);
    CHECK_NEAR("opposite(180) → 0",    nav_oppositeAngle(180),   0.0, 1e-9);
    CHECK_NEAR("opposite(-180) → 0",   nav_oppositeAngle(-180),  0.0, 1e-9);
    CHECK_NEAR("opposite(360) → 180",  nav_oppositeAngle(360), 180.0, 1e-9);

    // ── crossing case: wrapping through 0/360 must take the SHORT way ──────
    // reference 350, target 10 is a 20 deg step across the wrap, not -340.
    CHECK_NEAR("relAngle(350,10) → +20, NOT -340", nav_relativeAngle(350,10),  20.0, 1e-9);
    CHECK_NEAR("relAngle(10,350) → -20, NOT +340", nav_relativeAngle(10,350), -20.0, 1e-9);

    // ── negative inputs ─────────────────────────────────────────────────────
    CHECK_NEAR("normalize(-10) → 350",   nav_normalizeAngle(-10),  350.0, 1e-9);
    CHECK_NEAR("normalize(-370) → 350",  nav_normalizeAngle(-370), 350.0, 1e-9);
    CHECK_NEAR("relAngle(-170,170) → -20 (both negative/crossing)",
               nav_relativeAngle(-170,170), -20.0, 1e-9);
    CHECK_NEAR("relAngle(170,-170) → +20",
               nav_relativeAngle(170,-170),  20.0, 1e-9);

    // ── large out-of-range magnitudes: must wrap correctly AND return instantly.
    // The pre-fix loop-based nav_normalizeAngle took 5,965,232 iterations
    // (14.6 ms on x86, an estimated 0.5-1.5 s on a 240 MHz ESP32 -- a control-loop
    // stall) for input 2147483647. fmod is O(1) regardless of magnitude: this
    // test both pins the wrapped VALUE and, via the wall-clock budget below,
    // guards against that loop-based implementation ever regressing back in.
    CHECK_NEAR("normalize(400) → 40",          nav_normalizeAngle(400),   40.0, 1e-9);
    CHECK_NEAR("normalize(-500) → 220",        nav_normalizeAngle(-500), 220.0, 1e-9);
    CHECK_NEAR("normalize(9999) → 279",        nav_normalizeAngle(9999), 279.0, 1e-9);
    CHECK_NEAR("normalize(2147483647) → 127",  nav_normalizeAngle(2147483647.0), 127.0, 1e-6);
    CHECK_NEAR("normalize(-2147483647) → 233", nav_normalizeAngle(-2147483647.0),233.0, 1e-6);

    CHECK_NEAR("relAngle(0,400) → 40",          nav_relativeAngle(0,400),    40.0, 1e-9);
    CHECK_NEAR("relAngle(0,-500) → -140",       nav_relativeAngle(0,-500), -140.0, 1e-9);
    CHECK_NEAR("relAngle(0,9999) → -81",        nav_relativeAngle(0,9999),  -81.0, 1e-9);
    CHECK_NEAR("relAngle(0,2147483647) → 127",  nav_relativeAngle(0,2147483647.0), 127.0, 1e-6);
    CHECK_NEAR("relAngle(0,-2147483647) → -127",nav_relativeAngle(0,-2147483647.0),-127.0,1e-6);

    // Wall-clock budget: 100,000 calls at this magnitude must complete well
    // under the time a SINGLE old loop-based call used to take (14.6 ms).
    // A generous 50 ms ceiling still leaves ~1000x headroom over O(1) fmod cost
    // while being tight enough to fail hard if a loop ever creeps back in.
    auto t0 = std::chrono::steady_clock::now();
    volatile double sink = 0.0;
    for (int i = 0; i < 100000; ++i) {
        sink += nav_normalizeAngle(2147483647.0);
        sink += nav_relativeAngle(0.0, -2147483647.0);
    }
    auto t1 = std::chrono::steady_clock::now();
    double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    (void)sink;
    CHECK(("huge-input wrap returns instantly: 200000 calls < 50 ms (took " +
           std::to_string(ms) + " ms)").c_str(), ms < 50.0);
}

// ═════════════════════════════════════════════════════════════════════════════
// nav_handleNavigationWithState — force each decision branch by construction.
// lat/lng left at 0 and corridorHalfWidthM=0 → corridor disabled; heading/wind rule.
// ═════════════════════════════════════════════════════════════════════════════
static NavResult run(NavState& st, double boatHdg, double wptHdg, double dist,
                     double windDir, double reachedM) {
    return nav_handleNavigationWithState(
        st, boatHdg, wptHdg, dist, windDir,
        /*sail*/ 0.0f, /*rudder*/ 0.0f, reachedM,
        /*lat*/0,0, /*wpt*/0,0, /*corridor*/0.0);
}

void test_NavBranches() {
    section("nav_handleNavigationWithState — branch matrix");

    { NavState st{}; nav_resetState(st);
      NavResult r = run(st, 0, 0, /*dist*/3, /*wind*/90, /*reached*/10);
      CHECK("dist<=reached → waypointReached", r.waypointReached); }

    { NavState st{}; nav_resetState(st);
      // waypoint due north, wind from north → straight upwind (no-go)
      NavResult r = run(st, 90, 0, 500, 0, 10);
      CHECK_STR("waypoint upwind → mode upwind-zigzag", r.mode, "upwind-zigzag");
      CHECK("upwind rudder within ±110", std::fabs(r.rudderAngle) <= 110.0); }

    { NavState st{}; nav_resetState(st);
      // waypoint downwind (heading 180, wind from north → opposite=180 aligned)
      NavResult r = run(st, 180, 180, 500, 0, 10);
      CHECK("waypoint downwind → mode mentions downwind/avoid",
            r.mode && (std::strstr(r.mode,"downwind") || std::strstr(r.mode,"avoid"))); }

    { NavState st{}; nav_resetState(st);
      // aligned: boat & waypoint both heading 0, wind abeam (90) → direct.
      // CHARACTERIZATION (documents behaviour, NOT a correctness claim): even when
      // aligned, the rudder holds the wind feed-forward nav_rudderCommand(0,relWind)
      // = relWind/2. Whether relWind/2 is the *right* magnitude is unverified — it is
      // a bench/water question (see docs/ADVERSARIAL_FINDINGS.md, ±25% mistune halves
      // convergence). This test locks the current behaviour, it does not bless it.
      NavResult r = run(st, 0, 0, 500, 90, 10);
      CHECK_STR("aligned → mode direct", r.mode, "direct");
      double relWind = nav_relativeAngle(0, 90);              // = 90
      double expect  = nav_rudderCommand(0.0f, relWind);      // = 45 (wind compensation)
      CHECK_NEAR("[characterization] direct rudder = wind feed-forward (relWind/2)", r.rudderAngle, expect, 1e-3); }

    { NavState st{}; nav_resetState(st);
      // reaching angle, no zone → lofer or abattre
      NavResult r = run(st, 0, 40, 500, 90, 10);
      CHECK("off-axis reach → lofer|abattre",
            r.mode && (std::strstr(r.mode,"lofer") || std::strstr(r.mode,"abattre")));
      CHECK("lofer/abattre rudder within ±110", std::fabs(r.rudderAngle) <= 110.0); }
}

// ═════════════════════════════════════════════════════════════════════════════
// nav_handleWindObservation — latch wind only past the required distance.
// ═════════════════════════════════════════════════════════════════════════════
void test_WindObservation() {
    section("nav_handleWindObservation");

    NavResult a = nav_handleWindObservation(0,0, 0,0, /*smoothHdg*/0, /*dist*/10, /*req*/30, 10.f, 0.f);
    CHECK("below distance → wind NOT acquired", !a.windAcquired);

    NavResult b = nav_handleWindObservation(0,0, 0,0, /*smoothHdg*/0, /*dist*/40, /*req*/30, 10.f, 0.f);
    CHECK("past distance → wind acquired", b.windAcquired);
    CHECK_NEAR("acquired dir = smoothHdg+90 (0→90)", b.acquiredWindDir, 90.0, 1e-6);

    NavResult c = nav_handleWindObservation(0,0, 0,0, /*smoothHdg*/300, 40, 30, 10.f, 0.f);
    CHECK_NEAR("acquired dir wraps (300+90→30)", c.acquiredWindDir, 30.0, 1e-6);
}

// ═════════════════════════════════════════════════════════════════════════════
// MissionManager — state machine
// ═════════════════════════════════════════════════════════════════════════════
static GpsPosition gp(double lat, double lon) {
    GpsPosition p; p.valid = true; p.lat = lat; p.lon = lon; return p;
}

void test_MissionManager() {
    section("MissionManager — Linear + Circuit state machine");
    MissionManager mm;
    Waypoint tgt;

    // no waypoints: start() is a no-op, update() returns false (Idle)
    CHECK("empty mission: update false", !mm.update(gp(48.34,-4.52), tgt));
    mm.start();
    CHECK("empty mission: start() stays Idle", (uint8_t)mm.state()==(uint8_t)MissionState::Idle);

    // Linear plan, 2 waypoints, home set
    MissionPlan plan{};
    plan.mode = MissionMode::Linear;
    plan.waypoints[0] = {48.3400, -4.5100, 10.0f};
    plan.waypoints[1] = {48.3500, -4.5000, 10.0f};
    plan.count = 2;
    mm.loadMission(plan);
    mm.setHome(48.3000, -4.5300);
    mm.start();
    CHECK("after start → Running", (uint8_t)mm.state()==(uint8_t)MissionState::Running);

    // far from wp0 → target is wp0, still running
    CHECK("running yields target", mm.update(gp(48.20,-4.60), tgt));
    CHECK_NEAR("target is wp0 lat", tgt.lat, 48.3400, 1e-9);
    CHECK("index 0", mm.currentIndex()==0);

    // arrive at wp0 → advance to wp1
    mm.update(gp(48.3400,-4.5100), tgt);
    CHECK("advanced to wp1", mm.currentIndex()==1);

    // arrive at wp1 (last, Linear) → Returning toward home
    mm.update(gp(48.3500,-4.5000), tgt);
    CHECK("last wp reached → Returning", (uint8_t)mm.state()==(uint8_t)MissionState::Returning);
    bool active = mm.update(gp(48.3500,-4.5000), tgt);
    CHECK("returning yields home target", active);
    CHECK_NEAR("target is home lat", tgt.lat, 48.3000, 1e-9);

    // arrive home → Complete, no more target
    bool stillActive = mm.update(gp(48.3000,-4.5300), tgt);
    CHECK("home reached → not active", !stillActive);
    CHECK("state Complete", (uint8_t)mm.state()==(uint8_t)MissionState::Complete);

    // Circuit mode wraps instead of returning
    MissionManager cm;
    MissionPlan cplan{};
    cplan.mode = MissionMode::Circuit;
    cplan.waypoints[0] = {48.3400,-4.5100,10.0f};
    cplan.waypoints[1] = {48.3500,-4.5000,10.0f};
    cplan.count = 2;
    cm.loadMission(cplan);
    cm.start();
    cm.update(gp(48.3400,-4.5100), tgt);   // reach wp0 → wp1
    CHECK("circuit: idx→1", cm.currentIndex()==1);
    cm.update(gp(48.3500,-4.5000), tgt);   // reach wp1 → wraps to wp0
    CHECK("circuit: wraps idx→0", cm.currentIndex()==0);
    CHECK("circuit: stays Running", (uint8_t)cm.state()==(uint8_t)MissionState::Running);

    // stop() → Idle
    cm.stop();
    CHECK("stop → Idle", (uint8_t)cm.state()==(uint8_t)MissionState::Idle);
}

// ═════════════════════════════════════════════════════════════════════════════
// MissionManager — boundary: a plan at exactly MissionPlan::MAX_WAYPOINTS (16).
// CLAUDE.md's architecture section says "max 32" -- the code (MissionPlan.h) is
// authoritative at 16. Nothing exercised the array-filling boundary before.
// ═════════════════════════════════════════════════════════════════════════════
void test_MissionManagerMaxWaypoints() {
    section("MissionManager — plan at exactly MAX_WAYPOINTS (16)");

    static_assert(MissionPlan::MAX_WAYPOINTS == 16,
                  "test assumes the documented 16-waypoint boundary");

    MissionManager mm;
    MissionPlan plan{};
    plan.mode = MissionMode::Linear;
    for (uint8_t i = 0; i < MissionPlan::MAX_WAYPOINTS; ++i) {
        plan.waypoints[i] = {48.3000 + 0.001 * i, -4.5000, 10.0f};
    }
    plan.count = MissionPlan::MAX_WAYPOINTS;
    mm.loadMission(plan);
    mm.setHome(48.2900, -4.5000);
    mm.start();

    CHECK("plan reports 16 waypoints", mm.waypointCount() == MissionPlan::MAX_WAYPOINTS);
    CHECK("start → Running", (uint8_t)mm.state() == (uint8_t)MissionState::Running);
    CHECK("starts at index 0", mm.currentIndex() == 0);

    Waypoint tgt;
    // Walk every waypoint 0..14 by arriving exactly on it; each arrival must
    // advance to the next index without skipping or wrapping early.
    for (uint8_t i = 0; i + 1 < MissionPlan::MAX_WAYPOINTS; ++i) {
        bool active = mm.update(gp(plan.waypoints[i].lat, plan.waypoints[i].lon), tgt);
        char label[96];
        std::snprintf(label, sizeof label, "wp[%u] reached -> advances to idx %u", i, i + 1);
        CHECK(label, active && mm.currentIndex() == (uint8_t)(i + 1));
    }
    CHECK("walked all the way to the LAST waypoint (idx 15)",
          mm.currentIndex() == MissionPlan::MAX_WAYPOINTS - 1);
    CHECK("still Running just before the last arrival",
          (uint8_t)mm.state() == (uint8_t)MissionState::Running);

    // Arrive on waypoint 15 (the last one): must transition to Returning, not
    // wrap back into the array (off-by-one at the MAX_WAYPOINTS boundary would
    // either skip this or index past the fixed-size array).
    const uint8_t last = MissionPlan::MAX_WAYPOINTS - 1;
    bool activeAfterLast = mm.update(gp(plan.waypoints[last].lat, plan.waypoints[last].lon), tgt);
    CHECK("last waypoint (15) reached -> Returning", (uint8_t)mm.state() == (uint8_t)MissionState::Returning);
    CHECK("Returning yields the home target", activeAfterLast && std::fabs(tgt.lat - 48.2900) < 1e-9);

    // Arrive home -> Complete.
    bool stillActive = mm.update(gp(48.2900, -4.5000), tgt);
    CHECK("home reached -> mission Complete", !stillActive &&
          (uint8_t)mm.state() == (uint8_t)MissionState::Complete);
}

// ═════════════════════════════════════════════════════════════════════════════
// Falsifiable invariants (randomized) — properties that MUST hold, with checks
// that the assertions are non-vacuous and discriminating (can actually fail).
// ═════════════════════════════════════════════════════════════════════════════
void test_NavProperties() {
    section("navigation.h — randomized invariants + decomposition identity");
    std::mt19937 rng(2024);
    std::uniform_real_distribution<double> A(-540, 540), W(-180, 180);
    bool boundOK = true, idOK = true;
    int cmdWide = 0, corrRecovered = 0;
    for (int i = 0; i < 5000; ++i) {
        double corr = A(rng), relWind = W(rng);
        float cmd = nav_rudderCommand((float)corr, relWind);
        if (std::fabs(cmd) > NAV_RUDDER_COMMAND_LIMIT_DEG + 1e-3f) boundOK = false;
        float back = nav_rudderCorrection(cmd, relWind);
        if (std::fabs(back) > NAV_RUDDER_CORRECTION_LIMIT_DEG + 1e-3f) boundOK = false;
        float expected = std::max(-20.0f, std::min(20.0f, (float)corr));
        // Decomposition holds when the ±110 command clamp is not active. By design
        // |cmd| = |comp(≤90) + clamp(corr,±20)| ≤ 110, so the clamp is essentially
        // never hit — that itself is a property; we test the identity on the bulk.
        if (std::fabs(cmd) < NAV_RUDDER_COMMAND_LIMIT_DEG - 1e-3f) {
            if (std::fabs(back - expected) > 1e-2f) idOK = false;
            if (std::fabs(expected) > 1e-3f) corrRecovered++;
        }
        if (std::fabs(cmd) > 60.0f) cmdWide++;   // wide range ⇒ test is non-vacuous
    }
    CHECK("INVARIANT: |command| <= 110 AND |correction| <= 20 (5000 samples)", boundOK);
    CHECK("INVARIANT: correction(command(c,w),w) == clamp(c,+-20) when unsaturated", idOK);
    CHECK("non-vacuous: commands span a wide range (some |cmd|>60)", cmdWide > 0);
    CHECK("non-vacuous: nonzero corrections actually recovered", corrRecovered > 0);
}

void test_WindObsSweep() {
    section("nav_handleWindObservation — latch invariant across a heading sweep");
    bool latchOK = true, dirOK = true, nonvac = false;
    for (int h = 0; h < 360; h += 15) {
        NavResult below = nav_handleWindObservation(0,0, 0,0, (double)h, 10, 30, 10.f, 0.f);
        NavResult above = nav_handleWindObservation(0,0, 0,0, (double)h, 40, 30, 10.f, 0.f);
        if (below.windAcquired) latchOK = false;     // must NOT latch before required distance
        if (!above.windAcquired) latchOK = false;     // must latch after
        if (above.windAcquired) {
            nonvac = true;
            double expect = std::fmod(h + 90.0 + 360.0, 360.0);
            if (std::fabs(above.acquiredWindDir - expect) > 1e-6) dirOK = false;
        }
    }
    CHECK("INVARIANT: latches iff travelled >= required distance (all headings)", latchOK);
    CHECK("INVARIANT: acquired wind = smoothHeading+90 (all headings)", dirOK);
    CHECK("non-vacuous: some headings latched", nonvac);
    // discriminating power: the SAME data must NOT satisfy a would-be-wrong +80° rule
    NavResult chk = nav_handleWindObservation(0,0, 0,0, 0.0, 40, 30, 10.f, 0.f);
    CHECK("discriminating: acquired dir != smoothHeading+80 (wrong-offset control)",
          std::fabs(chk.acquiredWindDir - 80.0) > 1.0);
}

// ═════════════════════════════════════════════════════════════════════════════
// Auto-trim (learnt rudder offset) — 2026-09-16 GPS work.
// A constant mechanical rudder offset (winch off-centre, play) must be learnt and
// cancelled by an integral term applied OUTSIDE the ±20° correction clamp, so an
// offset larger than the clamp can no longer leave the boat sailing straight away
// from its waypoint forever.
// ═════════════════════════════════════════════════════════════════════════════
static NavResult runSteer(NavState& st, double boatHdg, double wptHdg, double dist,
                          double windDir, double dtS) {
    return nav_handleNavigationWithState(
        st, boatHdg, wptHdg, dist, windDir,
        /*sail*/ 0.0f, /*rudder*/ 0.0f, /*reached*/ 10.0,
        /*lat*/0,0, /*wpt*/0,0, /*corridor*/0.0, dtS);
}

void test_NavTrim() {
    section("navigation.h — auto-trim (learnt rudder offset)");

    // Reset helpers keep the trim on resetManoeuvre and zero it on resetTrim.
    { NavState st{}; nav_resetState(st);
      st.rudderTrimDeg = 7.5;
      nav_resetManoeuvre(st);
      CHECK_NEAR("resetManoeuvre keeps the learnt trim (hardware property)", st.rudderTrimDeg, 7.5, 1e-9);
      nav_resetTrim(st);
      CHECK_NEAR("resetTrim zeroes it", st.rudderTrimDeg, 0.0, 1e-9); }

    // Anti wind-up: a heading error OUTSIDE the learning window must not learn trim.
    { NavState st{}; nav_resetState(st);
      // boat 90°, waypoint 0°, wind from 0° → upwind-zigzag branch, |error| ≈ 135° > 30°.
      runSteer(st, 90, 0, 500, 0, 10.0);
      CHECK_NEAR("no learning when |heading error| > NAV_TRIM_ERROR_WINDOW_DEG",
                 st.rudderTrimDeg, 0.0, 1e-9); }

    // Learning: aligned-direct branch (|error| ≤ 5°). Positive error ⇒ trim drives negative.
    { NavState st{}; nav_resetState(st);
      // boat 0°, waypoint 3°, wind abeam (90°) → aligned-direct, headingError = +3°.
      double prev = 0.0;
      NavResult r = runSteer(st, 0, 3, 500, 90, 10.0);
      CHECK("trim moves in the error-correcting direction (positive error ⇒ negative trim)",
            st.rudderTrimDeg < prev);
      prev = st.rudderTrimDeg;
      runSteer(st, 0, 3, 500, 90, 10.0);
      CHECK("trim keeps integrating while the error persists", st.rudderTrimDeg < prev);
      (void)r; }

    // Saturation at ±NAV_TRIM_MAX_DEG whatever the (in-window) error size.
    { NavState st{}; nav_resetState(st);
      for (int i = 0; i < 200; ++i) runSteer(st, 0, 3, 500, 90, 10.0);
      CHECK_NEAR("trim saturates at NAV_TRIM_MAX_DEG (negative side)",
                 st.rudderTrimDeg, -NAV_TRIM_MAX_DEG, 1e-6); }

    // Trim applied OUTSIDE the ±20° correction clamp: the correction saturates at ±20°,
    // the final command therefore exceeds the correction limit.
    { NavState st{}; nav_resetState(st);
      // wind 0°, waypoint 40° (upwind zone) with boat 18° → tracked target = 45°,
      // headingError = +27°: inside the 30° trim window but large enough to saturate the
      // correction (±20°). Positive error ⇒ the trim learns negative.
      NavResult r = runSteer(st, 18, 40, 500, 0, 10.0);
      CHECK("in-window saturated-correction error learns trim", st.rudderTrimDeg != 0.0);
      CHECK("trim moves to cancel a positive heading error (learns negative)",
            st.rudderTrimDeg < 0.0);
      for (int i = 0; i < 40; ++i) r = runSteer(st, 18, 40, 500, 0, 10.0);
      CHECK_NEAR("trim saturates at -NAV_TRIM_MAX_DEG", st.rudderTrimDeg, -NAV_TRIM_MAX_DEG, 1e-6);
      CHECK("command exceeds the ±20° correction clamp (trim is added outside it)",
            r.rudderAngle < -(NAV_RUDDER_CORRECTION_LIMIT_DEG + 1.0));
      CHECK("command stays within the ±110° actuator limit",
            std::fabs(r.rudderAngle) <= NAV_RUDDER_COMMAND_LIMIT_DEG + 1e-6f); }
}

// ═════════════════════════════════════════════════════════════════════════════
// Manoeuvre watchdog — no state may block forever (2026-09-16). An avoid-gybe
// phase whose condition can never be met (e.g. an un-cancelled rudder offset)
// must time out and advance; a lack of progress toward the waypoint must reset
// the manoeuvre (keeping the learnt trim).
// ═════════════════════════════════════════════════════════════════════════════
void test_NavWatchdog() {
    section("navigation.h — manoeuvre watchdog (phase timeout / no-progress reset)");

    // Phase timeout: stuck in avoid-gybe phase 1 (condition <10° never met) → advances.
    { NavState st{}; nav_resetState(st);
      st.empannagePhase = NAV_EMPANNAGE_ALLER_LIMITE_UPWIND;
      st.watchedPhase   = NAV_EMPANNAGE_ALLER_LIMITE_UPWIND;
      st.phaseTimeS     = NAV_WATCHDOG_PHASE_TIMEOUT_S - 1.0;
      st.downwindSide   = 1;
      // boat 90°, wind 0° → phase-1 target = 315°, |error| = 135° ⇒ condition never met.
      runSteer(st, 90, 0, 500, 0, 0.5);   // 0.5 s → still 0.5 s short of timeout
      CHECK("phase stays put while under the timeout",
            st.empannagePhase == NAV_EMPANNAGE_ALLER_LIMITE_UPWIND);
      CHECK("phase timer accumulates the real dt", st.phaseTimeS > NAV_WATCHDOG_PHASE_TIMEOUT_S - 1.0);
      runSteer(st, 90, 0, 500, 0, 1.0);                      // now past the timeout
      CHECK("phase advances after NAV_WATCHDOG_PHASE_TIMEOUT_S even if the condition is never met",
            st.empannagePhase == NAV_EMPANNAGE_CROISER_AXE_UPWIND); }

    // No-progress reset: no gain for NAV_WATCHDOG_NO_PROGRESS_S → manoeuvre reset, trim kept.
    { NavState st{}; nav_resetState(st);
      st.rudderTrimDeg = 7.0;                                // learnt offset must survive
      st.bestDistanceM = 100.0;                              // progress needs dist < 95 m
      st.noProgressS   = NAV_WATCHDOG_NO_PROGRESS_S - 0.5;
      // boat 0°, waypoint 0°, wind abeam → aligned-direct; distance held at 100 m (no progress).
      NavResult r = runSteer(st, 0, 0, 100.0, 90, 1.0);
      CHECK("no-progress watchdog fires after NAV_WATCHDOG_NO_PROGRESS_S without gain",
            std::strcmp(r.logMessage, "Watchdog: no progress, manoeuvre reset") == 0);
      CHECK_NEAR("no-progress reset zeroes the progress clock", st.noProgressS, 0.0, 1e-9);
      CHECK_NEAR("no-progress reset keeps the learnt trim", st.rudderTrimDeg, 7.0, 1e-9);
      CHECK("no-progress reset clears the manoeuvre phase",
            st.empannagePhase == NAV_EMPANNAGE_AUCUN); }

    // Control: real progress (distance drops > NAV_WATCHDOG_PROGRESS_M) resets the clock,
    // so the watchdog does NOT fire spuriously.
    { NavState st{}; nav_resetState(st);
      st.bestDistanceM = 100.0;
      st.noProgressS   = NAV_WATCHDOG_NO_PROGRESS_S - 0.5;
      NavResult r = runSteer(st, 0, 0, 90.0, 90, 1.0);       // 100→90 m: 10 m gained
      CHECK("real progress prevents the no-progress reset",
            std::strcmp(r.logMessage, "Watchdog: no progress, manoeuvre reset") != 0);
      CHECK_NEAR("progress clock reset on a ≥5 m gain", st.noProgressS, 0.0, 1e-9);
      CHECK_NEAR("best distance updated", st.bestDistanceM, 90.0, 1e-6); }
}

// ── main ─────────────────────────────────────────────────────────────────────
int main() {
    printf("=== SeaDrone navigation-core tests ===\n");
    test_Navigator();
    test_NavHelpers();
    test_AngleWrapBoundaries();
    test_NavBranches();
    test_WindObservation();
    test_MissionManager();
    test_MissionManagerMaxWaypoints();
    test_NavProperties();
    test_WindObsSweep();
    test_NavTrim();
    test_NavWatchdog();
    printf("\n=== Results: %d passed, %d failed ===\n", g_pass, g_fail);
    return (g_fail > 0) ? 1 : 0;
}
