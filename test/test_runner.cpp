// ─────────────────────────────────────────────────────────────────────────────
// SeaDrone control-logic unit tests — CURRENT firmware API (2026-06-10+).
//
// Rewritten from scratch: the previous test_runner.cpp targeted a removed API
// (ManualServo/ManualProp modes, 3-arg update(frame,mode,millis), ESC arming,
// dual-ESC differential thrust, RcFrame.ch6, CH5 1300/1700 thresholds).
//
// This suite covers the units that are pure logic and host-compilable:
//   • ModeManager::decode()        — CH5 µs → ControlMode
//   • ManualController::update()   — unified manual: binary sail (CH2),
//                                    winch rotor (CH4), bidirectional ESC (CH3)
//
// Build: see test/CMakeLists.txt (native g++, no hardware).
// ─────────────────────────────────────────────────────────────────────────────
#include <cstdio>
#include <cstdint>

#include "core/Types.h"
#include "config/BoardConfig.h"
#include "config/Calibration.h"
#include "control/ModeManager.h"
#include "control/ManualController.h"

// ── tiny harness ─────────────────────────────────────────────────────────────
static int g_pass = 0;
static int g_fail = 0;

#define CHECK(label, expr)                                              \
    do {                                                                \
        if (expr) { printf("  PASS  %s\n", label); g_pass++; }          \
        else      { printf("  FAIL  %s  (line %d)\n", label, __LINE__); g_fail++; } \
    } while (0)

#define CHECK_EQ(label, got, want)                                      \
    do {                                                                \
        auto _g = (got); auto _w = (want);                              \
        if (_g == _w) { printf("  PASS  %s  (%ld)\n", label, (long)_g); g_pass++; } \
        else { printf("  FAIL  %s  got=%ld want=%ld  (line %d)\n",      \
                      label, (long)_g, (long)_w, __LINE__); g_fail++; } \
    } while (0)

// value in [lo,hi] inclusive
#define CHECK_RANGE(label, got, lo, hi)                                 \
    do {                                                                \
        auto _g = (got);                                                \
        if (_g >= (lo) && _g <= (hi)) { printf("  PASS  %s  (%ld in [%ld,%ld])\n", \
                label, (long)_g, (long)(lo), (long)(hi)); g_pass++; }   \
        else { printf("  FAIL  %s  got=%ld not in [%ld,%ld]  (line %d)\n", \
                label, (long)_g, (long)(lo), (long)(hi), __LINE__); g_fail++; } \
    } while (0)

static void section(const char* name) { printf("\n── %s\n", name); }

// Current RcFrame is 4 channels (CH2/3/4/5) — no CH6.
static RcFrame frame(uint16_t ch2 = 0, uint16_t ch3 = 0,
                     uint16_t ch4 = 0, uint16_t ch5 = 0) {
    RcFrame f; f.ch2 = ch2; f.ch3 = ch3; f.ch4 = ch4; f.ch5 = ch5; return f;
}

// Valid "carrier" channels so the ManualController channel-loss guard
// (ch2==0 || ch4==0 → safe neutral) does not fire while we probe one axis.
static constexpr uint16_t OK2 = 1600;  // sail stick (→ +1)
static constexpr uint16_t OK4 = 1500;  // rotor centered
static constexpr uint16_t OK3 = 1545;  // throttle neutral (CH3_CENTER_US)

// ═════════════════════════════════════════════════════════════════════════════
// ModeManager::decode  — CH5 thresholds: ≤1250 Auto | 1400–1600 Sail | >1800 Manual
//   ch5==0 → Failsafe ; out-of-band gaps → Sail (inert fallback)
// ═════════════════════════════════════════════════════════════════════════════
void test_ModeManager() {
    section("ModeManager::decode");
    ModeManager mm;
    auto dec = [&](uint16_t ch5){ return (uint8_t)mm.decode(frame(1500,1500,1500,ch5)); };

    CHECK_EQ("ch5=0    → Failsafe",              dec(0),    (uint8_t)ControlMode::Failsafe);
    CHECK_EQ("ch5=1000 → Automatic",             dec(1000), (uint8_t)ControlMode::Automatic);
    CHECK_EQ("ch5=1250 → Automatic (boundary ≤)",dec(1250), (uint8_t)ControlMode::Automatic);
    CHECK_EQ("ch5=1251 → Sail (gap fallback)",   dec(1251), (uint8_t)ControlMode::Sail);
    CHECK_EQ("ch5=1300 → Sail (gap fallback)",   dec(1300), (uint8_t)ControlMode::Sail);
    CHECK_EQ("ch5=1400 → Sail (band low)",       dec(1400), (uint8_t)ControlMode::Sail);
    CHECK_EQ("ch5=1500 → Sail (band mid)",       dec(1500), (uint8_t)ControlMode::Sail);
    CHECK_EQ("ch5=1600 → Sail (band high)",      dec(1600), (uint8_t)ControlMode::Sail);
    CHECK_EQ("ch5=1700 → Sail (gap fallback)",   dec(1700), (uint8_t)ControlMode::Sail);
    CHECK_EQ("ch5=1800 → Sail (boundary, not >)",dec(1800), (uint8_t)ControlMode::Sail);
    CHECK_EQ("ch5=1801 → Manual",                dec(1801), (uint8_t)ControlMode::Manual);
    CHECK_EQ("ch5=2000 → Manual",                dec(2000), (uint8_t)ControlMode::Manual);
}

// ═════════════════════════════════════════════════════════════════════════════
// Sail: binary ±10° from CH2, ±35 µs deadband around 1500, first-frame init.
// ═════════════════════════════════════════════════════════════════════════════
void test_Sail() {
    section("ManualController — sail (CH2 binary)");
    ManualController mc;

    // init from first frame
    mc.reset();
    CHECK_EQ("first frame ch2=1600 → SAIL_PLUS",
             mc.update(frame(1600, OK3, OK4)).sailUs, Calibration::SAIL_PLUS_US);
    mc.reset();
    CHECK_EQ("first frame ch2=1400 → SAIL_MINUS",
             mc.update(frame(1400, OK3, OK4)).sailUs, Calibration::SAIL_MINUS_US);
    mc.reset();
    CHECK_EQ("first frame ch2=1500 (=mid, >=) → SAIL_PLUS",
             mc.update(frame(1500, OK3, OK4)).sailUs, Calibration::SAIL_PLUS_US);
    mc.reset();
    CHECK_EQ("first frame ch2=1499 (<mid) → SAIL_MINUS",
             mc.update(frame(1499, OK3, OK4)).sailUs, Calibration::SAIL_MINUS_US);

    // toggle + deadband hold: init +1 then walk CH2
    mc.reset();
    mc.update(frame(1600, OK3, OK4));                                  // init +1
    CHECK_EQ("in-deadband 1500 → holds +1", mc.update(frame(1500,OK3,OK4)).sailUs, Calibration::SAIL_PLUS_US);
    CHECK_EQ("upper bound 1535 (=mid+db) → holds +1", mc.update(frame(1535,OK3,OK4)).sailUs, Calibration::SAIL_PLUS_US);
    CHECK_EQ("lower bound 1465 (=mid-db) → holds +1", mc.update(frame(1465,OK3,OK4)).sailUs, Calibration::SAIL_PLUS_US);
    CHECK_EQ("1464 (<mid-db) → snap -1", mc.update(frame(1464,OK3,OK4)).sailUs, Calibration::SAIL_MINUS_US);
    CHECK_EQ("1535 with state -1 → holds -1", mc.update(frame(1535,OK3,OK4)).sailUs, Calibration::SAIL_MINUS_US);
    CHECK_EQ("1536 (>mid+db) → snap +1", mc.update(frame(1536,OK3,OK4)).sailUs, Calibration::SAIL_PLUS_US);
}

// ═════════════════════════════════════════════════════════════════════════════
// Rotor: CH4 (1180–1790) → winch µs (1417–1583); ±35 µs deadband → 1500 center.
// mapUs clamps input to [CH4_MIN,CH4_MAX] first.
// ═════════════════════════════════════════════════════════════════════════════
void test_Rotor() {
    section("ManualController — rotor (CH4 winch)");
    ManualController mc;
    auto rot = [&](uint16_t ch4){ mc.reset(); mc.update(frame(OK2,OK3,OK4)); // init sail
                                  return mc.update(frame(OK2, OK3, ch4)).rotorUs; };

    CHECK_EQ("ch4=1500 (center) → ROTOR_CENTER",   rot(1500), Calibration::ROTOR_CENTER_US);
    CHECK_EQ("ch4=1465 (=mid-db) → ROTOR_CENTER",  rot(1465), Calibration::ROTOR_CENTER_US);
    CHECK_EQ("ch4=1535 (=mid+db) → ROTOR_CENTER",  rot(1535), Calibration::ROTOR_CENTER_US);
    CHECK_EQ("ch4=1180 (min) → ROTOR_MIN",         rot(1180), Calibration::ROTOR_MIN_US);
    CHECK_EQ("ch4=1790 (max) → ROTOR_MAX",         rot(1790), Calibration::ROTOR_MAX_US);
    CHECK_EQ("ch4=800 (<min, clamped) → ROTOR_MIN",rot(800),  Calibration::ROTOR_MIN_US);
    CHECK_EQ("ch4=2100 (>max, clamped) → ROTOR_MAX",rot(2100),Calibration::ROTOR_MAX_US);
    // linear interior point: 1417 + (1600-1180)*166/610 = 1531
    CHECK_EQ("ch4=1600 → 1531 (linear map)",       rot(1600), (uint16_t)1531);
    // monotonic + within physical envelope
    CHECK_RANGE("ch4=1300 in [MIN,CENTER]", rot(1300), Calibration::ROTOR_MIN_US, Calibration::ROTOR_CENTER_US);
    CHECK_RANGE("ch4=1700 in [CENTER,MAX]", rot(1700), Calibration::ROTOR_CENTER_US, Calibration::ROTOR_MAX_US);
}

// ═════════════════════════════════════════════════════════════════════════════
// ESC: CH3 bidirectional ratchet. center=1545, ±40 µs deadband → 1500 neutral.
//   forward (ch3<1545, toward 1100) → 1500..2000 ; reverse (>1545, toward 1990) → 1500..1000.
//   ch3==0 (lost) → 1500 neutral.
// ═════════════════════════════════════════════════════════════════════════════
void test_Esc() {
    section("ManualController — ESC (CH3 bidirectional)");
    ManualController mc;
    auto esc = [&](uint16_t ch3){ mc.reset(); mc.update(frame(OK2,OK3,OK4)); // init
                                  return mc.update(frame(OK2, ch3, OK4)).esc1Us; };

    CHECK_EQ("ch3=0 (lost) → NEUTRAL",             esc(0),    Calibration::ESC_NEUTRAL_US);
    CHECK_EQ("ch3=1545 (center) → NEUTRAL",        esc(1545), Calibration::ESC_NEUTRAL_US);
    CHECK_EQ("ch3=1585 (=+db) → NEUTRAL",          esc(1585), Calibration::ESC_NEUTRAL_US);
    CHECK_EQ("ch3=1505 (=-db) → NEUTRAL",          esc(1505), Calibration::ESC_NEUTRAL_US);
    CHECK_EQ("ch3=1100 (full fwd) → ESC_MAX",      esc(1100), Calibration::ESC_MAX_US);
    CHECK_EQ("ch3=1000 (<full fwd, clamped) → MAX",esc(1000), Calibration::ESC_MAX_US);
    CHECK_EQ("ch3=1990 (full rev) → ESC_REVERSE_MIN",esc(1990),Calibration::ESC_REVERSE_MIN_US);
    CHECK_EQ("ch3=2200 (>full rev, clamped) → REV_MIN",esc(2200),Calibration::ESC_REVERSE_MIN_US);
    // interior forward: frac=(1545-1322)/445 → 1500 + .5011*500 = 1750
    CHECK_EQ("ch3=1322 → 1750 (forward interp)",   esc(1322), (uint16_t)1750);
    // interior reverse: frac=(1768-1545)/445 → 1500 - .5011*500 = 1249
    CHECK_EQ("ch3=1768 → 1249 (reverse interp)",   esc(1768), (uint16_t)1249);
    // direction sanity
    CHECK_RANGE("forward side > neutral", esc(1200), Calibration::ESC_NEUTRAL_US+1, Calibration::ESC_MAX_US);
    CHECK_RANGE("reverse side < neutral", esc(1900), Calibration::ESC_REVERSE_MIN_US, Calibration::ESC_NEUTRAL_US-1);
}

// ═════════════════════════════════════════════════════════════════════════════
// Channel-loss guard: CH2 or CH4 == 0 → reset + safe neutral (ActuatorCommand{}).
// ═════════════════════════════════════════════════════════════════════════════
void test_ChannelLoss() {
    section("ManualController — channel-loss safe neutral");
    ManualController mc;

    mc.reset();
    mc.update(frame(1600, 1100, 1600));            // active command (fwd throttle, rotor off-center)
    {
        ActuatorCommand cmd = mc.update(frame(0, 1100, 1600));  // CH2 lost
        CHECK_EQ("ch2=0 → sail default 1520",  cmd.sailUs,  (uint16_t)1520);
        CHECK_EQ("ch2=0 → rotor default 1500", cmd.rotorUs, (uint16_t)1500);
        CHECK_EQ("ch2=0 → esc neutral 1500",   cmd.esc1Us,  (uint16_t)1500);
    }
    mc.reset();
    mc.update(frame(1600, 1100, 1600));
    {
        ActuatorCommand cmd = mc.update(frame(1600, 1100, 0));  // CH4 lost
        CHECK_EQ("ch4=0 → sail default 1520",  cmd.sailUs,  (uint16_t)1520);
        CHECK_EQ("ch4=0 → esc neutral 1500",   cmd.esc1Us,  (uint16_t)1500);
    }
}

// ── main ─────────────────────────────────────────────────────────────────────
int main() {
    printf("=== SeaDrone control-logic tests (current API) ===\n");
    test_ModeManager();
    test_Sail();
    test_Rotor();
    test_Esc();
    test_ChannelLoss();
    printf("\n=== Results: %d passed, %d failed ===\n", g_pass, g_fail);
    return (g_fail > 0) ? 1 : 0;
}
