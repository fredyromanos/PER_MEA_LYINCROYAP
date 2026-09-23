#pragma once
#include "../core/Types.h"

// Decides whether the boat currently has a heading it can steer on.
//
// There is no compass: the only heading source is the GPS course, which is only
// meaningful while the boat moves. The gate turns the raw per-sentence course
// samples into a steering heading:
//   - valid after HEADING_CONFIRM_SAMPLES consecutive course samples at or above
//     HEADING_SPEED_ON_KMPH (no gap longer than HEADING_SAMPLE_GAP_MS);
//   - once valid, samples down to GPS_COURSE_MIN_SPEED_KMPH keep refreshing it
//     (hysteresis); when samples stop, the last heading is HELD for HEADING_HOLD_MS
//     (wave, gust) before the gate reports "no heading";
//   - fix lost → no heading immediately.
// Pure logic, time passed in → host-testable.
class HeadingGate {
public:
    enum Source : uint8_t { None = 0, Gps = 1, Held = 2 };

    struct State {
        bool    valid  = false;
        float   deg    = 0.0f;
        Source  source = None;
    };

    const State& update(const GpsPosition& pos, uint32_t nowMs);
    const State& state() const { return state_; }
    void reset();

private:
    State    state_;
    uint32_t lastSeq_       = 0;
    uint8_t  confirmCount_  = 0;
    bool     confirmed_     = false;   // enough samples seen since last loss
    uint32_t lastGoodMs_    = 0;       // time of the last sample that refreshed the heading
    float    lastGoodDeg_   = 0.0f;
    uint32_t lastSampleMs_  = 0;       // time of the last valid course sample (confirmation gaps)
};
