#include "HeadingGate.h"
#include "../config/Calibration.h"

void HeadingGate::reset() {
    state_        = State{};
    confirmCount_ = 0;
    confirmed_    = false;
    lastGoodMs_   = 0;
    lastGoodDeg_  = 0.0f;
    lastSampleMs_ = 0;
    // lastSeq_ is kept: an already-consumed sample must not count again.
}

const HeadingGate::State& HeadingGate::update(const GpsPosition& pos, uint32_t nowMs) {
    if (!pos.valid) {
        reset();
        lastSeq_ = pos.courseSeq;
        return state_;
    }

    // courseSeq only advances on a valid course sample (speed >= GPS_COURSE_MIN_SPEED_KMPH).
    const bool newSample = pos.courseValid && pos.courseSeq != lastSeq_;
    if (newSample) {
        lastSeq_ = pos.courseSeq;
        if (!confirmed_) {
            const bool gap = confirmCount_ > 0 &&
                             (uint32_t)(nowMs - lastSampleMs_) > Calibration::HEADING_SAMPLE_GAP_MS;
            if (gap || pos.speedKmph < Calibration::HEADING_SPEED_ON_KMPH) confirmCount_ = 0;
            if (pos.speedKmph >= Calibration::HEADING_SPEED_ON_KMPH) confirmCount_++;
            if (confirmCount_ >= Calibration::HEADING_CONFIRM_SAMPLES) confirmed_ = true;
        }
        lastSampleMs_ = nowMs;
        if (confirmed_) {
            lastGoodMs_  = nowMs;
            lastGoodDeg_ = pos.courseDeg;
        }
    }

    if (!confirmed_) {
        state_ = State{};
        return state_;
    }

    const uint32_t age = (uint32_t)(nowMs - lastGoodMs_);
    if (age <= Calibration::HEADING_SAMPLE_GAP_MS) {
        state_ = State{true, lastGoodDeg_, Gps};
    } else if (age <= Calibration::HEADING_SAMPLE_GAP_MS + Calibration::HEADING_HOLD_MS) {
        state_ = State{true, lastGoodDeg_, Held};
    } else {
        const uint32_t seq = lastSeq_;
        reset();
        lastSeq_ = seq;
    }
    return state_;
}
