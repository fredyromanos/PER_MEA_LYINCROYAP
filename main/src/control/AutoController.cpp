#include "AutoController.h"
#include "../navigation/Navigator.h"
#include "../config/Calibration.h"
#include <math.h>

// Sail is binary ±10° → two discrete µs positions.
uint16_t AutoController::sailToUs(float sailAngleDeg) {
    return (sailAngleDeg >= 0.0f) ? Calibration::SAIL_PLUS_US
                                  : Calibration::SAIL_MINUS_US;
}

// Convert the navigation rudder command (degrees) into a winch µs target.
// Autonomous steering uses the FULL range the navigation expects
// (±NAV_RUDDER_COMMAND_LIMIT_DEG = ±110°) — no artificial envelope — mapped 1:1
// onto physical winch degrees, exactly like the simulation.
uint16_t AutoController::rudderToUs(float rudderAngleDeg) {
    // Defensive clamp to the navigation command range (nav already bounds it).
    const float lim = NAV_RUDDER_COMMAND_LIMIT_DEG;
    if (rudderAngleDeg >  lim) rudderAngleDeg =  lim;
    if (rudderAngleDeg < -lim) rudderAngleDeg = -lim;

    // 1° of nav command == 1° of physical winch rotation (same scale as manual).
    const float usPerDeg =
        (float)(Calibration::ROTOR_MAX_US - Calibration::ROTOR_CENTER_US)
        / Calibration::ROTOR_RANGE_DEG;          // 83 µs / 90° ≈ 0.92 µs/°
    int32_t us = (int32_t)Calibration::ROTOR_CENTER_US
               + (int32_t)lroundf(rudderAngleDeg * usPerDeg);

    // Safety clamp to the autonomous winch envelope (±110°, wider than manual).
    if (us < (int32_t)Calibration::ROTOR_AUTO_MIN_US) us = Calibration::ROTOR_AUTO_MIN_US;
    if (us > (int32_t)Calibration::ROTOR_AUTO_MAX_US) us = Calibration::ROTOR_AUTO_MAX_US;
    return (uint16_t)us;
}

ActuatorCommand AutoController::compute(float windDeg,
                                        const GpsPosition& pos,
                                        const Waypoint& target,
                                        uint32_t nowMs) {
    ActuatorCommand cmd{};  // safe neutral defaults if we bail early

    // Time since the previous call (control tick). Bounded: a first call or a long
    // pause must not inject a huge dt into the trim integrator or the watchdog.
    float dtS = (lastComputeMs_ == 0) ? 0.0f : (float)(nowMs - lastComputeMs_) / 1000.0f;
    if (dtS < 0.0f || dtS > 1.0f) dtS = 0.0f;
    lastComputeMs_ = nowMs;

    const HeadingGate::State& heading = gate_.update(pos, nowMs);

    // 1. No usable fix: neutral, resume automatically when the fix comes back.
    if (!pos.valid) {
        acquiring_   = false;
        rudderAngle_ = 0.0f;
        navMode_     = "gps-lost";
        navMessage_  = "GPS fix lost: neutral until fix returns";
        return cmd;
    }

    float dist    = Navigator::distanceM(pos.lat, pos.lon, target.lat, target.lon);
    float bearing = Navigator::bearingDeg(pos.lat, pos.lon, target.lat, target.lon);

    // 2. No trustworthy heading: push straight with the propeller until the GPS
    //    course is confirmed. Never steer on a heading we do not have.
    if (!heading.valid) {
        if (dist <= target.radiusM) {             // arrival only needs the position
            acquiring_   = false;
            rudderAngle_ = 0.0f;
            navMode_     = "reached";
            navMessage_  = "";
            return cmd;
        }
        if (!acquiring_) {
            acquiring_      = true;
            acquireStartMs_ = nowMs;
        }
        rudderAngle_ = 0.0f;
        if ((uint32_t)(nowMs - acquireStartMs_) > Calibration::HEADING_ACQUIRE_TIMEOUT_MS) {
            navMode_    = "heading-timeout";
            navMessage_ = "no GPS heading after propeller push: neutral";
            return cmd;
        }
        navMode_    = "acquire-heading";
        navMessage_ = "no GPS heading: straight propeller push, rudder centred";
        cmd.sailUs  = sailToUs(sailAngle_ != 0.0f ? sailAngle_ : (float)NAV_SAIL_RIGHT_DEG);
        cmd.rotorUs = Calibration::ROTOR_CENTER_US;
        cmd.esc1Us  = Calibration::AUTO_ESC_CRUISE_US;
        return cmd;
    }
    acquiring_ = false;

    // 3. Heading valid: navigation steers on the gated heading.
    NavResult r = nav_handleNavigationWithState(
        state_,
        (double)heading.deg,     // boat heading (gated GPS course)
        (double)bearing,
        (double)dist,
        (double)windDeg,
        sailAngle_,              // persisted navigation state (not reconstructed from µs)
        rudderAngle_,
        (double)target.radiusM,
        pos.lat, pos.lon,
        target.lat, target.lon,
        NAV_DEFAULT_CORRIDOR_HALF_WIDTH_M,
        dtS                       // real control period: feeds the auto-trim and the watchdog
    );

    navMode_    = r.mode       ? r.mode       : "?";
    navMessage_ = r.logMessage ? r.logMessage : "";

    if (r.waypointReached) {
        navMode_     = "reached";
        rudderAngle_ = 0.0f;     // recenter for the next leg
        return cmd;
    }

    // Persist the algorithm's angle state for the next tick.
    sailAngle_   = r.sailAngle;
    rudderAngle_ = r.rudderAngle;

    cmd.sailUs  = sailToUs(sailAngle_);
    cmd.rotorUs = rudderToUs(rudderAngle_);
    cmd.esc1Us = computeAutoPropulsionUs(
        pos,
        heading,
        dist,
        bearing,
        target.radiusM
    );
    return cmd;
}

void AutoController::reset() {
    nav_resetState(state_);
    gate_.reset();
    rudderAngle_ = 0.0f;
    sailAngle_   = 0.0f;
    acquiring_   = false;
}

void AutoController::beginWindObservation() {
    windObsComplete_    = false;
    windObsFailed_      = false;
    obsStarted_         = false;
    obsTimerStarted_    = false;
    obsSeeded_          = false;
    obsSamples_         = 0;
    windObsProgressPct_ = 0;
    observedWindDeg_    = 0.0f;
    smoothHeading_      = 0.0f;
    navMode_            = "wind-observation";
    navMessage_         = "waiting for GPS fix...";
}

ActuatorCommand AutoController::observeWind(const GpsPosition& pos, uint32_t nowMs) {
    ActuatorCommand cmd{};
    // Fixed tack to make the boat sail; rudder centered so it runs free.
    // No propeller: the course must come from the wind, or the estimate is biased.
    cmd.sailUs  = Calibration::SAIL_PLUS_US;
    cmd.rotorUs = Calibration::ROTOR_CENTER_US;

    if (windObsComplete_ || windObsFailed_) return cmd;

    if (!obsTimerStarted_) {
        obsTimerStarted_ = true;
        obsStartMs_      = nowMs;
        lastObsSeq_      = pos.courseSeq;   // only samples from now on count
    }
    if ((uint32_t)(nowMs - obsStartMs_) > Calibration::WIND_OBS_TIMEOUT_MS) {
        windObsFailed_ = true;
        navMode_       = "wind-failed";
        navMessage_    = "wind observation timed out (no fix or too slow)";
        return cmd;
    }

    if (!pos.valid) {
        navMessage_ = "no GPS fix";
        return cmd;
    }

    // Anchor the maneuver on the first valid fix.
    if (!obsStarted_) {
        obsStartLat_ = pos.lat;
        obsStartLon_ = pos.lon;
        obsStarted_  = true;
    }

    // Circular EMA of the GPS course, fed ONLY with new valid samples (1 Hz),
    // seeded with the first real sample instead of 0.
    if (pos.courseValid && pos.courseSeq != lastObsSeq_) {
        lastObsSeq_ = pos.courseSeq;
        if (!obsSeeded_) {
            smoothHeading_ = pos.courseDeg;
            obsSeeded_     = true;
        } else {
            float d = (float)nav_relativeAngle(smoothHeading_, pos.courseDeg);
            smoothHeading_ = (float)nav_normalizeAngle(
                smoothHeading_ + Calibration::WIND_OBS_SMOOTH_ALPHA * d);
        }
        if (obsSamples_ < 255) obsSamples_++;
    }

    double dist = Navigator::distanceM(obsStartLat_, obsStartLon_, pos.lat, pos.lon);

    // Progression 0–100 % : the slower of distance and course samples.
    double pctDist = dist / (double)Calibration::WIND_OBS_DISTANCE_M * 100.0;
    double pctSmp  = (double)obsSamples_ / (double)Calibration::WIND_OBS_MIN_SAMPLES * 100.0;
    double pct = pctDist < pctSmp ? pctDist : pctSmp;
    if (pct < 0.0)   pct = 0.0;
    if (pct > 100.0) pct = 100.0;
    windObsProgressPct_ = (uint8_t)pct;

    if (obsSamples_ < Calibration::WIND_OBS_MIN_SAMPLES) {
        navMessage_ = "observing wind: waiting for GPS course samples...";
        return cmd;
    }

    NavResult r = nav_handleWindObservation(
        pos.lat, pos.lon, obsStartLat_, obsStartLon_,
        (double)smoothHeading_, dist, (double)Calibration::WIND_OBS_DISTANCE_M,
        (float)NAV_SAIL_RIGHT_DEG, 0.0f);   // observing on the +10° (starboard) tack

    navMessage_ = r.logMessage ? r.logMessage : "observing wind...";

    if (r.windAcquired) {
        observedWindDeg_ = (float)r.acquiredWindDir;
        windObsComplete_ = true;
        navMode_         = "wind-acquired";
    }
    return cmd;
}

static float wrap180(float angleDeg) {
    while (angleDeg > 180.0f) angleDeg -= 360.0f;
    while (angleDeg < -180.0f) angleDeg += 360.0f;
    return angleDeg;
}

static uint16_t clampEscUs(int32_t us) {
    if (us < Calibration::ESC_STOP_US) return Calibration::ESC_STOP_US;
    if (us > Calibration::ESC_MAX_US)  return Calibration::ESC_MAX_US;
    return (uint16_t)us;
}


uint16_t AutoController::computeAutoPropulsionUs(const GpsPosition& pos,
                                                 const HeadingGate::State& heading,
                                                 float distM,
                                                 float bearingDeg,
                                                 float waypointRadiusM)
{
    if (!pos.valid) {
        return Calibration::ESC_STOP_US;
    }

    if (distM <= waypointRadiusM + Calibration::AUTO_PROP_STOP_RADIUS_M) {
        return Calibration::ESC_STOP_US;
    }

    // No speed field from the receiver means no measured motion: never reuse a stale speed.
    const float speedKmph = pos.speedValid ? pos.speedKmph : 0.0f;

    if (speedKmph >= Calibration::AUTO_PROP_TARGET_SPEED_KMPH) {
        return Calibration::ESC_STOP_US;
    }

    if (heading.valid) {
        float headingError = wrap180(bearingDeg - heading.deg);
        if (speedKmph > 0.5f && std::fabs(headingError) > Calibration::AUTO_PROP_HEADING_MAX_DEG) {
            return Calibration::AUTO_ESC_MIN_US;
        }
    }

    if (speedKmph < Calibration::AUTO_PROP_MIN_SPEED_KMPH) {
        return Calibration::AUTO_ESC_CRUISE_US;
    }

    float speedError = Calibration::AUTO_PROP_TARGET_SPEED_KMPH - speedKmph;
    int32_t escUs = static_cast<int32_t>(
        Calibration::ESC_STOP_US + speedError * 180.0f
    );

    if (escUs < Calibration::AUTO_ESC_MIN_US) {
        escUs = Calibration::AUTO_ESC_MIN_US;
    }

    return clampEscUs(escUs);
}
