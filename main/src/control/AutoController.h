#pragma once
#include "../core/Types.h"
#include "../navigation/NavigationSelector.h"
#include "HeadingGate.h"

class AutoController {
public:
    // --- Autonomous sailing toward a waypoint ---
    // Returns actuator commands for the current navigation step.
    // The controller keeps the navigation algorithm's rudder/sail angle as
    // internal state (the lofer/abattre integrator depends on it), so the
    // caller no longer needs to feed back the previous µs outputs.
    // Call reset() when leaving Automatic mode or when navigation is paused.
    //
    // GPS-only heading (no compass), so the command depends on data validity:
    //   fix invalid      → neutral ("gps-lost"), resumes by itself when the fix returns
    //   no heading yet   → straight-line propeller push, rudder centred, tack kept
    //                      ("acquire-heading"); neutral after HEADING_ACQUIRE_TIMEOUT_MS
    //                      ("heading-timeout")
    //   heading valid    → navigation.h steers on the gated heading
    ActuatorCommand compute(float windDeg,
                            const GpsPosition& pos,
                            const Waypoint& target,
                            uint32_t nowMs);

    static uint16_t computeAutoPropulsionUs(const GpsPosition& pos,
                                            const HeadingGate::State& heading,
                                            float distM,
                                            float bearingDeg,
                                            float waypointRadiusM);
    void reset();

    // --- Wind estimation from GPS track (Phase 5) ---
    // beginWindObservation() arms the maneuver; observeWind() must then be
    // called every control tick. While observing, the boat sails on a fixed
    // tack with the rudder centered (no propeller: it would bias the estimate).
    // The course EMA only takes NEW valid course samples. Once the boat has
    // travelled WIND_OBS_DISTANCE_M with at least WIND_OBS_MIN_SAMPLES samples,
    // the wind is latched and windObsComplete() returns true. After
    // WIND_OBS_TIMEOUT_MS without success, windObsFailed() returns true.
    void beginWindObservation();
    ActuatorCommand observeWind(const GpsPosition& pos, uint32_t nowMs);
    bool    windObsComplete()    const { return windObsComplete_; }
    bool    windObsFailed()      const { return windObsFailed_; }
    float   observedWindDeg()    const { return observedWindDeg_; }
    // Progression de la mesure de vent en % (0–100) : distance ET échantillons de cap.
    uint8_t windObsProgressPct() const { return windObsProgressPct_; }

    const HeadingGate::State& heading() const { return gate_.state(); }
    const char* navMode()    const { return navMode_; }
    const char* navMessage() const { return navMessage_; }

private:
    static uint16_t sailToUs(float sailAngleDeg);
    static uint16_t rudderToUs(float rudderAngleDeg);

    NavState    state_      = {};
    HeadingGate gate_;

    // Navigation algorithm state (persisted between ticks instead of being
    // reconstructed from the clamped µs outputs).
    float       rudderAngle_ = 0.0f;
    float       sailAngle_   = 0.0f;

    // Heading acquisition (propeller push while no valid heading)
    bool        acquiring_       = false;
    uint32_t    acquireStartMs_  = 0;
    uint32_t    lastComputeMs_   = 0;   // for the navigation time step (auto-trim, watchdog)

    // Wind observation state
    bool        windObsComplete_    = false;
    bool        windObsFailed_      = false;
    bool        obsStarted_         = false;
    bool        obsTimerStarted_    = false;
    bool        obsSeeded_          = false;
    uint8_t     windObsProgressPct_ = 0;
    uint8_t     obsSamples_         = 0;
    uint32_t    lastObsSeq_         = 0;
    uint32_t    obsStartMs_         = 0;
    float       observedWindDeg_    = 0.0f;
    double      obsStartLat_     = 0.0;
    double      obsStartLon_     = 0.0;
    float       smoothHeading_   = 0.0f;

    const char* navMode_    = "idle";
    const char* navMessage_ = "";
};
