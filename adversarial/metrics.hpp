#pragma once
// Falsifiable metrics for one autonomous run — replaces the non-discriminating
// binary "reached the waypoint". A boat that spirals in slowly, or reaches only
// by luck, must be distinguishable from clean convergence.
#include "boat_model.hpp"

struct TrialResult {
    bool   converged   = false;  // reached radius before timeout
    double timeS       = 0.0;    // time to converge (or timeout)
    double pathRatio   = 0.0;    // actual path / straight-line (1 = perfect, >>1 = wandering)
    double maxCrossTrackM = 0.0; // worst excursion from the start→wpt line
    int    tacks       = 0;      // sail-side changes
    double closestM    = 0.0;    // closest approach (diagnoses "never gets near")
    bool   stuck       = false;  // made no net progress over a long window (limit cycle)
};

// Run one trial to convergence or timeout. dt=0.1 s, matches the twin's 100 ms step.
inline TrialResult runTrial(const BoatParams& p,
                            double startLat, double startLng, double heading,
                            double wptLat, double wptLng,
                            unsigned seed, double timeoutS = 4000.0) {
    AdversarialBoat boat(p, seed);
    boat.init(startLat, startLng, heading, wptLat, wptLng);

    const double dt = 0.1;
    double t = 0.0;
    double lastDist = boat.distToWpt();
    double progressWindowStart = 0.0, distAtWindowStart = lastDist;

    TrialResult r;
    while (t < timeoutS) {
        if (!boat.step(dt)) { r.converged = true; break; }
        t += dt;

        // stuck detection: no net closing over a 300 s window
        if (t - progressWindowStart >= 300.0) {
            double now = boat.distToWpt();
            if (distAtWindowStart - now < 5.0) { r.stuck = true; break; }
            progressWindowStart = t; distAtWindowStart = now;
        }
    }

    r.timeS          = t;
    double straight  = boat.straightLen();
    r.pathRatio      = straight > 1.0 ? boat.pathLen() / straight : 0.0;
    r.maxCrossTrackM = boat.maxCrossTrack();
    r.tacks          = boat.tacks();
    r.closestM       = boat.closest();
    if (!r.converged && r.timeS >= timeoutS) r.stuck = r.stuck || (r.closestM > p.waypointRadiusM);
    return r;
}
