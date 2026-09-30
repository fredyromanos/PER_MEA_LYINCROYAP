#include "sim_boat.hpp"
#include "mocks/Arduino.h"

// ═══════════════════════════════════════════════════════════════
// INCLUSION DE LA LOGIQUE DE NAVIGATION RÉELLE (boat/navigation.h)
// C'est le MÊME code que celui qui tourne sur l'ESP32.
// Modifier navigation.h = modifier le comportement réel ET simulé.
// ═══════════════════════════════════════════════════════════════
#include "navigation.h"
#include "config/Calibration.h"

#include <cstdlib>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <cstring>
#include <algorithm>

SimulatedBoat::SimulatedBoat()
    : idealGps(false), lastGpsMs(0), rng(12345u), stepsNoHeading(0), stepsNav(0),
      currentWaypointId(-1), boatMode("standby"), windDirection(0),
      initialWindDirection(0), sailAngle(-10), rudderAngle(20),
      windObsStartLat(0), windObsStartLng(0) {
    const char* ideal = std::getenv("SIM_IDEAL_GPS");
    idealGps = (ideal && ideal[0] == '1');
}

// Uniform noise in [-amplitude, +amplitude], deterministic (same run = same result).
double SimulatedBoat::noise(double amplitude) {
    rng = rng * 1103515245u + 12345u;
    return (((rng >> 8) % 20001) / 10000.0 - 1.0) * amplitude;
}

// NEO-6M as the firmware driver (GpsUart) publishes it, sampled at 1 Hz:
//   - position with ±SIM_GPS_POS_NOISE_M noise
//   - course only when speed >= GPS_COURSE_MIN_SPEED_KMPH (receiver blanks it below),
//     with ±SIM_GPS_CRS_NOISE_DEG noise; courseSeq advances on each valid sample
// Between samples the published data does not change (same as the 50 Hz firmware loop).
void SimulatedBoat::updateGpsModel(const SimBoatState& s) {
    if (lastGpsMs != 0 && s.time - lastGpsMs < SIM_GPS_PERIOD_MS) return;
    lastGpsMs = (s.time == 0) ? 1 : s.time;

    // Optional periodic fix loss (IHM "gps-dropout" scenario): no fix at the END of each period.
    if (dropoutPeriodMs > 0 && (s.time % dropoutPeriodMs) >= dropoutPeriodMs - dropoutDurationMs) {
        gps.valid       = false;          // what GpsUart publishes once the fix is stale
        gps.courseValid = false;
        gps.speedValid  = false;
        gps.satellites  = 0;
        gps.hdop        = 99.9f;
        gps.ageMs       = (uint32_t)(s.time - lastValidFixMs);
        return;                           // lat/lon keep the last known position
    }
    lastValidFixMs = s.time;

    const double northM = noise(SIM_GPS_POS_NOISE_M);
    const double eastM  = noise(SIM_GPS_POS_NOISE_M);
    gps.valid       = true;
    gps.satellites  = 8;
    gps.hdop        = 1.2f;
    gps.ageMs       = 0;
    gps.lat         = s.latitude + northM / 111320.0;
    gps.lon         = s.longitude + eastM / (111320.0 * std::cos(s.latitude * M_PI / 180.0));
    // A real GPS receiver measures velocity OVER THE GROUND (Doppler / position delta),
    // not the boat's speed through the water — so the gate and the reported course must
    // come from speedOverGround/courseOverGround, not heading/speed. This is exactly the
    // case current is meant to exercise: under strong current a boat nearly stationary
    // through the water can still be moving (and turning course-valid) over the ground.
    // With no current/leeway (defaults), speedOverGround==speed and courseOverGround==
    // heading exactly, so this is bit-for-bit identical to the old heading/speed-based code.
    gps.speedKmph   = (float)(s.speedOverGround * 3.6);
    gps.speedValid  = true;
    gps.courseValid = gps.speedKmph >= Calibration::GPS_COURSE_MIN_SPEED_KMPH;
    if (gps.courseValid) {
        gps.courseDeg = (float)nav_normalizeAngle(s.courseOverGround + noise(SIM_GPS_CRS_NOISE_DEG));
        gps.courseSeq++;
    }
}

void SimulatedBoat::init(double startLat, double startLng, double windDir, double windSpeed, double initialHeading) {
    environment.init(startLat, startLng, windDir, windSpeed, initialHeading);
    boatMode = "setup-ready";
    windDirection = windDir;
    initialWindDirection = windDir;
    
    // Placer l'aileron du bon côté selon le vent relatif
    // relativeWind > 0 → vent de tribord → aileron +10 (voile à bâbord)
    // relativeWind < 0 → vent de bâbord → aileron -10 (voile à tribord)
    double relWind = windDir - initialHeading;
    while (relWind > 180) relWind -= 360;
    while (relWind < -180) relWind += 360;
    sailAngle = (relWind >= 0) ? 10 : -10;
    rudderAngle = 0;  // pas de déphasage au départ
    
    windObsStartLat = startLat;
    windObsStartLng = startLng;
    
    environment.setServoAngles(sailAngle, rudderAngle);
    
    std::cout << "\n======================================================" << std::endl;
    std::cout << "  AutoBoat Simulation  (REAL NAVIGATION CODE)" << std::endl;
    std::cout << "  Logic from: boat/navigation.h" << std::endl;
    std::cout << "======================================================\n" << std::endl;
}

void SimulatedBoat::addWaypoint(double lat, double lng) {
    SimWaypoint wpt = {lat, lng};
    waypoints.push_back(wpt);
    environment.addWaypoint(lat, lng);
    std::cout << "[BOAT] Waypoint " << (waypoints.size()-1) << ": " 
              << lat << ", " << lng << std::endl;
}

void SimulatedBoat::startWindObservation() {
    if (boatMode == "setup-ready") {
        boatMode = "wind-observation";
        // Comme dans boat.ino : on mémorise la position de départ
        const SimBoatState& s = environment.getState();
        windObsStartLat = s.latitude;
        windObsStartLng = s.longitude;
        // Placer l'aileron du bon côté selon le vent relatif au cap actuel
        double relWind = windDirection - s.heading;
        while (relWind > 180) relWind -= 360;
        while (relWind < -180) relWind += 360;
        sailAngle = (relWind >= 0) ? 10 : -10;
        rudderAngle = 0;  // pas de déphasage → le bateau va droit
        environment.setServoAngles(sailAngle, rudderAngle);
        std::cout << "[BOAT] Starting wind observation (REAL CODE)..."
                  << " aileron=" << sailAngle << std::endl;
    }
}

void SimulatedBoat::startNavigation() {
    if ((boatMode == "wind-ready" || boatMode == "wind-observation") && !waypoints.empty()) {
        currentWaypointId = 0;
        boatMode = "navigate";
        // Réinitialiser le déphasage safran pour la navigation
        rudderAngle = 0;
        gate.reset();   // comme AutoController::reset() au démarrage de mission
        navState_ = NavState{};   // S1: état nav propre à ce bateau, pas de fuite inter-scénario
        std::cout << "[BOAT] GPS model: " << (idealGps ? "IDEAL (exact heading)"
                                                       : "REALISTIC (1 Hz, noise, no course when slow)")
                  << std::endl;
        std::cout << "[BOAT] Navigation started (REAL CODE), heading to waypoint 0" << std::endl;
    } else {
        std::cout << "[BOAT] ERROR: Cannot navigate (mode=" << boatMode 
                  << ", waypoints=" << waypoints.size() << ")" << std::endl;
    }
}

void SimulatedBoat::stopNavigation() {
    boatMode = "standby";
    std::cout << "[BOAT] Navigation stopped" << std::endl;
}

// ════════════════════════════════════════════════════════════════
// updateNavigationLogic() — Appelle le VRAI code de navigation
// depuis boat/navigation.h (identique au code ESP32)
// ════════════════════════════════════════════════════════════════
void SimulatedBoat::updateNavigationLogic() {
    const SimBoatState& state = environment.getState();
    
    if (boatMode == "wind-observation") {
        // ──── OBSERVATION DU VENT (code réel via navigation.h) ────
        environment.setNavMode(0);
        
        // Calcul de la distance depuis la position de départ
        // (simule gpsBoat.computeDirectPath(currentWptLat, currentWptLng) + getDist())
        double distToStart, headingToStart;
        environment.computeDistanceToWaypoint(windObsStartLat, windObsStartLng, distToStart, headingToStart);
        
        // Appel de la VRAIE fonction d'observation du vent
        NavResult r = nav_handleWindObservation(
            state.latitude, state.longitude,
            windObsStartLat, windObsStartLng,
            state.heading,   // smoothHeading en simulation = heading exact
            distToStart,
            WIND_DISTANCE_SIM,
            sailAngle,
            rudderAngle
        );
        
        // Appliquer les angles
        sailAngle = r.sailAngle;
        rudderAngle = r.rudderAngle;
        float clampedSail = std::max(-10.0f, std::min(10.0f, sailAngle));
        float clampedRudder = std::max(-45.0f, std::min(45.0f, rudderAngle));
        environment.setServoAngles(clampedSail, clampedRudder);
        
        if (r.windAcquired) {
            boatMode = "wind-ready";
            windDirection = r.acquiredWindDir;
            std::cout << "[BOAT] Wind acquired (REAL CODE): " << windDirection 
                      << " deg" << std::endl;
        }
    } 
    else if (boatMode == "navigate" && !waypoints.empty()) {
        // ──── NAVIGATION AUTONOME (code réel via navigation.h) ────
        SimWaypoint& wpt = waypoints[currentWaypointId];

        // What the boat KNOWS: exact state (ideal) or the realistic GPS model + real HeadingGate.
        double boatLat = state.latitude, boatLon = state.longitude, boatHeading = state.heading;
        if (!idealGps) {
            updateGpsModel(state);
            const HeadingGate::State& h = gate.update(gps, (uint32_t)state.time);
            boatLat = gps.lat;
            boatLon = gps.lon;
            stepsNav++;
            if (!gps.valid) {
                // Same rule as AutoController::compute: fix lost → neutral (sail centre,
                // rudder centre, propeller stopped); resumes by itself when the fix returns.
                acquiring = false;
                rudderAngle = 0;
                environment.setPropellerSpeed(0.0);
                environment.setServoAngles(0.0f, 0.0f);
                environment.setNavMode(5);
                navStateLabel = "gps-lost";
                return;
            }
            if (!h.valid) {
                // Same rule as AutoController::compute: never steer on a heading we do not
                // have. Rudder centred, tack kept, straight propeller push until the GPS
                // course is confirmed; propeller off after HEADING_ACQUIRE_TIMEOUT_MS.
                stepsNoHeading++;
                if (!acquiring) { acquiring = true; acquireStartMs = state.time; }
                const bool timedOut =
                    state.time - acquireStartMs > Calibration::HEADING_ACQUIRE_TIMEOUT_MS;
                environment.setPropellerSpeed(timedOut ? 0.0 : SIM_PROP_CRUISE_MS);
                navStateLabel = timedOut ? "heading-timeout" : "acquire-heading";
                rudderAngle = 0;
                environment.setServoAngles(std::max(-10.0f, std::min(10.0f, sailAngle)), 0.0f);
                environment.setNavMode(5);
                if (state.time % 10000 < 100) {
                    std::cout << "[NAV-REAL] " << (timedOut ? "heading-timeout" : "acquire-heading")
                              << ": no GPS heading (speed " << gps.speedKmph
                              << " km/h), rudder centred" << std::endl;
                }
                return;
            }
            acquiring = false;
            environment.setPropellerSpeed(0.0);   // sail only once the heading is known
            navStateLabel = "navigate";
            boatHeading = h.deg;
        }

        // Simuler gpsBoat.computeDirectPath(currentWptLat, currentWptLng)
        double distance, wptHeading;
        if (idealGps) {
            environment.computeDistanceToWaypoint(wpt.lat, wpt.lng, distance, wptHeading);
        } else {
            // Same geometry as the ideal path, from the position the boat BELIEVES it is at.
            SimulationEnvironment::distanceAndBearing(boatLat, boatLon, wpt.lat, wpt.lng,
                                                      distance, wptHeading);
        }

        // Appel de la VRAIE fonction de navigation, avec l'état nav PROPRE à ce
        // bateau (S1) — nav_handleNavigation() utiliserait un `static NavState`
        // partagé par tout le process, ce qui ferait fuiter corridor/gybe/trim
        // d'un scénario à l'autre quand les 6 scénarios tournent dans le même process.
        NavResult r = nav_handleNavigationWithState(
            navState_,
            boatHeading,        // cap connu du bateau (gate GPS, ou exact en mode idéal)
            wptHeading,         // cap vers WPT
            distance,           // distance au WPT
            windDirection,      // variable globale
            sailAngle,          // angle courant (peut être accumulé)
            rudderAngle,        // angle courant (peut être accumulé)
            WAYPOINT_DISTANCE_SIM,
            boatLat,
            boatLon,
            wpt.lat,
            wpt.lng,
            NAV_DEFAULT_CORRIDOR_HALF_WIDTH_M
        );
        
        // Stocker les résultats bruts (comme les variables globales du vrai bateau)
        sailAngle = r.sailAngle;
        rudderAngle = r.rudderAngle;   // raw command — fed back to nav next tick (unchanged)

        // ── Faithful actuator model (reconciled with firmware) ───────────────
        // The nav rudder command carries a standing wind-compensation feed-forward
        // (nav_rudderCompensation = relWind/2): the rudder offset that cancels the
        // sail's yaw moment so the boat holds course. The real winch applies it as
        // a position; on the water the sail yaw and that offset cancel → straight.
        // The sim's kinematics do NOT model sail yaw, so driving the physics from
        // the raw command would spin the boat. Instead we drive the turn from the
        // pure STEERING CORRECTION the firmware itself extracts — nav_rudderCorrection
        // (clamped to ±NAV_RUDDER_CORRECTION_LIMIT_DEG = 20°) — treating the
        // compensation term as course-holding, not steering.
        double relWind = nav_relativeAngle(state.heading, windDirection);
        // Steering = command − wind feed-forward. Identical to nav_rudderCorrection()
        // while the auto-trim is 0 (the command is already comp + clamped correction),
        // but it does NOT re-clamp to ±20°, so the trim can actually steer the boat —
        // same convention as the adversarial model (actualRudder − sailYawEquiv).
        float steerDeg = (float)rudderAngle - nav_rudderCompensation(relWind);
        if (steerDeg > NAV_RUDDER_COMMAND_LIMIT_DEG) steerDeg = NAV_RUDDER_COMMAND_LIMIT_DEG;
        if (steerDeg < -NAV_RUDDER_COMMAND_LIMIT_DEG) steerDeg = -NAV_RUDDER_COMMAND_LIMIT_DEG;
        float clampedSail = std::max(-10.0f, std::min(10.0f, sailAngle));
        environment.setServoAngles(clampedSail, steerDeg);
        
        // Mapper le mode de navigation réel → navMode pour export HTML
        if (strcmp(r.mode, "vdb") == 0) {
            environment.setNavMode(1);
        } else if (strcmp(r.mode, "upwind-zigzag") == 0) {
            environment.setNavMode(6);
        } else if (strcmp(r.mode, "downwind-zigzag") == 0) {
            environment.setNavMode(7);
        } else if (strcmp(r.mode, "avoid-gybe") == 0) {
            environment.setNavMode(8);
        } else if (strcmp(r.mode, "lofer") == 0) {
            environment.setNavMode(3);
        } else if (strcmp(r.mode, "abattre") == 0) {
            environment.setNavMode(4);
        } else {
            environment.setNavMode(2); // direct
        }
        
        // Log navigation toutes les 10s (simule jsonMessage du code réel)
        if (r.logMessage && state.time % 10000 < 100) {
            std::cout << "[NAV-REAL] " << r.logMessage
                      << " | dist=" << (int)distance << "m"
                      << " | sail=" << clampedSail
                      << " | steer=" << steerDeg
                      << " (cmd=" << r.rudderAngle << ")" << std::endl;
        }
        
        // Waypoint atteint → passer au suivant ou terminer
        if (r.waypointReached) {
            if (currentWaypointId < (int)waypoints.size() - 1) {
                currentWaypointId++;
                rudderAngle = 0;  // reset pour le nouveau WPT
                std::cout << "[NAV-REAL] Waypoint " << (currentWaypointId-1) 
                          << " reached! Moving to waypoint " << currentWaypointId << std::endl;
            } else {
                std::cout << "[NAV-REAL] All waypoints reached!" << std::endl;
                boatMode = "standby";
            }
        }
    }
}

void SimulatedBoat::applyServoOutput() {
    // Les servos sont déjà appliqués via updateNavigationLogic
}

void SimulatedBoat::stepSimulation(unsigned long dt_ms) {
    // The receiver runs whatever the mode (telemetry needs a position before "navigate").
    if (!idealGps) updateGpsModel(environment.getState());
    if (boatMode != "navigate") navStateLabel = "idle";
    updateNavigationLogic();
    environment.update(dt_ms);
}

void SimulatedBoat::runSimulation(unsigned long duration_ms, unsigned long timeStep_ms) {
    unsigned long elapsedTime = 0;
    int stepCount = 0;

    // Stuck detector — mirrors adversarial/metrics.hpp::runTrial(): no net closing of
    // >5 m on the current waypoint over a 300 s window ⇒ stuck (limit cycle). Same
    // window (300 s) and threshold (5 m) as the adversarial harness, so the two tools
    // agree on what "stuck" means. Only armed while actively navigating, so it cannot
    // fire during wind-observation/idle phases.
    const unsigned long STUCK_WINDOW_MS  = 300000;  // 300 s, same as adversarial/metrics.hpp
    const double        STUCK_PROGRESS_M = 5.0;     // same as adversarial/metrics.hpp
    unsigned long windowStartMs = 0;
    double        distAtWindowStart = -1.0;
    int           windowWptId = -1;
    bool          stuckThisRun = false;

    while (elapsedTime < duration_ms) {
        stepSimulation(timeStep_ms);
        elapsedTime += timeStep_ms;
        stepCount++;

        // Afficher le statut toutes les 10 secondes simulées
        if (stepCount % (10000 / timeStep_ms) == 0) {
            printStatus();
        }

        if (boatMode == "navigate" && !waypoints.empty() && currentWaypointId >= 0) {
            if (currentWaypointId != windowWptId) {
                // Advancing to a new waypoint is itself progress — restart the window.
                windowWptId = currentWaypointId;
                windowStartMs = elapsedTime;
                distAtWindowStart = -1.0;
            }
            if (distAtWindowStart < 0.0 || elapsedTime - windowStartMs >= STUCK_WINDOW_MS) {
                double d, h;
                const SimWaypoint& wpt = waypoints[currentWaypointId];
                environment.computeDistanceToWaypoint(wpt.lat, wpt.lng, d, h);
                if (distAtWindowStart < 0.0) {
                    distAtWindowStart = d;
                    windowStartMs = elapsedTime;
                } else if (distAtWindowStart - d < STUCK_PROGRESS_M) {
                    stuckThisRun = true;
                    break;
                } else {
                    distAtWindowStart = d;
                    windowStartMs = elapsedTime;
                }
            }
        }

        // Arrêter tôt si tous les waypoints sont atteints
        if (boatMode == "standby" && currentWaypointId >= 0) {
            break;
        }
    }

    // Record the outcome of this call only if it was actually a navigation phase (a
    // wind-observation-phase call that simply runs out its fixed duration leaves the
    // outcome from the real navigation call untouched).
    if (boatMode == "standby" && currentWaypointId >= 0) {
        outcome_ = RunOutcome::Converged;
        convergeTimeMs_ = elapsedTime;
    } else if (stuckThisRun) {
        outcome_ = RunOutcome::Stuck;
        convergeTimeMs_ = elapsedTime;
        std::cout << "[SIM] STUCK: no >" << STUCK_PROGRESS_M << "m progress on waypoint "
                  << currentWaypointId << " in " << (STUCK_WINDOW_MS / 1000) << "s" << std::endl;
    } else if (boatMode == "navigate") {
        outcome_ = RunOutcome::TimedOut;
        convergeTimeMs_ = elapsedTime;
    }

    std::cout << "\n[SIM] Simulation finished!" << std::endl;
    printStatus();
}

void SimulatedBoat::printStatus() const {
    const SimBoatState& state = environment.getState();
    
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "[" << std::setw(6) << (state.time / 1000) << "s] "
              << "Pos: " << state.latitude << "°, " << state.longitude << "° | "
              << "Heading: " << std::setw(6) << std::setprecision(1) << state.heading << "° | "
              << "Speed: " << std::setw(4) << std::setprecision(2) << state.speed << "m/s | "
              << "Mode: " << boatMode << " | "
              << "Sail: " << std::setw(6) << std::setprecision(1) << state.sailAngle << "° | "
              << "Rudder: " << std::setw(6) << state.rudderAngle << "°" << std::endl;
    
    if (!waypoints.empty() && currentWaypointId >= 0 && currentWaypointId < (int)waypoints.size()) {
        const SimWaypoint& wpt = waypoints[currentWaypointId];
        double distance, heading;
        environment.computeDistanceToWaypoint(wpt.lat, wpt.lng, distance, heading);
        std::cout << "         → Waypoint " << currentWaypointId << ": " 
                  << std::setprecision(0) << distance << "m away, bearing " 
                  << std::setprecision(1) << heading << "°" << std::endl;
    }
}
