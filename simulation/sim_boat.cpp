#include "sim_boat.hpp"
#include "mocks/Arduino.h"

// ═══════════════════════════════════════════════════════════════
// INCLUSION DE LA LOGIQUE DE NAVIGATION RÉELLE (boat/navigation.h)
// C'est le MÊME code que celui qui tourne sur l'ESP32.
// Modifier navigation.h = modifier le comportement réel ET simulé.
// ═══════════════════════════════════════════════════════════════
#include "../main/src/navigation/navigation.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <iostream>

namespace {
constexpr double SIM_EARTH_RADIUS_M = 6371000.0;

void computePathFromPosition(double fromLat, double fromLng, double toLat,
                             double toLng, double &distanceM,
                             double &headingDeg) {
  const double meanLatRad = (fromLat + toLat) * 0.5 * M_PI / 180.0;
  const double northM =
      (toLat - fromLat) * M_PI / 180.0 * SIM_EARTH_RADIUS_M;
  const double eastM = (toLng - fromLng) * M_PI / 180.0 *
                       SIM_EARTH_RADIUS_M * std::cos(meanLatRad);
  distanceM = std::hypot(eastM, northM);
  headingDeg = std::atan2(eastM, northM) * 180.0 / M_PI;
  if (headingDeg < 0.0)
    headingDeg += 360.0;
}
}

SimulatedBoat::SimulatedBoat()
    : currentWaypointId(-1), boatMode("standby"), windDirection(0),
      initialWindDirection(0), sailAngle(-10), rudderAngle(20),
      windObsStartLat(0), windObsStartLng(0) {
  nav_resetState(navState);
}

void SimulatedBoat::init(double startLat, double startLng, double windDir,
                         double windSpeed, double initialHeading) {
  environment.init(startLat, startLng, windDir, windSpeed, initialHeading);
  boatMode = "setup-ready";
  windDirection = windDir;
  initialWindDirection = windDir;

  // Placer l'aileron du bon côté selon le vent relatif
  // relativeWind > 0 → vent de tribord → aileron +10 (voile à bâbord)
  // relativeWind < 0 → vent de bâbord → aileron -10 (voile à tribord)
  double relWind = windDir - initialHeading;
  while (relWind > 180)
    relWind -= 360;
  while (relWind < -180)
    relWind += 360;
  sailAngle = (relWind >= 0) ? 10 : -10;
  rudderAngle = 0; // pas de déphasage au départ

  windObsStartLat = startLat;
  windObsStartLng = startLng;
  nav_resetState(navState);

  environment.setServoAngles(sailAngle, rudderAngle);

  std::cout << "\n======================================================"
            << std::endl;
  std::cout << "  AutoBoat Simulation  (REAL NAVIGATION CODE)" << std::endl;
  std::cout << "  Logic from: boat/navigation.h" << std::endl;
  std::cout << "======================================================\n"
            << std::endl;
}

void SimulatedBoat::addWaypoint(double lat, double lng) {
  Waypoint wpt = {lat, lng};
  waypoints.push_back(wpt);
  environment.addWaypoint(lat, lng);
  std::cout << "[BOAT] Waypoint " << (waypoints.size() - 1) << ": " << lat
            << ", " << lng << std::endl;
}

void SimulatedBoat::startWindObservation() {
  if (boatMode == "setup-ready") {
    boatMode = "wind-observation";
    // Comme dans boat.ino : on mémorise la position de départ
    const SimSensorState &sensor = environment.getSensorState();
    windObsStartLat = sensor.latitude;
    windObsStartLng = sensor.longitude;
    // La procédure suppose que le bateau est placé à +90° du vent.
    sailAngle = 10;
    rudderAngle = 45;
    environment.setServoAngles(sailAngle, rudderAngle);
    std::cout << "[BOAT] Starting wind observation (REAL CODE)..."
              << " aileron=" << sailAngle << std::endl;
  }
}

void SimulatedBoat::startNavigation() {
  if ((boatMode == "wind-ready" || boatMode == "wind-observation") &&
      !waypoints.empty()) {
    currentWaypointId = 0;
    boatMode = "navigate";
    const SimSensorState &sensor = environment.getSensorState();
    double relativeWind = windDirection - sensor.heading;
    while (relativeWind > 180)
      relativeWind -= 360;
    while (relativeWind < -180)
      relativeWind += 360;
    rudderAngle = static_cast<float>(relativeWind / 2.0);
    nav_resetState(navState);
    std::cout << "[BOAT] Navigation started (REAL CODE), heading to waypoint 0"
              << std::endl;
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
  const SimBoatState &state = environment.getState();
  const SimSensorState &sensor = environment.getSensorState();

  if (boatMode == "wind-observation") {
    // ──── OBSERVATION DU VENT (code réel via navigation.h) ────
    environment.setNavMode(0);

    // Calcul de la distance depuis la position de départ
    // (simule gpsBoat.computeDirectPath(currentWptLat, currentWptLng) +
    // getDist())
    double distToStart, headingToStart;
    computePathFromPosition(sensor.latitude, sensor.longitude, windObsStartLat,
                            windObsStartLng, distToStart, headingToStart);

    // Appel de la VRAIE fonction d'observation du vent
    NavResult r = nav_handleWindObservation(
        sensor.latitude, sensor.longitude, windObsStartLat, windObsStartLng,
        sensor.heading,
        distToStart, WIND_DISTANCE_SIM, sailAngle, rudderAngle);

    // Appliquer les angles
    sailAngle = r.sailAngle;
    rudderAngle = r.rudderAngle;
    float clampedSail = std::max(-10.0f, std::min(10.0f, sailAngle));
    float clampedRudder = std::max(-110.0f, std::min(110.0f, rudderAngle));
    environment.setServoAngles(clampedSail, clampedRudder);

    if (r.windAcquired) {
      boatMode = "wind-ready";
      windDirection = r.acquiredWindDir;
      double relativeWind = windDirection - sensor.heading;
      while (relativeWind > 180)
        relativeWind -= 360;
      while (relativeWind < -180)
        relativeWind += 360;
      rudderAngle = static_cast<float>(relativeWind / 2.0);
      environment.setServoAngles(sailAngle, rudderAngle);
      std::cout << "[BOAT] Wind acquired (REAL CODE): " << windDirection
                << " deg" << std::endl;
    }
  } else if (boatMode == "navigate" && !waypoints.empty()) {
    // ──── NAVIGATION AUTONOME (code réel via navigation.h) ────
    Waypoint &wpt = waypoints[currentWaypointId];

    // Simuler gpsBoat.computeDirectPath(currentWptLat, currentWptLng)
    double distance, wptHeading;
    computePathFromPosition(sensor.latitude, sensor.longitude, wpt.lat, wpt.lng,
                            distance, wptHeading);

    // Appel de la VRAIE fonction de navigation
    NavResult r = nav_handleNavigationWithState(
        navState,
        sensor.heading,
        wptHeading,    // gpsBoat.getHeading() (cap vers WPT)
        distance,      // gpsBoat.getDist()
        windDirection, // variable globale
        sailAngle,     // angle courant (peut être accumulé)
        rudderAngle,   // angle courant (peut être accumulé)
        WAYPOINT_DISTANCE_SIM,
        sensor.latitude,
        sensor.longitude,
        wpt.lat,
        wpt.lng,
        corridorWidth);

    // Stocker les résultats bruts (comme les variables globales du vrai bateau)
    sailAngle = r.sailAngle;
    rudderAngle = r.rudderAngle;

    // Appliquer les angles au moteur physique AVEC contrainte servo
    // (identique au comportement réel : servoControl.setRudderAngle() clamp
    // [-45,+45])
    float clampedSail = std::max(-10.0f, std::min(10.0f, sailAngle));
    float clampedRudder = std::max(-110.0f, std::min(110.0f, rudderAngle));
    environment.setServoAngles(clampedSail, clampedRudder);

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
      std::cout << "[NAV-REAL] " << r.logMessage << " | dist=" << (int)distance
                << "m"
                << " | sail=" << clampedSail << " | rudder=" << clampedRudder
                << " (raw=" << r.rudderAngle << ")" << std::endl;
    }

    // Waypoint atteint → passer au suivant ou terminer
    if (r.waypointReached) {
      if (currentWaypointId < (int)waypoints.size() - 1) {
        currentWaypointId++;
        double relativeWind = windDirection - sensor.heading;
        while (relativeWind > 180)
          relativeWind -= 360;
        while (relativeWind < -180)
          relativeWind += 360;
        rudderAngle = static_cast<float>(relativeWind / 2.0);
        std::cout << "[NAV-REAL] Waypoint " << (currentWaypointId - 1)
                  << " reached! Moving to waypoint " << currentWaypointId
                  << std::endl;
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
  updateNavigationLogic();
  environment.update(dt_ms);
}

void SimulatedBoat::runSimulation(unsigned long duration_ms,
                                  unsigned long timeStep_ms) {
  unsigned long elapsedTime = 0;
  int stepCount = 0;

  while (elapsedTime < duration_ms) {
    stepSimulation(timeStep_ms);
    elapsedTime += timeStep_ms;
    stepCount++;

    // Afficher le statut toutes les 10 secondes simulées
    if (stepCount % (10000 / timeStep_ms) == 0) {
      printStatus();
    }

    // Arrêter tôt si tous les waypoints sont atteints
    if (boatMode == "standby" && currentWaypointId >= 0) {
      break;
    }
  }

  std::cout << "\n[SIM] Simulation finished!" << std::endl;
  printStatus();
}

void SimulatedBoat::printStatus() const {
  const SimBoatState &state = environment.getState();

  std::cout << std::fixed << std::setprecision(6);
  std::cout << "[" << std::setw(6) << (state.time / 1000) << "s] "
            << "Pos: " << state.latitude << "°, " << state.longitude << "° | "
            << "Heading: " << std::setw(6) << std::setprecision(1)
            << state.heading << "° | "
            << "Speed: " << std::setw(4) << std::setprecision(2) << state.speed
            << "m/s | "
            << "Mode: " << boatMode << " | "
            << "Sail: " << std::setw(6) << std::setprecision(1)
            << state.sailAngle << "° | "
            << "Rudder: " << std::setw(6) << state.rudderAngle << "°"
            << std::endl;

  if (!waypoints.empty() && currentWaypointId >= 0 &&
      currentWaypointId < (int)waypoints.size()) {
    const Waypoint &wpt = waypoints[currentWaypointId];
    double distance, heading;
    environment.computeDistanceToWaypoint(wpt.lat, wpt.lng, distance, heading);
    std::cout << "         → Waypoint " << currentWaypointId << ": "
              << std::setprecision(0) << distance << "m away, bearing "
              << std::setprecision(1) << heading << "°" << std::endl;
  }
}
