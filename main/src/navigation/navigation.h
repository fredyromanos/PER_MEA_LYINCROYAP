/**
 * Logique de navigation partagee entre le firmware Arduino et la simulation.
 */

#ifndef NAVIGATION_H
#define NAVIGATION_H

#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static const double NAV_DIRECT_DEAD_ZONE_DEG = 5.0;
static const double NAV_VDB_RUDDER_GAIN = 0.8;
static const double NAV_DIRECT_RUDDER_GAIN = 0.5;
static const float NAV_RUDDER_CORRECTION_LIMIT_DEG = 20.0f;
static const float NAV_RUDDER_COMMAND_LIMIT_DEG = 110.0f;
// Half-width of the tacking / gybing corridor around the direct route. 2026-09-16: 100 → 30 m.
// With 100 m the boat tacked once, ~100 m outside the route (visually "not a square" on a
// 400 m mission); 30 m gives regular tacks ~37 m from the line for ~+7 % mission time.
// Comparison and KPIs: see the GPS work report (corridor_kpi).
static const double NAV_DEFAULT_CORRIDOR_HALF_WIDTH_M = 30.0;
static const double NAV_GEO_EPSILON_DEG = 0.0000001;
static const double NAV_EARTH_RADIUS_M = 6371000.0;
static const double NAV_UPWIND_FORBIDDEN_ANGLE_DEG = 45.0;
static const double NAV_DOWNWIND_FORBIDDEN_ANGLE_DEG = 20.0;
static const double NAV_RUDDER_STEP_DEG = 5.0;
static const double NAV_SAIL_RIGHT_DEG = 10.0;
static const double NAV_SAIL_LEFT_DEG = -10.0;

// ── Rudder auto-trim (2026-09-16) ────────────────────────────────────────────
// A constant rudder offset (winch not centred, mechanical play) used to make the boat
// hold a heading 10-25° off its target: the correction is clamped to ±20°, and the
// avoid-gybe loop only advances within 10° of its target → the boat sailed straight
// away forever. The trim is an integral term learnt from the heading error while a
// target heading is being tracked, applied OUTSIDE the ±20° correction clamp.
static const double NAV_TRIM_GAIN_PER_S = 0.02;      // deg of trim per deg of error per s
static const double NAV_TRIM_MAX_DEG = 40.0;
static const double NAV_TRIM_ERROR_WINDOW_DEG = 30.0; // learn only near the target (anti wind-up)

// ── Manoeuvre watchdog (2026-09-16) ──────────────────────────────────────────
// No state may block forever: an avoid-gybe phase advances after a timeout, and a lack
// of progress towards the waypoint resets the manoeuvre state (the trim is kept).
static const double NAV_WATCHDOG_PHASE_TIMEOUT_S = 120.0;
static const double NAV_WATCHDOG_NO_PROGRESS_S = 300.0;
static const double NAV_WATCHDOG_PROGRESS_M = 5.0;
static const double NAV_DEFAULT_DT_S = 0.1;           // call period when the caller gives none

// fmod-based wrap to [0, 360): O(1) for any finite input. The old while-loop
// version was linear in the input magnitude — a bogus huge value (e.g. a
// corrupted radio payload) could cost millions of iterations and stall the
// control loop for seconds on the ESP32. fmod handles NaN/Inf without looping.
inline double nav_normalizeAngle(double angleDeg) {
  double wrappedDeg = std::fmod(angleDeg, 360.0);
  if (wrappedDeg < 0.0)
    wrappedDeg += 360.0;
  return wrappedDeg;
}

// fmod-based wrap to (-180, 180]: O(1) for any finite input (the old code only
// applied a single +-360 correction, so it was only correct while
// |target-reference| < 540 — see navigation.h history). A single fmod already
// reduces to (-360, 360); one corrective +-360 then lands it in (-180, 180].
inline double nav_relativeAngle(double referenceDeg, double targetDeg) {
  double relativeDeg = std::fmod(targetDeg - referenceDeg, 360.0);
  if (relativeDeg <= -180.0)
    relativeDeg += 360.0;
  else if (relativeDeg > 180.0)
    relativeDeg -= 360.0;
  return relativeDeg;
}

inline bool nav_sameSign(double a, double b) {
  return (a >= 0 && b >= 0) || (a < 0 && b < 0);
}

inline double nav_oppositeAngle(double angleDeg) {
  return nav_normalizeAngle(angleDeg + 180.0);
}

inline int nav_angleSide(double angleDeg) { return angleDeg < 0 ? -1 : 1; }

inline float nav_clampRudder(float rudderAngleDeg) {
  if (rudderAngleDeg > NAV_RUDDER_CORRECTION_LIMIT_DEG)
    return NAV_RUDDER_CORRECTION_LIMIT_DEG;
  if (rudderAngleDeg < -NAV_RUDDER_CORRECTION_LIMIT_DEG)
    return -NAV_RUDDER_CORRECTION_LIMIT_DEG;
  return rudderAngleDeg;
}

inline float nav_rudderCompensation(double relativeWindDeg) {
  return static_cast<float>(relativeWindDeg / 2.0);
}

inline float nav_rudderCorrection(float servoCommandDeg,
                                  double relativeWindDeg) {
  return nav_clampRudder(servoCommandDeg -
                         nav_rudderCompensation(relativeWindDeg));
}

inline float nav_rudderCommand(float correctionDeg, double relativeWindDeg) {
  float commandDeg =
      nav_rudderCompensation(relativeWindDeg) + nav_clampRudder(correctionDeg);
  if (commandDeg > NAV_RUDDER_COMMAND_LIMIT_DEG)
    return NAV_RUDDER_COMMAND_LIMIT_DEG;
  if (commandDeg < -NAV_RUDDER_COMMAND_LIMIT_DEG)
    return -NAV_RUDDER_COMMAND_LIMIT_DEG;
  return commandDeg;
}

inline bool nav_isBetweenOnShortestTurn(double angleDeg, double startDeg,
                                        double endDeg) {
  double turnDeg = nav_relativeAngle(startDeg, endDeg);
  double angleFromStartDeg = nav_relativeAngle(startDeg, angleDeg);
  if (turnDeg >= 0)
    return angleFromStartDeg > 0 && angleFromStartDeg < turnDeg;
  return angleFromStartDeg < 0 && angleFromStartDeg > turnDeg;
}

inline void nav_gpsDeltaMeters(double refLatDeg, double refLngDeg,
                               double latDeg, double lngDeg, double &eastM,
                               double &northM) {
  double latRad = refLatDeg * M_PI / 180.0;
  northM = (latDeg - refLatDeg) * NAV_EARTH_RADIUS_M * M_PI / 180.0;
  eastM = (lngDeg - refLngDeg) * NAV_EARTH_RADIUS_M * std::cos(latRad) * M_PI /
          180.0;
}

inline double nav_crossTrackErrorMeters(double startLatDeg, double startLngDeg,
                                        double endLatDeg, double endLngDeg,
                                        double boatLatDeg, double boatLngDeg) {
  double pathEastM, pathNorthM, boatEastM, boatNorthM;
  nav_gpsDeltaMeters(startLatDeg, startLngDeg, endLatDeg, endLngDeg, pathEastM,
                     pathNorthM);
  nav_gpsDeltaMeters(startLatDeg, startLngDeg, boatLatDeg, boatLngDeg,
                     boatEastM, boatNorthM);

  double pathLengthM =
      std::sqrt(pathEastM * pathEastM + pathNorthM * pathNorthM);
  if (pathLengthM < 0.1)
    return 0.0;

  return (pathNorthM * boatEastM - pathEastM * boatNorthM) / pathLengthM;
}

inline bool nav_hasPassedWaypoint(double startLatDeg, double startLngDeg,
                                  double endLatDeg, double endLngDeg,
                                  double boatLatDeg, double boatLngDeg) {
  double pathEastM, pathNorthM, boatEastM, boatNorthM;
  nav_gpsDeltaMeters(startLatDeg, startLngDeg, endLatDeg, endLngDeg, pathEastM,
                     pathNorthM);
  nav_gpsDeltaMeters(startLatDeg, startLngDeg, boatLatDeg, boatLngDeg,
                     boatEastM, boatNorthM);

  double pathLengthSquared =
      pathEastM * pathEastM + pathNorthM * pathNorthM;
  if (pathLengthSquared < 0.01)
    return false;

  return boatEastM * pathEastM + boatNorthM * pathNorthM >=
         pathLengthSquared;
}

inline int nav_sideMovingTowardCrossTrack(double startLatDeg,
                                          double startLngDeg, double endLatDeg,
                                          double endLngDeg, double axisDeg,
                                          double safeAngleDeg,
                                          int desiredCrossTrackSign,
                                          int fallbackSide) {
  double pathEastM, pathNorthM;
  nav_gpsDeltaMeters(startLatDeg, startLngDeg, endLatDeg, endLngDeg, pathEastM,
                     pathNorthM);
  double pathLengthM =
      std::sqrt(pathEastM * pathEastM + pathNorthM * pathNorthM);
  if (pathLengthM < 0.1)
    return fallbackSide;

  double pathEastUnit = pathEastM / pathLengthM;
  double pathNorthUnit = pathNorthM / pathLengthM;
  int bestSide = fallbackSide;
  double bestScore = -1.0;

  for (int side = -1; side <= 1; side += 2) {
    double headingRad = nav_normalizeAngle(axisDeg + side * safeAngleDeg) *
                        M_PI / 180.0;
    double crossVelocity =
        pathNorthUnit * std::sin(headingRad) -
        pathEastUnit * std::cos(headingRad);

    if ((desiredCrossTrackSign > 0 && crossVelocity > 0) ||
        (desiredCrossTrackSign < 0 && crossVelocity < 0)) {
      double score = std::abs(crossVelocity);
      if (score > bestScore) {
        bestScore = score;
        bestSide = side;
      }
    }
  }

  return bestSide;
}

inline int nav_sideMovingTowardWaypoint(double axisDeg, double safeAngleDeg,
                                        double waypointHeadingDeg,
                                        int fallbackSide) {
  int bestSide = fallbackSide;
  double bestProgress = -2.0;
  double waypointHeadingRad = waypointHeadingDeg * M_PI / 180.0;

  for (int side = -1; side <= 1; side += 2) {
    double headingRad = nav_normalizeAngle(axisDeg + side * safeAngleDeg) *
                        M_PI / 180.0;
    double progress =
        std::sin(headingRad) * std::sin(waypointHeadingRad) +
        std::cos(headingRad) * std::cos(waypointHeadingRad);
    if (progress > bestProgress) {
      bestProgress = progress;
      bestSide = side;
    }
  }

  return bestSide;
}

struct NavResult {
  float sailAngle;
  float rudderAngle;
  int sendInterval;
  const char *mode;
  const char *logMessage;
  bool windAcquired;
  double acquiredWindDir;
  bool waypointReached;
  // Set by the branches that steer towards an explicit heading (feeds the auto-trim).
  bool headingTracked;
  double headingErrorDeg;
};

struct NavCorridor {
  bool initialized;
  double startLatDeg;
  double startLngDeg;
  double targetLatDeg;
  double targetLngDeg;
};

enum NavEmpannagePhase {
  NAV_EMPANNAGE_AUCUN = 0,
  NAV_EMPANNAGE_ALLER_LIMITE_UPWIND,
  NAV_EMPANNAGE_CROISER_AXE_UPWIND,
  NAV_EMPANNAGE_REVENIR_LIMITE_DOWNWIND
};

struct NavState {
  NavCorridor corridor;
  int upwindSide;
  int downwindSide;
  NavEmpannagePhase empannagePhase;
  int empannageTargetSide;
  // Auto-trim: learnt constant rudder offset (deg). Hardware property → survives resets.
  double rudderTrimDeg;
  // Watchdog
  NavEmpannagePhase watchedPhase;
  double phaseTimeS;
  double bestDistanceM;   // 0 = not initialised
  double noProgressS;
};

inline void nav_resetCorridor(NavState &state) { state.corridor = {}; }

// Resets the manoeuvre (sides, avoid-gybe phase, watchdog) but keeps the learnt trim.
inline void nav_resetManoeuvre(NavState &state) {
  state.upwindSide = 0;
  state.downwindSide = 0;
  state.empannagePhase = NAV_EMPANNAGE_AUCUN;
  state.empannageTargetSide = 0;
  state.watchedPhase = NAV_EMPANNAGE_AUCUN;
  state.phaseTimeS = 0.0;
  state.bestDistanceM = 0.0;
  state.noProgressS = 0.0;
}

inline void nav_resetState(NavState &state) {
  nav_resetCorridor(state);
  nav_resetManoeuvre(state);
}

inline void nav_resetTrim(NavState &state) { state.rudderTrimDeg = 0.0; }

inline bool nav_hasCorridor(double boatLatDeg, double boatLngDeg,
                            double waypointLatDeg, double waypointLngDeg,
                            double halfWidthM) {
  return halfWidthM > 0.0 &&
         (std::abs(waypointLatDeg - boatLatDeg) > NAV_GEO_EPSILON_DEG ||
          std::abs(waypointLngDeg - boatLngDeg) > NAV_GEO_EPSILON_DEG);
}

inline void nav_updateCorridor(NavState &state, bool hasCorridor,
                               double boatLatDeg, double boatLngDeg,
                               double waypointLatDeg, double waypointLngDeg) {
  if (!hasCorridor) {
    nav_resetCorridor(state);
    return;
  }

  bool targetChanged =
      std::abs(waypointLatDeg - state.corridor.targetLatDeg) >
          NAV_GEO_EPSILON_DEG ||
      std::abs(waypointLngDeg - state.corridor.targetLngDeg) >
          NAV_GEO_EPSILON_DEG;
  if (state.corridor.initialized && !targetChanged)
    return;

  state.corridor = {true, boatLatDeg, boatLngDeg, waypointLatDeg,
                    waypointLngDeg};
  nav_resetManoeuvre(state);
}

inline void nav_applyTargetHeading(NavResult &result, double boatHeadingDeg,
                                   double targetHeadingDeg,
                                   double relativeWindDeg, double rudderGain,
                                   const char *mode, const char *logMessage,
                                   int sendInterval) {
  double headingErrorDeg = nav_relativeAngle(boatHeadingDeg, targetHeadingDeg);
  result.sailAngle =
      (relativeWindDeg < 0) ? NAV_SAIL_LEFT_DEG : NAV_SAIL_RIGHT_DEG;
  result.rudderAngle = nav_clampRudder(-headingErrorDeg * rudderGain);
  result.sendInterval = sendInterval;
  result.mode = mode;
  result.logMessage = logMessage;
  result.headingTracked = true;
  result.headingErrorDeg = headingErrorDeg;
}

inline int nav_corridorSide(const NavState &state, double boatLatDeg,
                            double boatLngDeg, double corridorHalfWidthM,
                            double axisDeg, double safeAngleDeg,
                            int currentSide) {
  if (!state.corridor.initialized)
    return currentSide;

  double crossTrackErrorM = nav_crossTrackErrorMeters(
      state.corridor.startLatDeg, state.corridor.startLngDeg,
      state.corridor.targetLatDeg, state.corridor.targetLngDeg, boatLatDeg,
      boatLngDeg);
  if (crossTrackErrorM > corridorHalfWidthM) {
    return nav_sideMovingTowardCrossTrack(
        state.corridor.startLatDeg, state.corridor.startLngDeg,
        state.corridor.targetLatDeg, state.corridor.targetLngDeg, axisDeg,
        safeAngleDeg, -1, currentSide);
  }
  if (crossTrackErrorM < -corridorHalfWidthM) {
    return nav_sideMovingTowardCrossTrack(
        state.corridor.startLatDeg, state.corridor.startLngDeg,
        state.corridor.targetLatDeg, state.corridor.targetLngDeg, axisDeg,
        safeAngleDeg, 1, currentSide);
  }
  return currentSide;
}

inline void nav_startAvoidEmpannage(NavResult &result, NavState &state,
                                    double boatHeadingDeg,
                                    double windDirectionDeg,
                                    double relativeWind, int targetSide,
                                    const char *logMessage) {
  state.empannageTargetSide = targetSide;
  state.empannagePhase = NAV_EMPANNAGE_ALLER_LIMITE_UPWIND;

  double targetHeading = nav_normalizeAngle(
      windDirectionDeg - state.downwindSide * NAV_UPWIND_FORBIDDEN_ANGLE_DEG);
  nav_applyTargetHeading(result, boatHeadingDeg, targetHeading, relativeWind,
                         NAV_DIRECT_RUDDER_GAIN, "avoid-gybe", logMessage, 300);
}

inline void nav_handleEmpannageLoop(NavResult &result, NavState &state,
                                    double boatHeadingDeg,
                                    double windDirectionDeg,
                                    double oppositeWind, double relativeWind,
                                    bool phaseTimedOut = false) {
  double targetHeading;
  const char *mode = "avoid-gybe";
  const char *logMessage = "Avoid empannage: loop around through upwind";

  // phaseTimedOut (watchdog): the boat could not reach this phase's condition in
  // NAV_WATCHDOG_PHASE_TIMEOUT_S → advance anyway instead of holding a heading forever.
  if (state.empannagePhase == NAV_EMPANNAGE_ALLER_LIMITE_UPWIND) {
    targetHeading = nav_normalizeAngle(
        windDirectionDeg - state.downwindSide * NAV_UPWIND_FORBIDDEN_ANGLE_DEG);
    if (std::abs(nav_relativeAngle(boatHeadingDeg, targetHeading)) < 10.0 ||
        phaseTimedOut)
      state.empannagePhase = NAV_EMPANNAGE_CROISER_AXE_UPWIND;
  } else if (state.empannagePhase == NAV_EMPANNAGE_CROISER_AXE_UPWIND) {
    targetHeading = nav_normalizeAngle(
        windDirectionDeg + state.downwindSide * NAV_UPWIND_FORBIDDEN_ANGLE_DEG);
    mode = "vdb";
    logMessage = "VDB: downwind loop crosses upwind axis";
    if (nav_angleSide(nav_relativeAngle(windDirectionDeg, boatHeadingDeg)) ==
            state.downwindSide ||
        phaseTimedOut) {
      state.downwindSide = state.empannageTargetSide;
      state.empannagePhase = NAV_EMPANNAGE_REVENIR_LIMITE_DOWNWIND;
    }
  } else {
    targetHeading = nav_normalizeAngle(
        oppositeWind +
        state.empannageTargetSide * NAV_DOWNWIND_FORBIDDEN_ANGLE_DEG);
    if (nav_angleSide(nav_relativeAngle(oppositeWind, boatHeadingDeg)) ==
            state.empannageTargetSide ||
        phaseTimedOut) {
      state.downwindSide = state.empannageTargetSide;
      state.empannagePhase = NAV_EMPANNAGE_AUCUN;
    }
  }

  nav_applyTargetHeading(result, boatHeadingDeg, targetHeading, relativeWind,
                         NAV_DIRECT_RUDDER_GAIN, mode, logMessage, 300);
}

inline void nav_handleForbiddenZone(NavResult &result, NavState &state,
                                    double boatHeadingDeg, double boatLatDeg,
                                    double boatLngDeg, double windAxisDeg,
                                    double waypointHeadingDeg,
                                    double relativeWind, bool hasCorridor,
                                    double corridorHalfWidthM, bool upwind) {
  int &side = upwind ? state.upwindSide : state.downwindSide;
  double forbiddenAngle =
      upwind ? NAV_UPWIND_FORBIDDEN_ANGLE_DEG : NAV_DOWNWIND_FORBIDDEN_ANGLE_DEG;
  double gain = upwind ? NAV_VDB_RUDDER_GAIN : NAV_DIRECT_RUDDER_GAIN;
  const char *mode = upwind ? "upwind-zigzag" : "downwind-zigzag";
  const char *logMessage = upwind ? "Waypoint in upwind forbidden zone"
                                  : "Waypoint in downwind forbidden zone";
  int sendInterval = upwind ? 300 : 2000;

  if (side == 0) {
    int currentSide =
        nav_angleSide(nav_relativeAngle(windAxisDeg, boatHeadingDeg));
    side = nav_sideMovingTowardWaypoint(
        windAxisDeg, forbiddenAngle, waypointHeadingDeg, currentSide);
  }
  if (hasCorridor) {
    side = nav_corridorSide(state, boatLatDeg, boatLngDeg, corridorHalfWidthM,
                            windAxisDeg, forbiddenAngle, side);
  }

  double targetHeading = nav_normalizeAngle(windAxisDeg + side * forbiddenAngle);
  nav_applyTargetHeading(result, boatHeadingDeg, targetHeading, relativeWind,
                         gain, mode, logMessage, sendInterval);
}

inline void nav_handleDownwindZigzag(NavResult &result, NavState &state,
                                     double boatHeadingDeg, double boatLatDeg,
                                     double boatLngDeg, double windDirectionDeg,
                                     double oppositeWind,
                                     double waypointHeadingDeg,
                                     double relativeWind, bool hasCorridor,
                                     double corridorHalfWidthM) {
  if (state.downwindSide == 0) {
    state.downwindSide =
        nav_angleSide(nav_relativeAngle(oppositeWind, boatHeadingDeg));
  }

  int desiredSide = state.downwindSide;
  if (hasCorridor) {
    desiredSide = nav_corridorSide(state, boatLatDeg, boatLngDeg,
                                   corridorHalfWidthM, oppositeWind,
                                   NAV_DOWNWIND_FORBIDDEN_ANGLE_DEG,
                                   state.downwindSide);
  }
  if (desiredSide != state.downwindSide) {
    nav_startAvoidEmpannage(result, state, boatHeadingDeg, windDirectionDeg,
                            relativeWind, desiredSide,
                            "Avoid empannage: loop around through upwind");
    return;
  }

  nav_handleForbiddenZone(result, state, boatHeadingDeg, boatLatDeg, boatLngDeg,
                          oppositeWind, waypointHeadingDeg, relativeWind, false,
                          corridorHalfWidthM, false);
}

inline void nav_handleLoferAbattre(NavResult &result, double relativeWind,
                                   float currentRudderAngle, bool lofer) {
  bool windLeft = relativeWind < 0;
  result.sailAngle = windLeft ? NAV_SAIL_LEFT_DEG : NAV_SAIL_RIGHT_DEG;
  result.rudderAngle =
      currentRudderAngle +
      (windLeft == lofer ? NAV_RUDDER_STEP_DEG : -NAV_RUDDER_STEP_DEG);
  result.mode = lofer ? "lofer" : "abattre";
  result.logMessage =
      lofer ? (windLeft ? "Lofer relativeWind < 0" : "Lofer relativeWind > 0")
            : (windLeft ? "Abattre relativeWind < 0"
                        : "Abattre relativeWind > 0");
}

inline NavResult
nav_handleWindObservation(double /* boatLat */, double /* boatLng */,
                          double /* startWptLat */, double /* startWptLng */,
                          double smoothHeadingDeg, double distanceFromStartM,
                          double requiredWindDistanceM, float currentSailAngle,
                          float currentRudderAngle) {
  NavResult result = {};
  result.sailAngle = currentSailAngle;
  result.rudderAngle = currentRudderAngle;
  result.sendInterval = 1000;
  result.mode = "wind-observation";

  if (distanceFromStartM >= requiredWindDistanceM) {
    result.windAcquired = true;
    result.acquiredWindDir = fmod(smoothHeadingDeg + 90.0 + 360.0, 360.0);
    result.logMessage = "wind acquired";
  }

  return result;
}

inline NavResult nav_handleNavigationWithState(
    NavState &state, double boatHeadingDeg, double waypointHeadingDeg,
    double waypointDistanceM, double windDirectionDeg, float currentSailAngle,
    float currentRudderAngle, double waypointReachedDistanceM,
    double boatLatDeg = 0.0, double boatLngDeg = 0.0,
    double waypointLatDeg = 0.0, double waypointLngDeg = 0.0,
    double corridorHalfWidthM = 0.0, double dtS = NAV_DEFAULT_DT_S) {
  NavResult result = {};
  result.sailAngle = currentSailAngle;
  result.rudderAngle = currentRudderAngle;
  result.sendInterval = 2000;
  result.mode = "direct";
  if (waypointDistanceM <= waypointReachedDistanceM) {
    result.waypointReached = true;
    nav_resetState(state);
    return result;
  }

  bool hasCorridor = nav_hasCorridor(boatLatDeg, boatLngDeg, waypointLatDeg,
                                     waypointLngDeg, corridorHalfWidthM);
  nav_updateCorridor(state, hasCorridor, boatLatDeg, boatLngDeg, waypointLatDeg,
                     waypointLngDeg);
  if (state.corridor.initialized &&
      nav_hasPassedWaypoint(
          state.corridor.startLatDeg, state.corridor.startLngDeg,
          state.corridor.targetLatDeg, state.corridor.targetLngDeg, boatLatDeg,
          boatLngDeg) &&
      std::abs(nav_crossTrackErrorMeters(
          state.corridor.startLatDeg, state.corridor.startLngDeg,
          state.corridor.targetLatDeg, state.corridor.targetLngDeg, boatLatDeg,
          boatLngDeg)) <= corridorHalfWidthM) {
    result.waypointReached = true;
    nav_resetState(state);
    return result;
  }

  double oppositeWind = nav_oppositeAngle(windDirectionDeg);
  double relativeWind = nav_relativeAngle(boatHeadingDeg, windDirectionDeg);
  double relativeWpt = nav_relativeAngle(boatHeadingDeg, waypointHeadingDeg);
  // The previous command carries the trim: remove it before reading back the pure
  // correction, otherwise the lofer/abattre integrator would count the trim twice.
  float currentRudderCorrection = nav_rudderCorrection(
      currentRudderAngle - (float)state.rudderTrimDeg, relativeWind);

  // ── Watchdog: time spent in the current avoid-gybe phase, and progress ──
  if (state.empannagePhase != state.watchedPhase) {
    state.watchedPhase = state.empannagePhase;
    state.phaseTimeS = 0.0;
  } else {
    state.phaseTimeS += dtS;
  }
  const bool phaseTimedOut =
      state.empannagePhase != NAV_EMPANNAGE_AUCUN &&
      state.phaseTimeS > NAV_WATCHDOG_PHASE_TIMEOUT_S;

  if (state.bestDistanceM <= 0.0 ||
      waypointDistanceM < state.bestDistanceM - NAV_WATCHDOG_PROGRESS_M) {
    state.bestDistanceM = waypointDistanceM;
    state.noProgressS = 0.0;
  } else {
    state.noProgressS += dtS;
  }
  bool noProgressReset = false;
  if (state.noProgressS > NAV_WATCHDOG_NO_PROGRESS_S) {
    // Nothing gained for NAV_WATCHDOG_NO_PROGRESS_S: drop the manoeuvre state and
    // decide again from the current situation (the learnt trim is kept).
    nav_resetManoeuvre(state);
    state.bestDistanceM = waypointDistanceM;
    noProgressReset = true;
  }
  bool waypointUpwind =
      std::abs(nav_relativeAngle(windDirectionDeg, waypointHeadingDeg)) <
      NAV_UPWIND_FORBIDDEN_ANGLE_DEG;
  bool waypointDownwind =
      std::abs(nav_relativeAngle(oppositeWind, waypointHeadingDeg)) <
      NAV_DOWNWIND_FORBIDDEN_ANGLE_DEG;
  bool directCrossesUpwind = nav_isBetweenOnShortestTurn(
      windDirectionDeg, boatHeadingDeg, waypointHeadingDeg);
  bool directCrossesDownwind = nav_isBetweenOnShortestTurn(
      oppositeWind, boatHeadingDeg, waypointHeadingDeg);

  if (state.empannagePhase != NAV_EMPANNAGE_AUCUN) {
    nav_handleEmpannageLoop(result, state, boatHeadingDeg, windDirectionDeg,
                             oppositeWind, relativeWind, phaseTimedOut);
  } else if (waypointUpwind || directCrossesUpwind) {
    nav_handleForbiddenZone(result, state, boatHeadingDeg, boatLatDeg,
                            boatLngDeg, windDirectionDeg, waypointHeadingDeg,
                            relativeWind, hasCorridor, corridorHalfWidthM, true);
  } else if (waypointDownwind) {
    nav_handleDownwindZigzag(result, state, boatHeadingDeg, boatLatDeg,
                             boatLngDeg, windDirectionDeg, oppositeWind,
                             waypointHeadingDeg, relativeWind, hasCorridor,
                             corridorHalfWidthM);
  } else if (directCrossesDownwind) {
    if (state.downwindSide == 0) {
      state.downwindSide =
          nav_angleSide(nav_relativeAngle(oppositeWind, boatHeadingDeg));
    }
    nav_startAvoidEmpannage(result, state, boatHeadingDeg, windDirectionDeg,
                            relativeWind, -state.downwindSide,
                            "Avoid empannage: direct path crosses downwind axis");
  } else if (std::abs(relativeWpt) <= NAV_DIRECT_DEAD_ZONE_DEG) {
    result.sailAngle =
        relativeWind < 0 ? NAV_SAIL_LEFT_DEG : NAV_SAIL_RIGHT_DEG;
    result.rudderAngle = 0;
    result.logMessage = "Direct: waypoint aligned";
    result.headingTracked = true;          // aligned: the target heading IS the waypoint
    result.headingErrorDeg = relativeWpt;
  } else {
    nav_handleLoferAbattre(result, relativeWind, currentRudderCorrection,
                           nav_sameSign(relativeWind, relativeWpt));
  }

  // ── Auto-trim: learn the constant rudder offset from the heading error ──
  // Only while steering towards an explicit heading and near it (anti wind-up).
  if (result.headingTracked && dtS > 0.0 &&
      std::abs(result.headingErrorDeg) < NAV_TRIM_ERROR_WINDOW_DEG) {
    state.rudderTrimDeg -= NAV_TRIM_GAIN_PER_S * result.headingErrorDeg * dtS;
    if (state.rudderTrimDeg > NAV_TRIM_MAX_DEG)
      state.rudderTrimDeg = NAV_TRIM_MAX_DEG;
    if (state.rudderTrimDeg < -NAV_TRIM_MAX_DEG)
      state.rudderTrimDeg = -NAV_TRIM_MAX_DEG;
  }

  if (noProgressReset)
    result.logMessage = "Watchdog: no progress, manoeuvre reset";

  // Command = wind feed-forward + clamped correction, then the trim OUTSIDE that
  // clamp (it must be able to cancel an offset larger than the correction limit).
  result.rudderAngle = nav_rudderCommand(result.rudderAngle, relativeWind);
  result.rudderAngle += (float)state.rudderTrimDeg;
  if (result.rudderAngle > NAV_RUDDER_COMMAND_LIMIT_DEG)
    result.rudderAngle = NAV_RUDDER_COMMAND_LIMIT_DEG;
  if (result.rudderAngle < -NAV_RUDDER_COMMAND_LIMIT_DEG)
    result.rudderAngle = -NAV_RUDDER_COMMAND_LIMIT_DEG;
  return result;
}

inline NavResult nav_handleNavigation(
    double boatHeadingDeg, double waypointHeadingDeg, double waypointDistanceM,
    double windDirectionDeg, float currentSailAngle, float currentRudderAngle,
    double waypointReachedDistanceM, double boatLatDeg = 0.0,
    double boatLngDeg = 0.0, double waypointLatDeg = 0.0,
    double waypointLngDeg = 0.0, double corridorHalfWidthM = 0.0,
    double dtS = NAV_DEFAULT_DT_S) {
  static NavState state = {};
  return nav_handleNavigationWithState(
      state, boatHeadingDeg, waypointHeadingDeg, waypointDistanceM,
      windDirectionDeg, currentSailAngle, currentRudderAngle,
      waypointReachedDistanceM, boatLatDeg, boatLngDeg, waypointLatDeg,
      waypointLngDeg, corridorHalfWidthM, dtS);
}

#endif // NAVIGATION_H
