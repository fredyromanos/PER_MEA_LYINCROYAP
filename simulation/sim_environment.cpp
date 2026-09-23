#include "sim_environment.hpp"
#include <iostream>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265359
#endif

// Constantes physiques
const double EARTH_RADIUS_M = 6371000.0;  // Rayon de la Terre en mètres

// ── Stage 1 surge dynamics (docs/SIMULATION_PHYSICS_REVIEW.md) ──
// Force balance: u_dot = (F_sail + F_prop - R_hull(u)) / M_EFF.
// F_sail    = SURGE_F_MAX * polar * sailEff * (wind/SURGE_WIND_REF)^2  [N]
// R_hull(u) = SURGE_R_QUAD * u^2 + SURGE_R_LIN * u                    [N]
// Coefficients are ILLUSTRATIVE (not measured) and tuned so terminal speed ≈ 2.5 m/s
// at max drive (polar=1, sailEff=0.9, wind=5 m/s), matching the old kinematic maxSpeed.
constexpr double SURGE_F_MAX    = 25.0;   // N, max sail forward drive
constexpr double SURGE_R_QUAD   = 3.24;   // N·s²/m², quadratic hull resistance
constexpr double SURGE_R_LIN    = 0.9;    // N·s/m, linear hull resistance
constexpr double SURGE_M_EFF    = 50.0;   // kg, mass + added mass
constexpr double SURGE_WIND_REF = 5.0;    // m/s, reference wind for the drive law

SimulationEnvironment::SimulationEnvironment() {
    state.latitude = 0;
    state.longitude = 0;
    state.heading = 0;
    state.speed = 0;
    state.sailAngle = 0;
    state.rudderAngle = 0;
    state.windDirection = 0;
    state.windSpeed = 5.0;  // 5 m/s par défaut
    state.time = 0;
}

void SimulationEnvironment::init(double startLat, double startLng, double windDir, double windSpd, double initialHeading) {
    state.latitude = startLat;
    state.longitude = startLng;
    state.heading = initialHeading;
    state.speed = 0;
    state.windDirection = windDir;
    state.windSpeed = windSpd;
    state.time = 0;
    history.clear();
    history.push_back(state);
    
    std::cout << "[SIM] Environment initialized" << std::endl;
    std::cout << "  Position: " << startLat << ", " << startLng << std::endl;
    std::cout << "  Wind: " << windDir << "° at " << windSpd << " m/s" << std::endl;
}

void SimulationEnvironment::setServoAngles(float sail, float rudder) {
    state.sailAngle = sail;
    state.rudderAngle = rudder;
}

void SimulationEnvironment::setPropellerSpeed(double speedMs) {
    propellerSpeedMs = speedMs < 0 ? 0 : speedMs;
}

void SimulationEnvironment::setWind(double windDir, double windSpeed) {
    state.windDirection = windDir;
    state.windSpeed = windSpeed;
}

void SimulationEnvironment::setNavMode(int mode) {
    state.navMode = mode;
}

void SimulationEnvironment::addWaypoint(double lat, double lng) {
    std::cout << "[SIM] Waypoint added: " << lat << ", " << lng << std::endl;
}

void SimulationEnvironment::update(unsigned long dt_ms) {
    // sailAngle = angle de l'aileron de voile (±10° ou 0)
    // rudderAngle = déphasage servo safran par rapport à la liaison mécanique
    updateBoatDynamics(state.sailAngle, state.rudderAngle, dt_ms);
    
    state.time += dt_ms;
    history.push_back(state);
}

void SimulationEnvironment::updateBoatDynamics(float aileronAngle, float rudderOffset, unsigned long dt_ms) {
    double dt_s = dt_ms / 1000.0;
    
    // ════════════════════════════════════════════════════════════
    // 1. VENT RELATIF
    // ════════════════════════════════════════════════════════════
    // relativeWind > 0 → vent de tribord, < 0 → vent de bâbord
    double relativeWind = state.windDirection - state.heading;
    while (relativeWind > 180) relativeWind -= 360;
    while (relativeWind < -180) relativeWind += 360;
    double absRelWind = std::abs(relativeWind);
    
    // ════════════════════════════════════════════════════════════
    // 2. POSITION RÉELLE DE LA VOILE (girouette + aileron)
    // ════════════════════════════════════════════════════════════
    // La voile est une AILE RIGIDE libre en rotation (girouette).
    // Elle s'aligne naturellement avec le flux de vent.
    // L'aileron (servo ±10°) crée un angle d'attaque qui génère
    // la portance aérodynamique.
    //
    //   Vent → ─────────►  (flux de vent)
    //              ╱  ← angle d'attaque (~= aileron angle)
    //          VOILE   (aile rigide)
    //
    // L'aileron détermine :
    //   1. Le CÔTÉ : aileron > 0 → portance vers bâbord
    //                aileron < 0 → portance vers tribord
    //   2. L'angle d'attaque ≈ |aileronAngle| (0° à 10°)
    //   aileron = 0 → voile libre, faseye, pas de portance
    double angleOfAttack = std::abs(aileronAngle);  // 0° à 10°
    
    // ════════════════════════════════════════════════════════════
    // 3. EFFICACITÉ VOILE : portance aile rigide + cohérence
    // ════════════════════════════════════════════════════════════
    // L'aileron est "cohérent" quand il pousse la voile sous le vent
    // (signe aileron = signe vent relatif).
    // Si incohérent ou neutre → la voile faseye, très peu de propulsion.
    bool aileronCoherent;
    if (std::abs(aileronAngle) < 1.0) {
        aileronCoherent = false;  // aileron neutre → voile libre → faseye
    } else {
        aileronCoherent = (aileronAngle > 0 && relativeWind > 0) ||
                          (aileronAngle < 0 && relativeWind < 0);
    }
    
    // Portance aile rigide : dépend de l'angle d'attaque.
    // Profil symétrique : Cl ≈ 2π * α (en radians) pour petits angles.
    // À 10° d'AoA → Cl ≈ 1.1, ce qui est excellent pour une aile.
    // On normalise : efficacité = AoA / 10° (linéaire, pas de décrochage
    // car l'aileron ne dépasse pas 10°).
    double wingLiftCoeff = aileronCoherent ? (angleOfAttack / 10.0) : 0.0;
    // Si l'aileron n'est pas coherent, la voile faseye : pas de propulsion utile.
    double sailEff = aileronCoherent ? (wingLiftCoeff * 0.9) : 0.0;
    
    // Polaire simplifiée (coefficient de vitesse selon l'angle au vent)
    // Pour une aile rigide, le reaching est toujours optimal.
    double polarCoeff;
    if (absRelWind < 35) {
        polarCoeff = 0.0;  // zone interdite : le bateau ne peut pas avancer face au vent
    } else if (absRelWind < 60) {
        polarCoeff = 0.2 + 0.8 * (absRelWind - 35) / 25.0;  // 0.2 → 1.0
    } else if (absRelWind < 110) {
        polarCoeff = 1.0;   // reaching → optimal
    } else if (absRelWind < 150) {
        polarCoeff = 1.0 - 0.3 * (absRelWind - 110) / 40.0;  // 1.0 → 0.7
    } else {
        polarCoeff = 0.7 - 0.3 * (absRelWind - 150) / 30.0;  // 0.7 → 0.4
    }
    
    // ── Stage 1 surge dynamics: force balance instead of target-speed convergence ──
    // The sail drive and hull resistance now determine terminal speed physically;
    // the hard-coded maxSpeed clamp is gone (resistance limits speed instead).
    const double windRatio = state.windSpeed / SURGE_WIND_REF;
    const double sailForce = SURGE_F_MAX * polarCoeff * sailEff * windRatio * windRatio;

    // Propeller: the force that holds the requested cruise speed against hull
    // resistance, so it still acts as a speed floor without a hard clamp.
    double propForce = 0.0;
    if (propellerSpeedMs > 0.0) {
        propForce = SURGE_R_QUAD * propellerSpeedMs * propellerSpeedMs
                  + SURGE_R_LIN  * propellerSpeedMs;
    }

    const double resistance = SURGE_R_QUAD * state.speed * state.speed
                            + SURGE_R_LIN  * state.speed;

    // u_dot = (F_sail + F_prop - R_hull(u)) / m_eff, integrated with Euler.
    const double u_dot = (sailForce + propForce - resistance) / SURGE_M_EFF;
    state.speed += u_dot * dt_s;
    if (state.speed < 0.0) state.speed = 0.0;
    // When there is no drive, kill the residual creep so the boat truly stops.
    if (sailForce + propForce <= 0.0 && state.speed < 0.01) state.speed = 0.0;
    // (surge dynamics above replace the old target-speed/inertia convergence)
    
    // ════════════════════════════════════════════════════════════
    // 4. SAFRAN = LIAISON MÉCANIQUE + DÉPHASAGE SERVO
    // ════════════════════════════════════════════════════════════
    // Liaison mécanique 2:1 : quand la voile (girouette) tourne,
    // le safran suit automatiquement pour compenser la poussée
    // latérale → le bateau va droit avec offset = 0.
    //
    // Le servo safran ajoute un déphasage (rudderOffset) par-dessus
    // cette position de base pour faire tourner le bateau.
    //
    // Seul l'offset produit un virage — la liaison se compense.
    // ════════════════════════════════════════════════════════════
    double steeringEff = std::min(1.0, state.speed / 0.5);  // plein effet à 0.5 m/s
    double turnRate = -(rudderOffset + rudderBiasDeg) * 0.5 * steeringEff;    // rudder > 0 → bâbord (heading ↓), < 0 → tribord (heading ↑)
    state.heading += turnRate * dt_s;
    
    while (state.heading >= 360) state.heading -= 360;
    while (state.heading < 0) state.heading += 360;
    
    // ════════════════════════════════════════════════════════════
    // 5. DÉPLACEMENT GPS
    // ════════════════════════════════════════════════════════════
    double moveDistanceM = state.speed * dt_s;
    double dx = moveDistanceM * std::sin(state.heading * M_PI / 180.0);
    double dy = moveDistanceM * std::cos(state.heading * M_PI / 180.0);
    
    double metersPerDegreeLat = EARTH_RADIUS_M * M_PI / 180.0;
    double refLat = state.latitude;
    double metersPerDegreeLng = EARTH_RADIUS_M * std::cos(refLat * M_PI / 180.0) * M_PI / 180.0;
    
    state.latitude += dy / metersPerDegreeLat;
    state.longitude += dx / metersPerDegreeLng;
}

void SimulationEnvironment::computeDistanceToWaypoint(double wptLat, double wptLng,
                                                       double &distance, double &heading) const {
    distanceAndBearing(state.latitude, state.longitude, wptLat, wptLng, distance, heading);
}

// Local flat-earth geometry, consistent with the physics above (longitude scaled by
// cos(latitude)). Matches the firmware Navigator::distanceM / bearingDeg to well under
// 1 m / 0.1° at the scale of a mission. The previous version did NOT scale longitude:
// at 48°N east-west distances were ~50 % too long and diagonal bearings were skewed.
void SimulationEnvironment::distanceAndBearing(double fromLat, double fromLng,
                                               double toLat, double toLng,
                                               double &distance, double &heading) {
    const double metersPerDegreeLat = EARTH_RADIUS_M * M_PI / 180.0;
    const double meanLat = (fromLat + toLat) * 0.5 * M_PI / 180.0;
    const double northM = (toLat - fromLat) * metersPerDegreeLat;
    const double eastM  = (toLng - fromLng) * metersPerDegreeLat * std::cos(meanLat);
    distance = std::sqrt(northM * northM + eastM * eastM);
    heading = std::atan2(eastM, northM) * 180.0 / M_PI;
    if (heading < 0) heading += 360;
}
