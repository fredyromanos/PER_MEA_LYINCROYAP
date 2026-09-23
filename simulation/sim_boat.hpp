/**
 * Wrapper pour exécuter la logique RÉELLE du bateau en simulation
 * 
 * Ce fichier utilise boat/navigation.h (le VRAI code de navigation)
 * et le connecte à l'environnement de simulation physique.
 * 
 * Modifier navigation.h = modifier le comportement réel ET simulé.
 */

#ifndef SIM_BOAT_HPP
#define SIM_BOAT_HPP

#include "sim_environment.hpp"
#include "control/HeadingGate.h"   // REAL firmware heading gate (GPS-only heading)
#include <string>
#include <vector>
#include <iostream>

// Constantes de navigation (identiques à config_pins.h du vrai bateau)
static constexpr double WAYPOINT_DISTANCE_SIM = 10.0;   // mètres
static constexpr double WIND_DISTANCE_SIM     = 30.0;   // mètres

// Modèle GPS réaliste (NEO-6M, pas de compas) — voir SimulatedBoat::updateGpsModel().
static constexpr unsigned long SIM_GPS_PERIOD_MS   = 1000;  // 1 Hz
static constexpr double        SIM_GPS_POS_NOISE_M = 3.0;   // ± bruit de position (uniforme)
static constexpr double        SIM_GPS_CRS_NOISE_DEG = 5.0; // ± bruit de cap GPS en mouvement
// Vitesse donnée par l'hélice à AUTO_ESC_CRUISE_US (1700 µs). HYPOTHÈSE non mesurée :
// à mesurer sur l'eau. Doit dépasser HEADING_SPEED_ON_KMPH (1.5 km/h = 0.42 m/s).
static constexpr double        SIM_PROP_CRUISE_MS  = 0.8;

// Named SimWaypoint (not Waypoint): the firmware core/Types.h, pulled in by
// HeadingGate.h, already defines a different Waypoint struct.
struct SimWaypoint {
    double lat;
    double lng;
};

class SimulatedBoat {
public:
    SimulatedBoat();
    
    /**
     * Initialise le bateau et la simulation
     * @param startLat Latitude initiale
     * @param startLng Longitude initiale
     * @param windDir Direction du vent (degrés)
     * @param windSpeed Vitesse du vent (m/s)
     * @param initialHeading Cap initial du bateau (degrés, 0=Nord)
     */
    void init(double startLat, double startLng, double windDir, double windSpeed, double initialHeading = 0);
    
    void addWaypoint(double lat, double lng);
    void startNavigation();
    void stopNavigation();
    void startWindObservation();
    
    /**
     * Exécute une étape de la boucle principale du bateau
     */
    void stepSimulation(unsigned long dt_ms);
    
    /**
     * Exécute la simulation pour un temps donné
     */
    void runSimulation(unsigned long duration_ms, unsigned long timeStep_ms = 100);
    
    const SimBoatState& getState() const { return environment.getState(); }
    const std::vector<SimBoatState>& getHistory() const { return environment.getHistory(); }
    double getInitialWindDir() const { return initialWindDirection; }
    double getWindSpeed() const { return environment.getWindSpeed(); }
    std::vector<std::pair<double, double>> getWaypointPairs() const {
        std::vector<std::pair<double, double>> v;
        for (const auto& w : waypoints) v.push_back({w.lat, w.lng});
        return v;
    }
    
    void printStatus() const;
    
    /**
     * Force la direction du vent connue par la navigation.
     * Utile pour tester la nav avec un vent correct quand l'observation est imprécise.
     * (Le vrai code déduit le vent par heading+90, ce qui n'est exact que
     * quand le vent est à 90° relatif — la simulation peut corriger.)
     */
    void setWindDirection(double windDir) { 
        windDirection = windDir; 
        environment.setWind(windDir, environment.getWindSpeed());
        std::cout << "[SIM] Wind direction overridden to " << windDir << " deg" << std::endl;
    }
    
    /**
     * Force le mode du bateau (pour sauter l'observation du vent par ex.)
     */
    void setBoatMode(const std::string& mode) { boatMode = mode; }
    
    // Statistiques du modèle GPS (pour le résumé de fin de scénario)
    unsigned long navStepsNoHeading() const { return stepsNoHeading; }
    unsigned long navStepsTotal() const { return stepsNav; }

    // === Accès pour le simulateur IHM (IHM/AutoBoat_Simulation/simu_gps) ===
    const GpsPosition&        gpsPosition()  const { return gps; }
    const HeadingGate::State& headingState() const { return gate.state(); }
    const std::string&        mode()         const { return boatMode; }
    int    waypointIndex() const { return currentWaypointId < 0 ? 0 : currentWaypointId; }
    size_t waypointCount() const { return waypoints.size(); }
    float  sailCommand()   const { return sailAngle; }
    float  rudderCommand() const { return rudderAngle; }
    double knownWindDirection() const { return windDirection; }
    const char* navState()  const { return navStateLabel; }
    void   clearWaypoints() { waypoints.clear(); currentWaypointId = -1; }
    // Wind the BOAT believes (wind-command), without changing the physical wind.
    void   setKnownWind(double windDir) { windDirection = windDir; }
    // Idle firmware = all actuators neutral (sail centred → no drive, propeller stopped).
    void   setSpeed(double speedMs) { environment.setSpeed(speedMs); }
    void   setRudderBias(double deg) { environment.setRudderBias(deg); }
    void   holdNeutral() { environment.setServoAngles(0.0f, 0.0f); environment.setPropellerSpeed(0.0); }
    // Perte de fix périodique : toutes les periodS secondes, pas de fix pendant durationS.
    void   setGpsDropout(unsigned long periodS, unsigned long durationS) {
        dropoutPeriodMs = periodS * 1000UL; dropoutDurationMs = durationS * 1000UL;
    }
    unsigned long lastFixMs() const { return lastValidFixMs; }

private:
    SimulationEnvironment environment;
    std::vector<SimWaypoint> waypoints;

    // === GPS réaliste + gate de cap (code firmware) ===
    // SIM_IDEAL_GPS=1 dans l'environnement → ancien comportement (cap et position exacts).
    bool          idealGps;
    HeadingGate   gate;
    GpsPosition   gps;
    unsigned long lastGpsMs;
    bool          acquiring = false;
    unsigned long acquireStartMs = 0;
    unsigned long dropoutPeriodMs = 0;     // 0 = pas de perte de fix simulée
    unsigned long dropoutDurationMs = 0;
    unsigned long lastValidFixMs = 0;
    const char*   navStateLabel = "idle";
    uint32_t      rng;
    unsigned long stepsNoHeading;
    unsigned long stepsNav;
    void   updateGpsModel(const SimBoatState& s);
    double noise(double amplitude);   // uniforme dans [-amplitude, +amplitude], déterministe
    int currentWaypointId;
    std::string boatMode;
    
    // === Variables globales du vrai bateau (reproduites ici) ===
    double windDirection;
    double initialWindDirection;
    float sailAngle;                  // Angle voile courant (accumulé par navigation.h)
    float rudderAngle;                // Angle gouvernail courant (accumulé par navigation.h)
    
    // Observation du vent : position de départ (simule currentWptLat/Lng)
    double windObsStartLat;
    double windObsStartLng;
    
    // Appelle le VRAI code de navigation depuis boat/navigation.h
    void updateNavigationLogic();
    void applyServoOutput();
};

#endif // SIM_BOAT_HPP
