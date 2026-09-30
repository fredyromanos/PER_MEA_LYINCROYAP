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
#include <string>
#include <vector>
#include <iostream>
#include "../main/src/navigation/navigation.h"

// Constantes de navigation (identiques à config_pins.h du vrai bateau)
static constexpr double WAYPOINT_DISTANCE_SIM = 10.0;   // mètres
static constexpr double WIND_DISTANCE_SIM     = 30.0;   // mètres

struct Waypoint {
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

    void setCorridorWidth(double widthMeters) 
    { 
    corridorWidth = widthMeters; 
    }

    void setWindChaos(double chaos) 
    { windChaos = std::max(0.0, std::min(1.0, chaos)); 
    }

    void printStatus() const;

    /**
     * Force la direction du vent connue par la navigation.
     * Utile pour tester la nav avec un vent correct quand l'observation est imprécise.
     * (Le vrai code déduit le vent par heading+90, ce qui n'est exact que
     * quand le vent est à 90° relatif — la simulation peut corriger.)
     */
    void setWind(double windDir, double windSpeed) {
        windDirection = windDir;
        environment.setWind(windDir, windSpeed);
        std::cout << "[SIM] Wind set to " << windDir << " deg at "
                  << windSpeed << " m/s" << std::endl;
    }

    void setWindDirection(double windDir) {
        setWind(windDir, environment.getWindSpeed());
    }

    /**
     * Force le mode du bateau (pour sauter l'observation du vent par ex.)
     */
    void setBoatMode(const std::string& mode) { boatMode = mode; }

private:
    SimulationEnvironment environment;
    std::vector<Waypoint> waypoints;
    int currentWaypointId;
    std::string boatMode;

    // === Variables globales du vrai bateau (reproduites ici) ===
    double windDirection;
    double initialWindDirection;
    float sailAngle;                  // Angle voile courant (accumulé par navigation.h)
    float rudderAngle;                // Commande servo: compensation + correction

    // Observation du vent : position de départ (simule currentWptLat/Lng)
    double windObsStartLat;
    double windObsStartLng;

    // Etat interne requis par la nouvelle logique de navigation partagee
    NavState navState;

    // Appelle le VRAI code de navigation depuis boat/navigation.h
    void updateNavigationLogic();
    void applyServoOutput();
    double corridorWidth = 100.0;
    double windChaos = 0.0;
    double targetWindDir = 0.0;
    unsigned long lastWindChangeTime = 0;
};

#endif // SIM_BOAT_HPP
