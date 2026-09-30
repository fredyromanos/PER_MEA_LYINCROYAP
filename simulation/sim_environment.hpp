/**
 * Simulateur Physique du Bateau
 * 
 * Simule:
 * - Position GPS du bateau
 * - Vent
 * - Physique du bateau (cap, vitesse)
 * - Interaction entre les angles de servos et le mouvement du bateau
 */

#ifndef SIM_ENVIRONMENT_HPP
#define SIM_ENVIRONMENT_HPP

#include <cmath>
#include <vector>
#include <string>

struct SimBoatState {
    // Position GPS
    double latitude;      // En degrés
    double longitude;     // En degrés
    
    // Cap et vitesse
    double heading;       // En degrés (0-360)
    double speed;         // En m/s
    
    // Angles des servos
    float sailAngle;      // -10 à +10°
    float rudderAngle;    // -20 à +20°
    
    // Vent
    double windDirection; // Direction d'où vient le vent (0-360°)
    double windSpeed;     // En m/s

    // Route et vitesse SUR LE FOND (over ground) — ce qu'un vrai récepteur GPS
    // mesure (Doppler / delta de position), PAS la vitesse à travers l'eau.
    // Identiques à heading/speed tant qu'aucun courant/leeway n'est actif
    // (valeurs par défaut) ; voir SimulationEnvironment::setCurrent/setLeewayGain.
    double courseOverGround; // En degrés (0-360)
    double speedOverGround;  // En m/s

    // Temps
    unsigned long time;   // En ms
    
    // Mode de navigation (0=observation, 1=vdb, 2=direct, 3=lofer,
    // 4=abattre, 5=standby, 6=upwind-zigzag, 7=downwind-zigzag,
    // 8=avoid-gybe)
    int navMode = 0;
};

class SimulationEnvironment {
public:
    SimulationEnvironment();
    
    /**
     * Initialise la simulation avec position, vent, et waypoints
     * @param initialHeading Cap initial du bateau (degrés, 0=Nord)
     */
    void init(double startLat, double startLng, double windDir, double windSpd, double initialHeading = 0);
    
    /**
     * Avance la simulation d'un pas de temps dt_ms
     * Recalcule position, cap, vitesse selon les angles des servos et le vent
     */
    void update(unsigned long dt_ms);
    
    /**
     * Applique les angles des servos et met à jour la physique
     */
    void setServoAngles(float sail, float rudder);

    /**
     * Hélice : vitesse minimale imposée (m/s), 0 = hélice arrêtée (défaut).
     * Utilisée par le modèle GPS réaliste pendant l'acquisition du cap.
     */
    void setPropellerSpeed(double speedMs);

    /** Vitesse actuelle imposée (m/s) — ex. prédiction partant d'un bateau déjà en mouvement. */
    void setSpeed(double speedMs) { state.speed = speedMs < 0 ? 0 : speedMs; }

    /**
     * Crée un waypoint dans la simulation
     */
    void addWaypoint(double lat, double lng);
    
    /**
     * Change la direction du vent
     */
    void setWind(double windDir, double windSpeed);
    
    /**
     * Définit le mode de navigation actuel (pour export)
     */
    void setNavMode(int mode);
    
    // === GETTERS ===
    double getLatitude() const { return state.latitude; }
    double getLongitude() const { return state.longitude; }
    double getHeading() const { return state.heading; }
    double getSpeed() const { return state.speed; }
    float getSailAngle() const { return state.sailAngle; }
    float getRudderAngle() const { return state.rudderAngle; }
    double getWindDirection() const { return state.windDirection; }
    double getWindSpeed() const { return state.windSpeed; }
    double getCourseOverGround() const { return state.courseOverGround; }
    double getSpeedOverGround() const { return state.speedOverGround; }

    /**
     * Calcule distance et cap vers un waypoint
     */
    static void distanceAndBearing(double fromLat, double fromLng, double toLat, double toLng,
                                   double &distance, double &heading);
    void computeDistanceToWaypoint(double wptLat, double wptLng,
                                   double &distance, double &heading) const;
    
    /**
     * Retourne l'état complet du bateau
     */
    const SimBoatState& getState() const { return state; }
    
    /**
     * Retourne l'historique complet des états
     */
    const std::vector<SimBoatState>& getHistory() const { return history; }

private:
    SimBoatState state;
    double propellerSpeedMs = 0.0;   // 0 = hélice arrêtée
    double rudderBiasDeg = 0.0;      // décalage constant du safran (jeu, winch mal centré) — 0 = parfait

    // Courant marin : vitesse (m/s) et direction VERS LAQUELLE il PORTE (convention
    // "set" océanographique — l'INVERSE de windDirection ci-dessus, qui est la
    // direction D'OÙ VIENT le vent). Même convention que adversarial/boat_model.hpp
    // ::currentDir, pour que les deux outils restent d'accord. Défauts = 0 : aucun
    // courant ⇒ comportement bit-à-bit identique aux 6 scénarios existants.
    double currentDirDeg  = 0.0;
    double currentSpeedMs = 0.0;
    // Gain de leeway (dérive sous le vent), ILLUSTRATIF — structure et valeur
    // salvaged from the June 2026 model, voir docs/SIMULATION_PHYSICS_REVIEW.md
    // Appendix (LEEWAY_GAIN=0.040 y est la référence). Défaut = 0 : aucun effet.
    double leewayGain = 0.0;
public:
    /** Décalage constant du safran (°) ajouté à la commande : modélise un défaut d'étalonnage. */
    void setRudderBias(double deg) { rudderBiasDeg = deg; }
    double rudderBias() const { return rudderBiasDeg; }

    /**
     * Courant marin : speedMs (m/s, >=0) et dirDeg = direction VERS LAQUELLE le
     * courant PORTE (voir commentaire currentDirDeg ci-dessus). Défaut (jamais
     * appelé) = 0 m/s ⇒ pas de courant, régression des 6 scénarios inchangée.
     */
    void setCurrent(double dirDeg, double speedMs) {
        currentDirDeg = dirDeg;
        currentSpeedMs = speedMs < 0 ? 0 : speedMs;
    }
    double currentDir() const { return currentDirDeg; }
    double currentSpeed() const { return currentSpeedMs; }

    /**
     * Gain de leeway (dérive sous le vent), ILLUSTRATIF — voir updateBoatDynamics()
     * et docs/SIMULATION_PHYSICS_REVIEW.md Appendix pour la forme et l'origine de
     * la valeur de référence (0.040). Défaut (jamais appelé) = 0 ⇒ pas de dérive.
     */
    void setLeewayGain(double gain) { leewayGain = gain; }
    double leewayGainValue() const { return leewayGain; }
private:
    std::vector<SimBoatState> history;  // Historique pour analyse
    
    /**
     * Calcule l'angle de cap en fonction des servos et du vent
     * Simule le comportement hydrodynamique du bateau
     */
    void updateBoatDynamics(float sailAngle, float rudderAngle, unsigned long dt_ms);
    
    /**
     * Convertit position GPS en mètres pour les calculs physiques
     */
    static void gpsToMeters(double lat, double lng, double refLat, double refLng,
                            double &x, double &y);
    
    /**
     * Convertit mètres en position GPS
     */
    static void metersToGps(double x, double y, double refLat, double refLng,
                            double &lat, double &lng);
};

#endif // SIM_ENVIRONMENT_HPP
