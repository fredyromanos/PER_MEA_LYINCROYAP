/**
 * Export HTML interactif multi-scénarios avec Leaflet.js
 * 
 * Génère UN SEUL fichier HTML contenant tous les scénarios :
 * - Onglets pour switcher entre scénarios
 * - Carte OpenStreetMap via Leaflet (CARTO Dark)
 * - Trajectoire colorée par mode (VDB/LOFER/ABATTRE/observation)
 * - Animation du bateau avec contrôles (play/pause/slider/vitesse)
 * - Flèche de vent, jauges voile/gouvernail, infos temps réel
 * - Marqueurs waypoints numérotés
 */

#ifndef SIM_HTML_EXPORT_HPP
#define SIM_HTML_EXPORT_HPP

#include "../sim_environment.hpp"
#include <string>
#include <vector>
#include <fstream>

/**
 * Données d'un scénario pour l'export HTML
 */
struct ScenarioData {
    std::string name;
    std::vector<SimBoatState> history;
    std::vector<std::pair<double, double>> waypoints;
    double windDir;
    double windSpeed;
};

class HTMLExporter {
public:
    /**
     * Exporte tous les scénarios dans un seul fichier HTML interactif
     * @param filename   Chemin du fichier HTML à générer
     * @param scenarios  Vecteur de ScenarioData (un par scénario)
     */
    static void exportAllScenarios(
        const std::string& filename,
        const std::vector<ScenarioData>& scenarios
    );

private:
    static void writeHeader(std::ofstream& f);
    static void writeStyles(std::ofstream& f);
    static void writeAllScenariosJSON(std::ofstream& f, const std::vector<ScenarioData>& scenarios);
    static void writeHTMLBody(std::ofstream& f, const std::vector<ScenarioData>& scenarios);
    static void writeJavaScript(std::ofstream& f);
    static void writeFooter(std::ofstream& f);
};

#endif // SIM_HTML_EXPORT_HPP
