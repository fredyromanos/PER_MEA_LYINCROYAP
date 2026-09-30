#include "sim_vv_archive.hpp"
#include <filesystem>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <fstream>
#include <iostream>

void VVArchive::generateArchive(const std::vector<ScenarioData>& scenarios) {
    if (scenarios.empty()) {
        std::cerr << "[V&V] Aucun scénario à archiver." << std::endl;
        return;
    }

    // 1. Création de l'horodatage
    auto now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    std::tm* local_time = std::localtime(&now_time);
    char timeBuffer[80];
    std::strftime(timeBuffer, sizeof(timeBuffer), "%Y%m%d_%H%M%S", local_time);

    // 2. Création du répertoire d'export (ex: output/run_20260923_143000)
    std::string dirPath = std::string("output/run_") + timeBuffer;
    std::filesystem::create_directories(dirPath);

    std::string htmlFile = dirPath + "/simulation.html";
    std::string csvFile = dirPath + "/telemetrie_vv.csv";

    // 3. Export de la carte HTML
    HTMLExporter::exportAllScenarios(htmlFile, scenarios);

    // 4. Export du CSV pour analyse sur Excel/MATLAB
    std::ofstream csv(csvFile);
    csv << "Scenario,Temps_s,Latitude,Longitude,Cap,Vitesse,Voile,Safran,VentDir,VentVit\n";
    for (const auto& sc : scenarios) {
        for (const auto& h : sc.history) {
            csv << sc.name << "," << (h.time / 1000.0) << "," 
                << std::fixed << std::setprecision(6) << h.latitude << "," << h.longitude << "," 
                << std::setprecision(2) << h.heading << "," << h.speed << "," 
                << h.sailAngle << "," << h.physicalRudderAngle << "," 
                << h.windDirection << "," << h.windSpeed << "\n";
        }
    }
    csv.close();

    std::cout << "\n[V&V] Archive historique sauvegardée avec succès !" << std::endl;
    std::cout << "      Dossier : " << dirPath << std::endl;
    std::cout << "      -> HTML interactif : " << htmlFile << std::endl;
    std::cout << "      -> Matrice CSV       : " << csvFile << std::endl;
}