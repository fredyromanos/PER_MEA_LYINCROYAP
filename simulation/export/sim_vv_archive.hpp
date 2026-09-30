#ifndef SIM_VV_ARCHIVE_HPP
#define SIM_VV_ARCHIVE_HPP

#include "sim_html_export.hpp"
#include <vector>
#include <string>

class VVArchive {
public:
    /**
     * Génère un dossier horodaté contenant la carte HTML et le CSV de télémétrie.
     */
    static void generateArchive(const std::vector<ScenarioData>& scenarios);
};

#endif // SIM_VV_ARCHIVE_HPP