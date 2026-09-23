#include "export/sim_html_export.hpp"
#include "mocks/Arduino.h"
#include "sim_boat.hpp"
#include <cmath>
#include <iostream>

// ── Helpers ──

ScenarioData runScenario1() {
  std::cout << "\n========== SCENARIO 1: Simple  ==========" << std::endl;
  SimTime::init();

  SimulatedBoat boat;
  boat.init(48.340, -4.520, 0, 4.0, 90);
  boat.addWaypoint(48.34, -4.510);
  boat.startWindObservation();
  boat.runSimulation(60000, 100);
  boat.setWindDirection(0);
  boat.startNavigation();
  boat.runSimulation(6400000, 100);

  return {"S1 SIMPLE", boat.getHistory(), boat.getWaypointPairs(),
          boat.getInitialWindDir(), boat.getWindSpeed()};
}
ScenarioData runScenario2() {
  std::cout << "\n========== SCENARIO 2: Navigation VDB  =========="
            << std::endl;
  SimTime::init();

  SimulatedBoat boat;
  boat.init(48.340, -4.520, 90, 4.0, 45);
  boat.addWaypoint(48.34, -4.510);
  boat.startWindObservation();
  boat.runSimulation(0, 100);
  boat.setWindDirection(90);
  boat.startNavigation();
  boat.runSimulation(6400000, 100);

  return {"S2 VDB", boat.getHistory(), boat.getWaypointPairs(),
          boat.getInitialWindDir(), boat.getWindSpeed()};
}

ScenarioData runScenario3() {
  std::cout << "\n========== SCENARIO 3: Navigation LOFER  =========="
            << std::endl;
  SimTime::init();

  SimulatedBoat boat;
  boat.init(48.340, -4.520, 50.00, 4.0, 0);
  boat.addWaypoint(48.3405, -4.5280);
  boat.startWindObservation();
  boat.runSimulation(1, 100);
  boat.setWindDirection(220);
  boat.startNavigation();
  boat.runSimulation(6400000, 100);

  return {"S3 Lofer", boat.getHistory(), boat.getWaypointPairs(),
          boat.getInitialWindDir(), boat.getWindSpeed()};
}

ScenarioData runScenario4() {
  std::cout << "\n========== SCENARIO 4: Navigation ABATTRE  "
               "=========="
            << std::endl;
  SimTime::init();

  SimulatedBoat boat;
  boat.init(48.340, -4.520, 90, 4.0, 315);
  boat.addWaypoint(48.349, -4.528);
  boat.startWindObservation();
  boat.runSimulation(60000, 100);
  boat.setWindDirection(90);
  boat.startNavigation();
  boat.runSimulation(9900000, 100);

  return {"S4 Abattre", boat.getHistory(), boat.getWaypointPairs(),
          boat.getInitialWindDir(), boat.getWindSpeed()};
}

ScenarioData runScenario5() {
  std::cout << "\n========== SCENARIO 5: Test Reel 2 WPT  =========="
            << std::endl;
  SimTime::init();

  SimulatedBoat boat;
  boat.init(48.340, -4.520, 315, 3.0, 225);
  boat.addWaypoint(48.3350, -4.5280);
  boat.addWaypoint(48.3440, -4.5150);
  boat.startWindObservation();
  boat.runSimulation(60000, 100);
  boat.setWindDirection(315);
  boat.startNavigation();
  boat.runSimulation(6400000, 100);

  return {"S5 Complex", boat.getHistory(), boat.getWaypointPairs(),
          boat.getInitialWindDir(), boat.getWindSpeed()};
}

ScenarioData runScenario6() {
  std::cout << "\n========== SCENARIO 6: Navigation DOWNWIND  =========="
            << std::endl;
  SimTime::init();

  SimulatedBoat boat;
  boat.init(48.340, -4.520, 90, 4.0, 45);
  boat.addWaypoint(48.34, -4.51);
  boat.startWindObservation();
  boat.runSimulation(0, 100);
  boat.setWindDirection(270);
  boat.startNavigation();
  boat.runSimulation(6400000, 100);

  return {"S6 Downwind", boat.getHistory(), boat.getWaypointPairs(),
          boat.getInitialWindDir(), boat.getWindSpeed()};
}

int main(int argc, char *argv[]) {
  std::cout << "╔═══════════════════════════════════════╗" << std::endl;
  std::cout << "║     AutoBoat Simulation System        ║" << std::endl;
  std::cout << "║     Integrated Physics + Navigation   ║" << std::endl;
  std::cout << "╚═══════════════════════════════════════╝" << std::endl;

  std::vector<ScenarioData> allScenarios;

  allScenarios.push_back(runScenario1());
  allScenarios.push_back(runScenario2());
  allScenarios.push_back(runScenario3());
  allScenarios.push_back(runScenario4());
  allScenarios.push_back(runScenario5());
  allScenarios.push_back(runScenario6());

  if (allScenarios.empty()) {
    std::cout << "Scenario "
              << " not implemented. Available: 1-6 (or 0/none for all)"
              << std::endl;
    return 1;
  }

  // Export HTML unique avec tous les scénarios
  HTMLExporter::exportAllScenarios("output/simulation.html", allScenarios);

  std::cout << "\n✓ Simulation complete!" << std::endl;
  std::cout
      << "  Open output/simulation.html in a browser to view all scenarios."
      << std::endl;

  return 0;
}
