#include "export/sim_html_export.hpp"
#include "mocks/Arduino.h"
#include "sim_boat.hpp"
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>

// ── Helpers ──

// ── Optional overrides from the environment (all unset = historical behaviour) ──
//   SIM_WIND_DIR     true wind direction, degrees, direction it comes FROM
//   SIM_WIND_SPEED   true wind speed, m/s
//   SIM_LEEWAY_GAIN  leeway gain (0 = off; 0.040 = reference value from
//                    docs/SIMULATION_PHYSICS_REVIEW.md, illustrative)
// `make run-windy` fills the two wind variables from a real Windy forecast via
// fetch_wind.py. When set they replace the scenario's own wind, both for the physics
// and for the wind the boat "knows" (as if the operator sent the forecast with
// wind-command).
static bool envDouble(const char* name, double& out) {
  const char* v = std::getenv(name);
  if (!v || !*v) return false;
  char* end = nullptr;
  double d = std::strtod(v, &end);
  if (end == v || *end != '\0') return false;
  out = d;
  return true;
}
static double windDirOr(double scenarioDir) {
  double d;
  return envDouble("SIM_WIND_DIR", d) ? std::fmod(std::fmod(d, 360.0) + 360.0, 360.0) : scenarioDir;
}
static double windSpeedOr(double scenarioSpeed) {
  double d;
  return (envDouble("SIM_WIND_SPEED", d) && d >= 0.0) ? d : scenarioSpeed;
}
static double leewayGainFromEnv() {
  double d;
  return envDouble("SIM_LEEWAY_GAIN", d) ? d : 0.0;
}

// Per-scenario PASS/FAIL outcome + time-to-converge, filled in by each runScenarioN()
// alongside the ScenarioData used for the HTML export — see main()'s SUMMARY table and
// exit code. This is what turns the twin into an automated regression gate instead of a
// demo a human has to eyeball ("All waypoints reached!" in the console).
struct ScenarioResult {
  std::string name;
  SimulatedBoat::RunOutcome outcome = SimulatedBoat::RunOutcome::NotRun;
  double timeS = 0.0;
};

ScenarioData runScenario1(double currentSpeedMs, double currentDirDeg, ScenarioResult& result) {
  std::cout << "\n========== SCENARIO 1: Simple  ==========" << std::endl;
  SimTime::init();

  SimulatedBoat boat;
  boat.init(48.340, -4.520, windDirOr(0), windSpeedOr(4.0), 90);
  boat.setCurrent(currentDirDeg, currentSpeedMs);
  boat.setLeewayGain(leewayGainFromEnv());
  boat.addWaypoint(48.34, -4.510);
  boat.startWindObservation();
  boat.runSimulation(60000, 100);
  boat.setWindDirection(windDirOr(0));
  boat.startNavigation();
  boat.runSimulation(6400000, 100);
  result = {"S1 SIMPLE", boat.outcome(), boat.convergeTimeS()};

  return {"S1 SIMPLE", boat.getHistory(), boat.getWaypointPairs(),
          boat.getInitialWindDir(), boat.getWindSpeed()};
}
ScenarioData runScenario2(double currentSpeedMs, double currentDirDeg, ScenarioResult& result) {
  std::cout << "\n========== SCENARIO 2: Navigation VDB  =========="
            << std::endl;
  SimTime::init();

  SimulatedBoat boat;
  boat.init(48.340, -4.520, windDirOr(90), windSpeedOr(4.0), 45);
  boat.setCurrent(currentDirDeg, currentSpeedMs);
  boat.setLeewayGain(leewayGainFromEnv());
  boat.addWaypoint(48.34, -4.510);
  boat.startWindObservation();
  boat.runSimulation(0, 100);
  boat.setWindDirection(windDirOr(90));
  boat.startNavigation();
  boat.runSimulation(6400000, 100);
  result = {"S2 VDB", boat.outcome(), boat.convergeTimeS()};

  return {"S2 VDB", boat.getHistory(), boat.getWaypointPairs(),
          boat.getInitialWindDir(), boat.getWindSpeed()};
}

ScenarioData runScenario3(double currentSpeedMs, double currentDirDeg, ScenarioResult& result) {
  std::cout << "\n========== SCENARIO 3: Navigation LOFER  =========="
            << std::endl;
  SimTime::init();

  SimulatedBoat boat;
  boat.init(48.340, -4.520, windDirOr(50.00), windSpeedOr(4.0), 0);
  boat.setCurrent(currentDirDeg, currentSpeedMs);
  boat.setLeewayGain(leewayGainFromEnv());
  boat.addWaypoint(48.3405, -4.5280);
  boat.startWindObservation();
  boat.runSimulation(1, 100);
  boat.setWindDirection(windDirOr(220));
  boat.startNavigation();
  boat.runSimulation(6400000, 100);
  result = {"S3 Lofer", boat.outcome(), boat.convergeTimeS()};

  return {"S3 Lofer", boat.getHistory(), boat.getWaypointPairs(),
          boat.getInitialWindDir(), boat.getWindSpeed()};
}

ScenarioData runScenario4(double currentSpeedMs, double currentDirDeg, ScenarioResult& result) {
  std::cout << "\n========== SCENARIO 4: Navigation ABATTRE  "
               "=========="
            << std::endl;
  SimTime::init();

  SimulatedBoat boat;
  boat.init(48.340, -4.520, windDirOr(90), windSpeedOr(4.0), 315);
  boat.setCurrent(currentDirDeg, currentSpeedMs);
  boat.setLeewayGain(leewayGainFromEnv());
  boat.addWaypoint(48.349, -4.528);
  boat.startWindObservation();
  boat.runSimulation(60000, 100);
  boat.setWindDirection(windDirOr(90));
  boat.startNavigation();
  boat.runSimulation(9900000, 100);
  result = {"S4 Abattre", boat.outcome(), boat.convergeTimeS()};

  return {"S4 Abattre", boat.getHistory(), boat.getWaypointPairs(),
          boat.getInitialWindDir(), boat.getWindSpeed()};
}

ScenarioData runScenario5(double currentSpeedMs, double currentDirDeg, ScenarioResult& result) {
  std::cout << "\n========== SCENARIO 5: Test Reel 2 WPT  =========="
            << std::endl;
  SimTime::init();

  SimulatedBoat boat;
  boat.init(48.340, -4.520, windDirOr(315), windSpeedOr(3.0), 225);
  boat.setCurrent(currentDirDeg, currentSpeedMs);
  boat.setLeewayGain(leewayGainFromEnv());
  boat.addWaypoint(48.3350, -4.5280);
  boat.addWaypoint(48.3440, -4.5150);
  boat.startWindObservation();
  boat.runSimulation(60000, 100);
  boat.setWindDirection(windDirOr(315));
  boat.startNavigation();
  boat.runSimulation(6400000, 100);
  result = {"S5 Complex", boat.outcome(), boat.convergeTimeS()};

  return {"S5 Complex", boat.getHistory(), boat.getWaypointPairs(),
          boat.getInitialWindDir(), boat.getWindSpeed()};
}

ScenarioData runScenario6(double currentSpeedMs, double currentDirDeg, ScenarioResult& result) {
  std::cout << "\n========== SCENARIO 6: Navigation DOWNWIND  =========="
            << std::endl;
  SimTime::init();

  SimulatedBoat boat;
  boat.init(48.340, -4.520, windDirOr(90), windSpeedOr(4.0), 45);
  boat.setCurrent(currentDirDeg, currentSpeedMs);
  boat.setLeewayGain(leewayGainFromEnv());
  boat.addWaypoint(48.34, -4.51);
  boat.startWindObservation();
  boat.runSimulation(0, 100);
  boat.setWindDirection(windDirOr(270));
  boat.startNavigation();
  boat.runSimulation(6400000, 100);
  result = {"S6 Downwind", boat.outcome(), boat.convergeTimeS()};

  return {"S6 Downwind", boat.getHistory(), boat.getWaypointPairs(),
          boat.getInitialWindDir(), boat.getWindSpeed()};
}

int main(int argc, char *argv[]) {
  // argv[1] sélectionne le scénario (1-6) ; absent ou 0 = tous. Utilisé par
  // `make run` (scénario 1) et `make run-all` (une invocation par scénario).
  // argv[2]/argv[3] (optionnels) activent un COURANT uniforme sur le(s) scénario(s)
  // choisi(s) — voir Task 2 : currentSpeedMs (m/s, défaut 0 = pas de courant) et
  // currentDirDeg (direction VERS LAQUELLE le courant PORTE, défaut 90 = vers l'Est,
  // ignoré si currentSpeedMs=0). Absents ⇒ comportement historique inchangé.
  int selected = 0;
  double currentSpeedMs = 0.0;
  double currentDirDeg = 90.0;
  if (argc > 1) {
    char *end = nullptr;
    long v = std::strtol(argv[1], &end, 10);
    if (end == argv[1] || *end != '\0' || v < 0 || v > 6) {
      std::cout << "Usage: " << argv[0]
                << " [scenario] [currentSpeedMs] [currentDirDeg]\n"
                << "  scenario       : 1-6, or 0/absent for all\n"
                << "  currentSpeedMs : m/s of tidal current (default 0 = none)\n"
                << "  currentDirDeg  : direction the current flows TOWARD, \"set\"\n"
                << "                   convention (default 90 = toward East)"
                << std::endl;
      return 1;
    }
    selected = (int)v;
  }
  if (argc > 2) currentSpeedMs = std::strtod(argv[2], nullptr);
  if (argc > 3) currentDirDeg = std::strtod(argv[3], nullptr);

  std::cout << "╔═══════════════════════════════════════╗" << std::endl;
  std::cout << "║     AutoBoat Simulation System        ║" << std::endl;
  std::cout << "║     Integrated Physics + Navigation   ║" << std::endl;
  std::cout << "╚═══════════════════════════════════════╝" << std::endl;
  {
    double d;
    if (envDouble("SIM_WIND_DIR", d))
      std::cout << "  Wind override: from " << windDirOr(0) << " deg at "
                << windSpeedOr(0) << " m/s (SIM_WIND_DIR/SIM_WIND_SPEED)" << std::endl;
    if (leewayGainFromEnv() != 0.0)
      std::cout << "  Leeway gain: " << leewayGainFromEnv() << std::endl;
  }
  if (currentSpeedMs != 0.0) {
    std::cout << "  Current: " << currentSpeedMs << " m/s toward " << currentDirDeg
              << " deg" << std::endl;
  }

  std::vector<ScenarioData> allScenarios;
  std::vector<ScenarioResult> results;

  { ScenarioResult r; if (selected == 0 || selected == 1) { allScenarios.push_back(runScenario1(currentSpeedMs, currentDirDeg, r)); results.push_back(r); } }
  { ScenarioResult r; if (selected == 0 || selected == 2) { allScenarios.push_back(runScenario2(currentSpeedMs, currentDirDeg, r)); results.push_back(r); } }
  { ScenarioResult r; if (selected == 0 || selected == 3) { allScenarios.push_back(runScenario3(currentSpeedMs, currentDirDeg, r)); results.push_back(r); } }
  { ScenarioResult r; if (selected == 0 || selected == 4) { allScenarios.push_back(runScenario4(currentSpeedMs, currentDirDeg, r)); results.push_back(r); } }
  { ScenarioResult r; if (selected == 0 || selected == 5) { allScenarios.push_back(runScenario5(currentSpeedMs, currentDirDeg, r)); results.push_back(r); } }
  { ScenarioResult r; if (selected == 0 || selected == 6) { allScenarios.push_back(runScenario6(currentSpeedMs, currentDirDeg, r)); results.push_back(r); } }

  // Export HTML unique avec tous les scénarios
  HTMLExporter::exportAllScenarios("output/simulation.html", allScenarios);

  std::cout << "\n✓ Simulation complete!" << std::endl;
  std::cout
      << "  Open output/simulation.html in a browser to view all scenarios."
      << std::endl;

  // === PASS/FAIL summary — turns the twin into an automated regression gate instead of
  // a demo a human has to eyeball for "All waypoints reached!" in the console log. ===
  std::cout << "\n======================================================" << std::endl;
  std::cout << "  SUMMARY" << std::endl;
  std::cout << "======================================================" << std::endl;
  int passed = 0;
  for (const auto& r : results) {
    const char* label =
        (r.outcome == SimulatedBoat::RunOutcome::Converged) ? "PASS"
      : (r.outcome == SimulatedBoat::RunOutcome::Stuck)     ? "FAIL (stuck)"
      : (r.outcome == SimulatedBoat::RunOutcome::TimedOut)  ? "FAIL (timeout)"
                                                             : "FAIL (not run)";
    if (r.outcome == SimulatedBoat::RunOutcome::Converged) passed++;
    std::cout << "  " << std::left << std::setw(14) << r.name << std::setw(16) << label
              << "time=" << std::fixed << std::setprecision(0) << r.timeS << "s" << std::endl;
  }
  std::cout << "\nSUMMARY: " << passed << "/" << results.size() << " passed" << std::endl;

  // Non-zero exit on any failure: makes `./boat_simulator` usable as a CI gate, not just
  // a demo (see docs/ADVERSARIAL_FINDINGS.md for the equivalent gate on the adversarial side).
  if (passed != (int)results.size()) {
    std::cout << "\n!! " << (results.size() - passed) << "/" << results.size()
              << " scenario(s) FAILED\n" << std::endl;
    return 1;
  }

  return 0;
}
