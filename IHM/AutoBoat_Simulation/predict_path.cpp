// ─────────────────────────────────────────────────────────────────────────────
// predict_path — trajectory prediction for the IHM map ("🧭 Trajectoire").
//
// Runs exactly what Simulation/simulation runs for the static HTML report:
// the REAL firmware navigation.h, the same boat physics and the same steering
// conversion (nav_rudderCorrection), from the boat's current state, with an
// exact heading. Replaces the JavaScript re-implementation (nav.js) whose logic
// and kinematics had drifted from the firmware.
//
// Usage:
//   predict_path --lat 48.36 --lon -4.56 --heading 41 --wind 270 [--wind-speed 4]
//                --wp 48.3630,-4.5640 [--wp ...] [--max-s 1800]
// Output (stdout, one line):
//   {"path":[[lat,lon],...],"reached":true|false,"duration_s":N,"waypoints_reached":K}
// ─────────────────────────────────────────────────────────────────────────────
#include "sim_boat.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

int main(int argc, char** argv) {
    // Default speed: the IHM only asks for a prediction when the heading is valid,
    // i.e. the boat already moves (>= HEADING_SPEED_ON_KMPH). 1 m/s ≈ 3.6 km/h.
    double lat = 0, lon = 0, heading = 0, wind = 0, windSpeed = 4.0, speed = 1.0;
    unsigned long maxS = 1800;
    bool haveLat = false, haveLon = false, haveWind = false;
    std::vector<std::pair<double, double>> wps;

    for (int i = 1; i + 1 < argc; i += 2) {
        std::string k = argv[i];
        const char* v = argv[i + 1];
        if (k == "--lat") { lat = std::atof(v); haveLat = true; }
        else if (k == "--lon") { lon = std::atof(v); haveLon = true; }
        else if (k == "--heading") heading = std::atof(v);
        else if (k == "--wind") { wind = std::atof(v); haveWind = true; }
        else if (k == "--wind-speed") windSpeed = std::atof(v);
        else if (k == "--speed") speed = std::atof(v);
        else if (k == "--max-s") maxS = std::strtoul(v, nullptr, 10);
        else if (k == "--wp") {
            double a, b;
            if (std::sscanf(v, "%lf,%lf", &a, &b) == 2) wps.push_back({a, b});
        }
    }
    if (!haveLat || !haveLon || !haveWind || wps.empty()) {
        std::printf("{\"error\":\"need --lat --lon --wind and at least one --wp\"}\n");
        return 2;
    }

    // Prediction = what the navigation does with a correct heading (ideal GPS).
    setenv("SIM_IDEAL_GPS", "1", 1);
    std::streambuf* saved = std::cout.rdbuf(nullptr);   // simulator logs are not JSON

    SimulatedBoat boat;
    boat.init(lat, lon, wind, windSpeed, heading);
    for (const auto& w : wps) boat.addWaypoint(w.first, w.second);
    boat.setBoatMode("wind-ready");
    boat.startNavigation();
    boat.setSpeed(speed);

    std::string path = "[";
    char pt[64];
    std::snprintf(pt, sizeof pt, "[%.6f,%.6f]", lat, lon);
    path += pt;

    unsigned long t = 0;
    const unsigned long stepMs = 100, sampleMs = 2000;
    while (t < maxS * 1000UL && boat.mode() == "navigate") {
        boat.stepSimulation(stepMs);
        t += stepMs;
        if (t % sampleMs == 0) {
            const SimBoatState& s = boat.getState();
            std::snprintf(pt, sizeof pt, ",[%.6f,%.6f]", s.latitude, s.longitude);
            path += pt;
        }
    }
    const SimBoatState& s = boat.getState();
    std::snprintf(pt, sizeof pt, ",[%.6f,%.6f]", s.latitude, s.longitude);
    path += pt;
    path += "]";

    const bool reached = boat.mode() != "navigate";
    const int wpReached = reached ? (int)wps.size() : boat.waypointIndex();
    std::cout.rdbuf(saved);
    std::printf("{\"path\":%s,\"reached\":%s,\"duration_s\":%lu,\"waypoints_reached\":%d}\n",
                path.c_str(), reached ? "true" : "false", t / 1000, wpReached);
    return 0;
}
