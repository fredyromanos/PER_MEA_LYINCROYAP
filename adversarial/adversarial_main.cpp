// ─────────────────────────────────────────────────────────────────────────────
// Adversarial falsification campaign for the autonomous navigation.
//
//   ./adversarial            → baseline control + all sensitivity sweeps + CSV
//   ./adversarial --baseline → baseline control only (CI gate; exit!=0 if it fails)
//
// Baseline control: with EVERY disturbance = 0 (yawMismatch = 1), the boat must
// converge on the twin's scenarios. If it doesn't, the harness is rigged to fail.
// Sweeps then push each unknown and report the breaking frontier (no "pass").
// ─────────────────────────────────────────────────────────────────────────────
#include "metrics.hpp"
#include <cstdio>
#include <cstring>
#include <vector>
#include <string>
#include <functional>
#include <random>

static constexpr double MDLAT = 6371000.0 * M_PI / 180.0;  // NAV_EARTH_RADIUS_M·π/180

struct Mission {
    double startLat, startLng, heading, wptLat, wptLng, windTrue, windSpeed;
};

static Mission place(double lat, double lng, double hdg, double windTrue,
                     double windSpeed, double brgDeg, double distM) {
    double dLat = distM * std::cos(brgDeg * M_PI / 180.0) / MDLAT;
    double dLng = distM * std::sin(brgDeg * M_PI / 180.0) /
                  (MDLAT * std::cos(lat * M_PI / 180.0));
    return { lat, lng, hdg, lat + dLat, lng + dLng, windTrue, windSpeed };
}

// ── Baseline control: the six twin scenarios (effective wind after setWindDirection) ──
static std::vector<Mission> baselineScenarios() {
    const double LA = 48.340, LO = -4.520;
    std::vector<Mission> v;
    v.push_back({LA, LO, 90, 48.340, -4.510,   0, 4.0}); // S1 simple
    v.push_back({LA, LO, 45, 48.340, -4.510,  90, 4.0}); // S2 VDB
    v.push_back({LA, LO,  0, 48.3405,-4.5280, 220, 4.0}); // S3 lofer
    v.push_back({LA, LO,315, 48.349, -4.528,  90, 4.0}); // S4 abattre
    v.push_back({LA, LO,225, 48.3350,-4.5280,315, 3.0}); // S5 (first waypoint)
    v.push_back({LA, LO, 45, 48.340, -4.510, 270, 4.0}); // S6 downwind
    return v;
}

static int runBaseline() {
    printf("\n=== BASELINE CONTROL (all disturbances 0, yawMismatch=1) ===\n");
    printf("Proves the harness is not rigged to fail — must converge like the twin.\n\n");
    auto sc = baselineScenarios();
    int ok = 0;
    for (size_t i = 0; i < sc.size(); ++i) {
        BoatParams p; p.windTrue = sc[i].windTrue; p.windSpeed = sc[i].windSpeed;
        TrialResult r = runTrial(p, sc[i].startLat, sc[i].startLng, sc[i].heading,
                                 sc[i].wptLat, sc[i].wptLng, /*seed*/1);
        printf("  S%zu  %-9s  t=%5.0fs  pathRatio=%4.2f  tacks=%d  closest=%.0fm\n",
               i + 1, r.converged ? "REACHED" : (r.stuck ? "STUCK" : "TIMEOUT"),
               r.timeS, r.pathRatio, r.tacks, r.closestM);
        if (r.converged) ok++;
    }
    printf("\n  baseline: %d/%zu converged\n", ok, sc.size());
    return ok == (int)sc.size() ? 0 : 1;
}

// ── Monte-Carlo sweep of one parameter ──
static std::mt19937 masterRng(12345);

static void sweep(FILE* csv, const char* name, const std::vector<double>& vals,
                  std::function<void(BoatParams&, double, std::mt19937&)> setParam,
                  int N = 200) {
    printf("\n── SWEEP: %s  (%d random missions per value)\n", name, N);
    printf("   %10s | %%converged | med pathRatio | %%stuck | med tacks\n", name);
    for (double v : vals) {
        std::mt19937 rng(0xC0FFEE ^ (unsigned)(v * 1000) ^ std::hash<std::string>{}(name));
        int conv = 0, stuck = 0;
        std::vector<double> ratios; std::vector<int> tks;
        for (int i = 0; i < N; ++i) {
            double hdg = std::uniform_real_distribution<>(0, 360)(rng);
            double wind = std::uniform_real_distribution<>(0, 360)(rng);
            double brg = std::uniform_real_distribution<>(0, 360)(rng);
            double dist = std::uniform_real_distribution<>(300, 900)(rng);
            Mission m = place(48.34, -4.52, hdg, wind, 4.0, brg, dist);
            BoatParams p; p.windTrue = m.windTrue; p.windSpeed = m.windSpeed;
            setParam(p, v, rng);
            TrialResult r = runTrial(p, m.startLat, m.startLng, m.heading,
                                     m.wptLat, m.wptLng, /*seed*/(unsigned)(i + 1));
            if (r.converged) { conv++; ratios.push_back(r.pathRatio); tks.push_back(r.tacks); }
            if (r.stuck) stuck++;
        }
        auto med = [](std::vector<double> x){ if(x.empty())return 0.0; std::sort(x.begin(),x.end()); return x[x.size()/2]; };
        auto medi= [](std::vector<int> x){ if(x.empty())return 0; std::sort(x.begin(),x.end()); return x[x.size()/2]; };
        double pc = 100.0 * conv / N, ps = 100.0 * stuck / N;
        printf("   %10.3f |   %5.1f%%   |    %5.2f      | %5.1f%% |   %d\n",
               v, pc, med(ratios), ps, medi(tks));
        fprintf(csv, "%s,%.4f,%.1f,%.3f,%.1f,%d\n", name, v, pc, med(ratios), ps, medi(tks));
    }
}

int main(int argc, char** argv) {
    bool baselineOnly = (argc > 1 && std::strcmp(argv[1], "--baseline") == 0);

    int baseFail = runBaseline();
    if (baselineOnly) return baseFail;
    if (baseFail) { printf("\n!! baseline control FAILED — harness suspect, aborting sweeps\n"); return 1; }

    FILE* csv = std::fopen("adversarial.csv", "w");
    if (csv) fprintf(csv, "sweep,value,pct_converged,med_path_ratio,pct_stuck,med_tacks\n");

    // 1. Wind-estimate error (deg). No wind sensor on the boat → this WILL be nonzero.
    sweep(csv, "windErrDeg", {0,5,10,15,20,30,45},
          [](BoatParams& p, double v, std::mt19937&){ p.windEstErrorDeg = v; });

    // 2. Tidal current speed (m/s), random direction per trial. Coastal mission.
    sweep(csv, "currentMps", {0,0.25,0.5,1.0,1.5},
          [](BoatParams& p, double v, std::mt19937& r){
              p.currentSpeed = v; p.currentDir = std::uniform_real_distribution<>(0,360)(r); });

    // 3. Open-loop winch slew rate (deg/s). Lower = slower winch, no feedback.
    sweep(csv, "winchDegPerS", {1e9,200,100,50,20,10},
          [](BoatParams& p, double v, std::mt19937&){ p.winchRateDegPerS = v; });

    // 4. Sail-yaw compensation mismatch. 1.0 = twin's implicit assumption (perfect cancel).
    sweep(csv, "yawMismatch", {1.0,0.9,0.75,0.5,0.25,0.0,1.25,1.5},
          [](BoatParams& p, double v, std::mt19937&){ p.yawMismatch = v; });

    // 5. GPS noise (position m ; course deg scaled with it).
    sweep(csv, "gpsPosM", {0,1,3,5,8},
          [](BoatParams& p, double v, std::mt19937&){ p.gpsPosNoiseM = v; p.gpsCourseNoiseDeg = v; });

    if (csv) std::fclose(csv);
    printf("\nCSV written: adversarial.csv\n");
    printf("Interpretation & required hardware measurements: docs/ADVERSARIAL_FINDINGS.md\n");
    return 0;
}
