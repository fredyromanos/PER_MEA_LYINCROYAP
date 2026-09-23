// ─────────────────────────────────────────────────────────────────────────────
// simu_gps — IHM digital twin with the GPS-only heading logic.
//
// Unlike simu_boat (random drift, fixed heading), this runs:
//   • the boat physics of Simulation/simulation (sail polar, rudder, propeller)
//   • a realistic NEO-6M model: 1 Hz, ±3 m / ±5°, no course below 1 km/h,
//     optional periodic fix loss
//   • the REAL firmware HeadingGate + navigation.h
//   • the REAL firmware heartbeat formatter (comm/Heartbeat.h) → byte-identical telemetry
// and accepts the IHM commands with the same keys as LoRaComm::dispatch
// (waypoints, wind-command, navigate, stop, wind-observation).
//
// Usage:  simu_gps [--port /tmp/ttySimu] [--speedup 5] [--dropout 90:15]
//                  [--wind 270] [--wind-speed 4] [--lat 48.340] [--lon -4.520]
//                  [--heading 0]
// ─────────────────────────────────────────────────────────────────────────────
#include "sim_boat.hpp"
#include "comm/Heartbeat.h"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <string>
#include <termios.h>
#include <unistd.h>

static volatile std::sig_atomic_t g_stop = 0;
static void onSignal(int) { g_stop = 1; }

struct Options {
    const char*   port      = "/tmp/ttySimu";
    unsigned      speedup   = 5;
    unsigned long dropPeriodS = 0, dropDurationS = 0;
    double        wind      = 270.0;
    double        windSpeed = 4.0;
    // Start ON THE WATER (Rade de Brest), same point as the static simulation scenarios.
    // The former default (48.360687, -4.565710) is on land, at the Technopôle.
    double        lat       = 48.340;
    double        lon       = -4.520;
    double        heading   = 0.0;
};

static Options parseArgs(int argc, char** argv) {
    Options o;
    for (int i = 1; i + 1 < argc; i += 2) {
        std::string k = argv[i];
        const char* v = argv[i + 1];
        if (k == "--port") o.port = v;
        else if (k == "--speedup") o.speedup = (unsigned)std::max(1, std::atoi(v));
        else if (k == "--dropout") std::sscanf(v, "%lu:%lu", &o.dropPeriodS, &o.dropDurationS);
        else if (k == "--wind") o.wind = std::atof(v);
        else if (k == "--wind-speed") o.windSpeed = std::atof(v);
        else if (k == "--lat") o.lat = std::atof(v);
        else if (k == "--lon") o.lon = std::atof(v);
        else if (k == "--heading") o.heading = std::atof(v);
        else std::fprintf(stderr, "unknown option %s\n", k.c_str());
    }
    return o;
}

static int openPort(const char* port) {
    int fd = open(port, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) return -1;
    termios t{};
    if (tcgetattr(fd, &t) == 0) {
        cfmakeraw(&t);
        tcsetattr(fd, TCSANOW, &t);
    }
    return fd;
}

// serial_link may send the JSON as an escaped string ("{\"origin\":...}"): normalise.
static std::string unescape(const std::string& in) {
    std::string out;
    out.reserve(in.size());
    for (size_t i = 0; i < in.size(); ++i) {
        if (in[i] == '\\' && i + 1 < in.size() && in[i + 1] == '"') { out += '"'; ++i; }
        else out += in[i];
    }
    return out;
}

struct Twin {
    SimulatedBoat boat;
    bool windKnown       = false;
    bool navigatePending = false;

    // Same keys and precedence as LoRaComm::dispatch.
    void handle(const std::string& raw) {
        const std::string line = unescape(raw);
        const char* json = line.c_str();
        if (!std::strstr(json, "\"origin\":\"server\"") || !std::strstr(json, "\"type\":\"command\"")) return;
        const char* msg = std::strstr(json, "\"message\":");
        if (!msg) return;

        if (std::strstr(msg, "\"waypoints\"")) {
            const char* p = std::strstr(msg, "\"points\":\"");
            if (!p) return;
            p += 10;
            boat.clearWaypoints();
            while (*p && *p != '"') {
                char* end = nullptr;
                double la = std::strtod(p, &end);
                if (end == p || *end != ',') break;
                p = end + 1;
                double lo = std::strtod(p, &end);
                if (end == p) break;
                boat.addWaypoint(la, lo);
                p = end;
                if (*p == ',') ++p;
            }
            std::printf("[CMD] waypoints: %zu\n", boat.waypointCount());
        } else if (std::strstr(msg, "\"wind-command\"")) {
            const char* v = std::strstr(msg, "\"value\":");
            if (!v) return;
            boat.setKnownWind(std::atof(v + 8));
            windKnown = true;
            std::printf("[CMD] wind-command %.0f\n", boat.knownWindDirection());
        } else if (std::strstr(msg, "\"navigate\"")) {
            navigatePending = true;
            std::printf("[CMD] navigate%s\n", windKnown ? "" : " (waiting for wind, as firmware)");
        } else if (std::strstr(msg, "\"stop\"")) {
            navigatePending = false;
            if (boat.mode() == "navigate") boat.stopNavigation();
            boat.setBoatMode("setup-ready");
            std::printf("[CMD] stop\n");
        } else if (std::strstr(msg, "\"wind-observation\"")) {
            boat.setBoatMode("setup-ready");
            boat.startWindObservation();
            std::printf("[CMD] wind-observation\n");
        } else if (std::strstr(msg, "\"home\"") || std::strstr(msg, "\"restart\"")) {
            std::printf("[CMD] ignored in simulation: %.60s\n", msg);
        }
    }

    // Firmware rule: a started mission only steers once the wind is known.
    void tryStartNavigation() {
        if (boat.mode() == "wind-ready" && !windKnown) windKnown = true;   // from wind observation
        if (!navigatePending || !windKnown || boat.waypointCount() == 0) return;
        if (boat.mode() == "navigate") { navigatePending = false; return; }
        boat.setBoatMode("wind-ready");
        boat.startNavigation();
        navigatePending = false;
    }

    std::string heartbeat() const {
        const GpsPosition&        gps = boat.gpsPosition();
        const HeadingGate::State& hs  = boat.headingState();
        const bool navigating = boat.mode() == "navigate";
        const unsigned long nowMs = boat.getState().time;

        HeartbeatFields f;
        f.mode      = (navigating || navigatePending) ? "navigate" : "route-ready";
        f.lat       = gps.lat;
        f.lon       = gps.lon;
        f.sailDeg   = boat.sailCommand() >= 0 ? 10 : -10;
        f.rudderDeg = navigating ? (int)std::lround(std::max(-110.0f, std::min(110.0f, boat.rudderCommand()))) : 0;
        f.headingSrc = navigating ? (uint8_t)hs.source : (gps.courseValid ? (uint8_t)HeadingGate::Gps : 0);
        f.heading   = (navigating && hs.valid) ? hs.deg : gps.courseDeg;
        f.fixAgeS   = gps.valid ? 0 : (uint8_t)std::min<unsigned long>(99, (nowMs - boat.lastFixMs()) / 1000);
        f.windDeg   = windKnown ? (float)boat.knownWindDirection() : 0.0f;
        f.batVolts  = 7.80f;
        f.gpsFix    = gps.valid;
        f.sats      = gps.satellites;
        f.hdop      = gps.hdop;
        f.rcOk      = true;               // no RC in simulation
        f.wptTotal  = (uint8_t)boat.waypointCount();
        f.wptCur    = (uint8_t)boat.waypointIndex();

        char buf[320];
        int n = formatHeartbeat(buf, sizeof buf, f);
        return (n > 0 && (size_t)n <= HEARTBEAT_LORA_MAX_BYTES) ? std::string(buf) : std::string();
    }
};

int main(int argc, char** argv) {
    const Options opt = parseArgs(argc, argv);
    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);
    setvbuf(stdout, nullptr, _IOLBF, 0);

    int fd = -1;
    for (int i = 0; i < 50 && fd < 0; ++i) {        // socat may still be creating the PTY
        fd = openPort(opt.port);
        if (fd < 0) usleep(100000);
    }
    if (fd < 0) {
        std::fprintf(stderr, "cannot open %s: %s\n", opt.port, std::strerror(errno));
        return 1;
    }

    Twin twin;
    twin.boat.init(opt.lat, opt.lon, opt.wind, opt.windSpeed, opt.heading);
    if (opt.dropPeriodS > 0) twin.boat.setGpsDropout(opt.dropPeriodS, opt.dropDurationS);
    std::printf("[SIMU_GPS] port=%s speedup=x%u wind=%.0f/%.1fm/s dropout=%lu:%lu\n",
                opt.port, opt.speedup, opt.wind, opt.windSpeed, opt.dropPeriodS, opt.dropDurationS);

    std::string rx;
    unsigned tick = 0;
    while (!g_stop) {
        // Commands from the IHM (non-blocking).
        char buf[512];
        ssize_t n;
        // The virtual port can disappear (socat restarted, Start / Réinit coms): never exit,
        // reopen it and keep simulating so the boat state survives.
        if (fd < 0) {
            fd = openPort(opt.port);
            if (fd >= 0) std::printf("[SIMU_GPS] port %s reopened\n", opt.port);
        }
        while (fd >= 0 && (n = read(fd, buf, sizeof buf)) > 0) {
            rx.append(buf, (size_t)n);
            size_t nl;
            while ((nl = rx.find('\n')) != std::string::npos) {
                twin.handle(rx.substr(0, nl));
                rx.erase(0, nl + 1);
            }
            if (rx.size() > 4096) rx.clear();
        }

        // Simulated time: speedup × 100 ms per 100 ms of wall time.
        for (unsigned i = 0; i < opt.speedup; ++i) {
            twin.tryStartNavigation();
            const std::string& m = twin.boat.mode();
            if (m != "navigate" && m != "wind-observation") twin.boat.holdNeutral();
            twin.boat.stepSimulation(100);
        }

        // Telemetry once per wall-clock second, like the 1 Hz firmware heartbeat.
        if (++tick % 10 == 0) {
            const std::string hb = twin.heartbeat();
            if (!hb.empty()) {
                std::string line = hb + "\n";
                if (fd >= 0 && write(fd, line.data(), line.size()) < 0 && errno != EAGAIN) {
                    std::printf("[SIMU_GPS] write failed (%s): reopening port\n", std::strerror(errno));
                    close(fd);
                    fd = -1;
                }
                const auto& s = twin.boat.getState();
                std::printf("[t=%5lus] %-15s nav=%-15s hv=%u fix=%d spd=%.2fm/s wpt=%d/%zu\n",
                            s.time / 1000, twin.boat.mode().c_str(), twin.boat.navState(),
                            (unsigned)twin.boat.headingState().source,
                            twin.boat.gpsPosition().valid ? 1 : 0, s.speed,
                            twin.boat.waypointIndex(), twin.boat.waypointCount());
            }
        }
        usleep(100000);
    }
    if (fd >= 0) close(fd);
    return 0;
}
