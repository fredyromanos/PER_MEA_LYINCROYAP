#include "LoRaComm.h"
#include "../drivers/LoRaRadio.h"
#include "../app/DroneApp.h"
#include "../navigation/MissionPlan.h"
#include "../config/DebugConfig.h"
#include "../config/Calibration.h"
#include "Heartbeat.h"
#include <cmath>
#include <cstdlib>

void LoRaComm::begin(LoRaRadio& radio, DroneApp& app) {
    radio_ = &radio;
    app_   = &app;
    DBG_RADIO("begin OK");
}

void LoRaComm::update() {
    if (!radio_) return;
    if (!radio_->poll(rxBuf_, sizeof(rxBuf_))) return;
    lastRxRssi_ = radio_->rssi();
    DBG_RADIO("RX rssi=%d  %s", lastRxRssi_, rxBuf_);
    dispatch(rxBuf_);
}

void LoRaComm::sendHeartbeat(ControlMode mode, MissionState mState,
                              double lat, double lon, float heading,
                              uint8_t headingSrc, uint8_t fixAgeS,
                              float batVolts, uint8_t wptCur, uint8_t wptTotal,
                              float windDeg, uint16_t sailUs, uint16_t rotorUs,
                              bool gpsFix, uint8_t sats, float hdop, bool rcOk,
                              bool windObs, uint8_t windObsPct) {
    const char* modeStr;

    if (mode == ControlMode::Automatic || mode == ControlMode::Failsafe) {
        modeStr  = (mState == MissionState::Running || mState == MissionState::Returning)
                   ? "navigate" : "route-ready";
    } else {
        modeStr  = "standby";
    }

    // control_mode supprimé du heartbeat : 100 % redondant avec "mode"
    // (standby ⟺ radio, route-ready/navigate ⟺ autonomous). L'IHM le déduit.
    // Cela libère ~28 octets pour rester sous la limite LoRa de 255 octets.

    // Sail: binaire ±10° selon position par rapport au centre
    const int8_t sailDeg = (sailUs >= Calibration::SAIL_CENTER_US) ? 10 : -10;

    // Rotor: interpolation linéaire ROTOR_MIN/MAX (1417/1583 µs) → ±90°.
    // Même pente en auto, donc extrapole correctement jusqu'à ±110° (1399/1601 µs).
    const int16_t rotorDeg = (int16_t)(
        -Calibration::ROTOR_RANGE_DEG + (float)(rotorUs - Calibration::ROTOR_MIN_US)
        / (float)(Calibration::ROTOR_MAX_US - Calibration::ROTOR_MIN_US)
        * (2.0f * Calibration::ROTOR_RANGE_DEG)
    );

    // Réductions de taille : heading entier, location 5 décimales (~1.1 m),
    // waypoints aplati en "wt"/"wc", "wobs" seulement pendant la mesure.
    // Format et budget 255 octets : comm/Heartbeat.h (testé sur PC).
    HeartbeatFields f;
    f.mode = modeStr;           f.lat = lat;               f.lon = lon;
    f.sailDeg = sailDeg;        f.rudderDeg = rotorDeg;
    f.heading = heading;        f.headingSrc = headingSrc; f.fixAgeS = fixAgeS;
    f.windDeg = windDeg;        f.batVolts = batVolts;
    f.gpsFix = gpsFix;          f.sats = sats;             f.hdop = hdop;  f.rcOk = rcOk;
    f.wptTotal = wptTotal;      f.wptCur = wptCur;
    f.windObs = windObs;        f.windObsPct = windObsPct;

    char buf[320];
    const int len = formatHeartbeat(buf, sizeof(buf), f);
    if (len < 0 || (size_t)len > HEARTBEAT_LORA_MAX_BYTES) {
        DBG_RADIO("HB too long (%d B) — not sent", len);
        return;
    }

    if (!radio_) return;
    if (radio_->send(buf)) {
        txCount_++;
        DBG_RADIO("HB TX #%lu mode=%s bat=%.2fV loc=(%.4f,%.4f)",
            (unsigned long)txCount_, modeStr, batVolts, lat, lon);
    }
}

void LoRaComm::dispatch(const char* json) {
    DBG_RADIO("dispatch: %.120s", json);
    if (!strstr(json, "\"origin\":\"server\"")) {
        DBG_RADIO("REJECTED: no origin:server");
        return;
    }
    if (!strstr(json, "\"type\":\"command\"")) {
        DBG_RADIO("REJECTED: no type:command");
        return;
    }
    const char* msg = strstr(json, "\"message\":");
    if (!msg) {
        DBG_RADIO("REJECTED: no message field");
        return;
    }

    if (strstr(msg, "\"waypoints\"")) {
        handleWaypoints(msg);
    } else if (strstr(msg, "\"wind-command\"")) {
        handleWindCommand(msg);
    } else if (strstr(msg, "\"navigate\"")) {
        app_->startMission();
        DBG_RADIO("CMD: navigate → startMission");
    } else if (strstr(msg, "\"stop\"")) {
        app_->stopMission();
        DBG_RADIO("CMD: stop → stopMission");
    } else if (strstr(msg, "\"home\"")) {
        handleHome(msg);
    } else if (strstr(msg, "\"wind-observation\"")) {
        app_->startWindObservation();
        DBG_RADIO("CMD: wind-observation → start GPS-track wind estimation");
    } else if (strstr(msg, "\"restart\"")) {
        DBG_RADIO("CMD: restart");
        delay(100);
        ESP.restart();
    }
}

void LoRaComm::handleWaypoints(const char* msg) {
    const char* numPtr = strstr(msg, "\"number\":");
    if (!numPtr) return;
    int count = atoi(numPtr + 9);
    if (count <= 0 || count > (int)MissionPlan::MAX_WAYPOINTS) return;

    const char* ptsPtr = strstr(msg, "\"points\":\"");
    if (!ptsPtr) return;
    ptsPtr += 10;
    const char* endQ = strchr(ptsPtr, '"');
    if (!endQ) return;
    size_t ptsLen = (size_t)(endQ - ptsPtr);
    if (ptsLen >= 200) return;

    char pts[200];
    memcpy(pts, ptsPtr, ptsLen);
    pts[ptsLen] = '\0';

    MissionPlan plan{};
    plan.mode = MissionMode::Linear;
    char* p   = pts;
    bool ok = true;
    for (int i = 0; i < count; i++) {
        double lat = atof(p);
        char* c1 = strchr(p, ',');
        if (!c1) { ok = false; break; }
        p = c1 + 1;
        double lon = atof(p);
        // Range-check + reject the (0,0) sentinel: atof() on a truncated/garbled
        // field silently yields 0.0, which would otherwise look like a valid
        // waypoint off the coast of Africa that the boat dutifully sails toward.
        if (lat < -90.0 || lat > 90.0 || lon < -180.0 || lon > 180.0 ||
            (lat == 0.0 && lon == 0.0)) {
            ok = false;
            break;
        }
        plan.waypoints[plan.count++] = {lat, lon, 10.0f};
        char* c2 = strchr(p, ',');
        if (c2) { p = c2 + 1; }
        else if (i + 1 < count) { ok = false; break; }  // fewer points than "number" claimed
    }
    // All-or-nothing: never commit a plan truncated by a parse/range failure —
    // the boat must keep the old mission rather than sail toward a bad prefix.
    if (ok && plan.count == count) {
        app_->loadMission(plan);
        DBG_RADIO("CMD: waypoints loaded (%u points)", plan.count);
    } else {
        DBG_RADIO("REJECTED: waypoints parse/range failure");
    }
}

void LoRaComm::handleWindCommand(const char* msg) {
    const char* valPtr = strstr(msg, "\"value\":");
    if (!valPtr) return;
    char* endPtr = nullptr;
    long raw = strtol(valPtr + 8, &endPtr, 10);
    // Reject a non-numeric or wildly out-of-range payload (corrupted/truncated radio
    // frame) instead of feeding it straight into the nav math as a "valid" heading.
    if (endPtr == valPtr + 8 || raw < -36000 || raw > 36000) {
        DBG_RADIO("REJECTED: wind-command bad value");
        return;
    }
    // Normalize into [0,360) — same fmod-based wrap as navigation.h's
    // nav_normalizeAngle, duplicated here to keep LoRaComm decoupled from the
    // navigation headers (and from the USE_OLD_NAVIGATION switch).
    double normalized = fmod((double)raw, 360.0);
    if (normalized < 0.0) normalized += 360.0;
    float windDeg = (float)normalized;
    app_->setWindDirection(windDeg);
    DBG_RADIO("CMD: wind-command %.0f deg", windDeg);
}

void LoRaComm::handleHome(const char* msg) {
    const char* latPtr = strstr(msg, "\"lat\":");
    const char* lonPtr = strstr(msg, "\"lon\":");
    if (!latPtr || !lonPtr) return;
    double lat = atof(latPtr + 6);
    double lon = atof(lonPtr + 6);
    app_->setHome(lat, lon);
    DBG_RADIO("CMD: home (%.6f, %.6f)", lat, lon);
}
