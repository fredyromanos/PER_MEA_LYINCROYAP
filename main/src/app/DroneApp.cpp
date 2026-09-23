#include "DroneApp.h"
#include "../drivers/AxpPower.h"
#include "../navigation/Navigator.h"
#include "../config/DebugConfig.h"

static const char* modeName(ControlMode m) {
    switch (m) {
        case ControlMode::Failsafe:    return "FAILSAFE";
        case ControlMode::Sail:        return "SAIL    ";
        case ControlMode::Manual:      return "MANUAL  ";
        case ControlMode::Automatic:   return "AUTO    ";
    }
    return "?       ";
}

void DroneApp::begin() {
    Serial.begin(115200);
    delay(200);
    Serial.println("\n=== SeaDrone boot ===");
    DBG_APP("boot start");

    // 1. AXP192: enable power rails (LDO2=LoRa, LDO3=GPS, DCDC1=3.3V)
    if (!AxpPower::begin()) {
        Serial.println("[AXP]  WARN: init failed — check I2C bus. Continuing.");
    } else {
        Serial.println("[AXP]  OK");
    }

    // 2. Battery ADC: set attenuation before first read
    battery_.begin();
    Serial.println("[BAT]  OK  — R2=562k R1=120k, 11dB attenuation");

    // 3. RC receiver: attaches interrupts on CH2/CH3/CH4/CH5
    rc_.begin();
    Serial.println("[RC]   OK  — waiting for signal...");

    // 4. GPS: Serial1 on GPIO34 (RX) / GPIO12 (TX) at 9600 baud
    gps_.begin();
    Serial.println("[GPS]  OK  — waiting for fix (GPIO34 RX, GPIO12 TX, 9600 baud)");

    // 5. LoRa: SX1276 on SPI HSPI (SCK=5 MISO=19 MOSI=27 CS=18), DIO0=26, RST=23
    if (!loraRadio_.begin()) {
        Serial.println("[LORA]  ERROR: init failed — check SPI wiring / AXP192 LDO2");
    } else {
        lora_.begin(loraRadio_, *this);
        Serial.println("[LORA]  OK  — 433 MHz, TX heartbeat 1 Hz");
    }

    // 6. MCPWM actuators: outputs begin at safe neutral positions
    if (!actuators_.begin()) {
        Serial.println("[ACT]  ERROR: MCPWM init failed");
    } else {
        Serial.println("[ACT]  OK  — sail=1520 rotor=1500 esc=1000");
    }

    manual_.reset();

    Serial.println("=== Ready ===");
    Serial.println("Format: [MODE] CH2=#### CH3=#### CH4=#### CH5=#### | sail=#### rotor=#### esc1=####");
    DBG_APP("boot complete");
}

void DroneApp::update() {
    gps_.update();   // drain Serial1 every iteration — must not be rate-limited
    lora_.update();  // poll LoRa for incoming commands — non-blocking

    const uint32_t now = millis();

    if (static_cast<uint32_t>(now - lastControlMs_) >= CONTROL_PERIOD_MS) {
        controlTick(now);
        lastControlMs_ = now;
    }

    if (static_cast<uint32_t>(now - lastBatMs_) >= BAT_PERIOD_MS) {
        lastBatVolts_ = battery_.readVolts();
        lastBatMs_    = now;
    }

    if (static_cast<uint32_t>(now - lastLoraMs_) >= LORA_PERIOD_MS) {
        loraHbTick();
        lastLoraMs_ = now;
    }

    if (static_cast<uint32_t>(now - lastDebugMs_) >= DEBUG_PERIOD_MS) {
        debugTick();
        lastDebugMs_ = now;
    }
}

void DroneApp::controlTick(uint32_t nowMs) {
    lastFrame_  = rc_.readFrame();
    activeMode_ = modeManager_.decode(lastFrame_);

    // Log mode transitions
    if (activeMode_ != prevMode_) {
        DBG_APP("mode: %s -> %s  (CH5=%u)",
            modeName(prevMode_), modeName(activeMode_), (unsigned)lastFrame_.ch5);
        prevMode_ = activeMode_;
    }

    if (activeMode_ == ControlMode::Manual) {
        lastCommand_ = manual_.update(lastFrame_);
        lastTargetActive_ = false;
    } else if (activeMode_ == ControlMode::Sail) {
        // Position voile : inerte — tous les actionneurs au neutre.
        manual_.reset();
        lastCommand_ = ActuatorCommand{};
        lastTargetActive_ = false;
    } else if (windObsActive_) {
        // Wind-observation maneuver: sail a fixed tack and infer wind from GPS track
        manual_.reset();
        lastTargetActive_ = false;
        lastCommand_ = autoCtrl_.observeWind(gps_.position(), nowMs);
        if (autoCtrl_.windObsComplete()) {
            windDeg_       = autoCtrl_.observedWindDeg();
            windValid_     = true;
            windObsActive_ = false;
        } else if (autoCtrl_.windObsFailed()) {
            // Keep any previous wind value; the observation simply did not produce one.
            DBG_APP("wind observation failed: %s", autoCtrl_.navMessage());
            windObsActive_ = false;
        }
    } else {
        // Automatic OR Failsafe (RC lost) — both follow LoRa mission
        manual_.reset();
        lastTargetActive_ = mission_.update(gps_.position(), lastTarget_);
        if (lastTargetActive_ && windValid_) {
            lastCommand_ = autoCtrl_.compute(windDeg_, gps_.position(), lastTarget_, nowMs);
        } else {
            // No active target, or wind not yet known → hold safe neutral
            autoCtrl_.reset();
            lastCommand_ = ActuatorCommand{};
        }
    }

    actuators_.write(lastCommand_);
}

void DroneApp::debugTick() {
    Serial.printf("[%s] CH2=%4u CH3=%4u CH4=%4u CH5=%4u | sail=%4u rotor=%4u esc1=%4u | bat=%.2fV\n",
        modeName(activeMode_),
        lastFrame_.ch2, lastFrame_.ch3, lastFrame_.ch4, lastFrame_.ch5,
        lastCommand_.sailUs, lastCommand_.rotorUs,
        actuators_.esc1Us(),
        lastBatVolts_);

    const GpsPosition& gp = gps_.position();
    if (gp.valid) {
        static const char* hSrc[] = { "none", "gps", "held" };
        const HeadingGate::State& hs = autoCtrl_.heading();
        Serial.printf("[GPS ] lat=%10.6f lon=%11.6f spd=%5.1fkm/h%s crs=%5.1f°%s sat=%u hdop=%.1f age=%lums | nav-hdg=%s %.0f°\n",
            gp.lat, gp.lon, gp.speedKmph, gp.speedValid ? "" : "(none)",
            gp.courseDeg, gp.courseValid ? "" : "(invalid)",
            (unsigned)gp.satellites, gp.hdop, (unsigned long)gp.ageMs,
            hSrc[hs.source], hs.deg);
    } else {
        Serial.printf("[GPS ] NO FIX  age=%lums sat=%u hdop=%.1f visible=%u  chars=%lu  badCRC=%lu\n",
            (unsigned long)gp.ageMs, (unsigned)gp.satellites, gp.hdop,
            (unsigned)gps_.satsInView(),
            (unsigned long)gps_.charsProcessed(),
            (unsigned long)gps_.failedChecksums());
        if (gps_.lastLine()[0] != '\0')
            Serial.printf("       last: %s\n", gps_.lastLine());
    }

#if DEBUG_ENABLED && DEBUG_RADIO
    Serial.printf("[LORA] tx=%lu  rxDet=%lu  rxRssi=%d%s\n",
        (unsigned long)lora_.txCount(),
        (unsigned long)loraRadio_.rxDetectedCount(),
        lora_.lastRxRssi(),
        loraRadio_.ready() ? "" : "  [NOT INIT]");
#endif

    if (activeMode_ == ControlMode::Automatic ||
        activeMode_ == ControlMode::Failsafe) {
        static const char* stateNames[] = { "IDLE", "RUNNING", "RETURNING", "COMPLETE" };
        const char* mName = (mission_.mode() == MissionMode::Circuit) ? "CIRCUIT" : "LINEAR ";
        const uint8_t s = static_cast<uint8_t>(mission_.state());

        Serial.printf("[AUTO ] wind=%.0f°%s  nav=%s  %s\n",
            windDeg_,
            windObsActive_ ? " (observing...)"
                           : (!windValid_ ? " (!set wind: wind-command or wind-observation before navigate!)" : ""),
            autoCtrl_.navMode(),
            autoCtrl_.navMessage());

        if (lastTargetActive_ && gp.valid) {
            const float dist    = Navigator::distanceM(gp.lat, gp.lon, lastTarget_.lat, lastTarget_.lon);
            const float bearing = Navigator::bearingDeg(gp.lat, gp.lon, lastTarget_.lat, lastTarget_.lon);
            Serial.printf("[MISN ] %s %s wp=%u/%u target=(%.6f,%.6f r=%.0fm) dist=%.0fm brg=%.0f°\n",
                stateNames[s], mName,
                (unsigned)mission_.currentIndex() + 1, (unsigned)mission_.waypointCount(),
                lastTarget_.lat, lastTarget_.lon, lastTarget_.radiusM,
                dist, bearing);
        } else {
            Serial.printf("[MISN ] %s %s wp=%u/%u home=%s%s\n",
                stateNames[s], mName,
                (unsigned)mission_.currentIndex() + 1, (unsigned)mission_.waypointCount(),
                mission_.hasHome() ? "SET" : "NOT SET",
                !gp.valid ? " (no GPS fix)" : "");
        }
    }
}

void DroneApp::loraHbTick() {
    const GpsPosition& gp = gps_.position();

    // RC présent si au moins un canal renvoie une largeur valide
    // (RcReceiver renvoie 0 sur un canal après 100 ms sans impulsion).
    const bool rcOk = (lastFrame_.ch2 != 0 || lastFrame_.ch3 != 0 ||
                       lastFrame_.ch4 != 0 || lastFrame_.ch5 != 0);

    // Heading telemetry: while navigating, the gated heading actually used to steer;
    // otherwise just whether the GPS course is usable right now.
    const HeadingGate::State& hs = autoCtrl_.heading();
    const bool navigating = lastTargetActive_ && windValid_;
    const uint8_t headingSrc = navigating ? (uint8_t)hs.source
                                          : (gp.courseValid ? (uint8_t)HeadingGate::Gps : 0);
    const float headingDeg = (navigating && hs.valid) ? hs.deg : gp.courseDeg;
    const uint32_t ageS = (gp.ageMs == 0xFFFFFFFFUL) ? 99 : gp.ageMs / 1000;

    lora_.sendHeartbeat(
        activeMode_,
        mission_.state(),
        gp.lat, gp.lon, headingDeg, headingSrc, (uint8_t)(ageS > 99 ? 99 : ageS),
        lastBatVolts_,
        mission_.currentIndex(),
        mission_.waypointCount(),
        windDeg_,
        lastCommand_.sailUs,
        lastCommand_.rotorUs,
        gp.valid, gp.satellites, gp.hdop, rcOk,
        windObsActive_, autoCtrl_.windObsProgressPct()
    );
}
