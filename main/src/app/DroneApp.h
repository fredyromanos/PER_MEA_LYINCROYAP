#pragma once
#include "../drivers/RcReceiver.h"
#include "../drivers/McpwmActuators.h"
#include "../drivers/BatteryAdc.h"
#include "../drivers/GpsUart.h"
#include "../drivers/LoRaRadio.h"
#include "../comm/LoRaComm.h"
#include "../control/ModeManager.h"
#include "../control/ManualController.h"
#include "../control/AutoController.h"
#include "../navigation/MissionManager.h"
#include "../core/Types.h"

class DroneApp {
public:
    void begin();
    void update();

    // --- Mission interface (called by the LoRa command handler) ---
    void loadMission(const MissionPlan& plan) { mission_.loadMission(plan); }
    void clearMission()                        { mission_.clearMission(); }
    void setHome(double lat, double lon)       { mission_.setHome(lat, lon); }
    void startMission()                        { windObsActive_ = false; mission_.start(); }
    void stopMission()                         { windObsActive_ = false; mission_.stop(); }
    void emergencyReturn()                     { windObsActive_ = false; mission_.emergencyReturn(); }

    // Wind: set manually (wind-command) or estimated from the GPS track (wind-observation).
    void setWindDirection(float deg)           { windDeg_ = deg; windValid_ = true; }
    void startWindObservation()                { autoCtrl_.beginWindObservation(); windObsActive_ = true; }

private:
    RcReceiver       rc_;
    McpwmActuators   actuators_;
    BatteryAdc       battery_;
    GpsUart          gps_;
    LoRaRadio        loraRadio_;
    LoRaComm         lora_;
    ModeManager      modeManager_;
    ManualController manual_;
    AutoController   autoCtrl_;
    MissionManager   mission_;

    RcFrame         lastFrame_;
    ActuatorCommand lastCommand_;
    Waypoint        lastTarget_;
    bool            lastTargetActive_ = false;
    ControlMode     activeMode_       = ControlMode::Failsafe;
    ControlMode     prevMode_         = ControlMode::Failsafe;
    float           lastBatVolts_     = 0.0f;
    float           windDeg_          = 0.0f;
    bool            windValid_        = false;  // true once wind is set or estimated
    bool            windObsActive_    = false;  // wind-observation maneuver in progress
    uint32_t        lastGpsChars_     = 0;
    uint32_t        lastControlMs_    = 0;
    uint32_t        lastBatMs_        = 0;
    uint32_t        lastDebugMs_      = 0;
    uint32_t        lastLoraMs_       = 0;

    static constexpr uint32_t CONTROL_PERIOD_MS = 20;    // 50 Hz
    static constexpr uint32_t BAT_PERIOD_MS     = 250;   // 4 Hz
    static constexpr uint32_t DEBUG_PERIOD_MS   = 200;   // 5 Hz
    static constexpr uint32_t LORA_PERIOD_MS    = 1000;  // 1 Hz heartbeat

    void controlTick(uint32_t nowMs);
    void debugTick();
    void loraHbTick();
};
