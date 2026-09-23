#pragma once
#include <Arduino.h>
#include "../core/Types.h"

class LoRaRadio;   // forward declaration — full type in LoRaComm.cpp
class DroneApp;    // forward declaration — full type in LoRaComm.cpp

class LoRaComm {
public:
    void begin(LoRaRadio& radio, DroneApp& app);
    void update();   // poll for incoming packets and dispatch commands

    void sendHeartbeat(ControlMode mode, MissionState mState,
                       double lat, double lon, float heading,
                       uint8_t headingSrc, uint8_t fixAgeS,
                       float batVolts, uint8_t wptCur, uint8_t wptTotal,
                       float windDeg, uint16_t sailUs, uint16_t rotorUs,
                       bool gpsFix, uint8_t sats, float hdop, bool rcOk,
                       bool windObs, uint8_t windObsPct);

    uint32_t txCount()    const { return txCount_; }
    int      lastRxRssi() const { return lastRxRssi_; }

private:
    LoRaRadio* radio_      = nullptr;
    DroneApp*  app_        = nullptr;
    uint32_t   txCount_    = 0;
    int        lastRxRssi_ = 0;
    char       rxBuf_[256] = {};

    void dispatch(const char* json);
    void handleWaypoints(const char* msg);
    void handleWindCommand(const char* msg);
    void handleHome(const char* msg);
};
