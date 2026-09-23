#pragma once
#include <Arduino.h>

class LoRaRadio {
public:
    bool begin();
    bool send(const char* msg);     // blocking TX
    bool poll(char* buf, size_t maxLen);  // non-blocking RX, returns true if packet
    int      rssi()            const { return lastRssi_; }
    bool     ready()           const { return ready_; }
    uint32_t rxDetectedCount() const { return rxDetected_; }

private:
    int      lastRssi_   = 0;
    bool     ready_      = false;
    uint32_t rxDetected_ = 0;  // parsePacket() > 0 events (radio heard a preamble)
};
