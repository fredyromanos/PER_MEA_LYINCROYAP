// I2C stub: counts every bus operation so tests can prove whether a chip was contacted.
#pragma once
#include "Arduino.h"
class TwoWire {
public:
    int transmissions = 0, bytesWritten = 0, requests = 0;
    std::vector<uint8_t> nextRead;   // bytes returned by read()
    void begin(int = -1, int = -1) {}
    void setClock(uint32_t) {}
    void end() {}
    void beginTransmission(uint8_t) { ++transmissions; }
    size_t write(uint8_t) { ++bytesWritten; return 1; }
    uint8_t endTransmission(bool = true) { return 0; }
    uint8_t requestFrom(uint8_t, uint8_t n) { ++requests; return n; }
    int available() { return (int)nextRead.size(); }
    int read() { if (nextRead.empty()) return 0; int v = nextRead.front(); nextRead.erase(nextRead.begin()); return v; }
};
extern TwoWire Wire;
