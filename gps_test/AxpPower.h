#pragma once

// AXP192 power management initialization for LilyGO T-Beam V1.1.
// Must be called first in setup(), before any other peripheral.
// Enables the GPS (LDO3) and LoRa (LDO2) power rails.
class AxpPower {
public:
    static bool begin();
};
