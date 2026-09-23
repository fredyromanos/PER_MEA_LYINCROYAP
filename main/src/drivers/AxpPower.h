#pragma once

// AXP192 power management initialization for LilyGO T-Beam V1.1.
// Must be called first in setup(), before any other peripheral.
// Enables LoRa (LDO2) and GPS (LDO3) power rails, then releases
// the I2C bus so GPIO21/22 can be reused as RC PWM inputs.
class AxpPower {
public:
    static bool begin();
};
