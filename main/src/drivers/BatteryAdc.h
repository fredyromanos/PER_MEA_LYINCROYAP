#pragma once
#include <Arduino.h>

// Reads battery voltage via GPIO35 voltage divider (R1=560kΩ, R2=120kΩ).
// Call begin() once in setup, then readVolts() at any rate.
class BatteryAdc {
public:
    void  begin();
    float readVolts();
};
