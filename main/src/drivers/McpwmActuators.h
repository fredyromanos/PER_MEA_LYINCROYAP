#pragma once
#include "../core/Types.h"

// Drives sail servo, rotor winch, and single ESC via ESP32 MCPWM hardware.
// MCPWM is used instead of LEDC because LEDC conflicts with RC PWM interrupts.
//
// MCPWM mapping:
//   TIMER_0 OPR_A → sail servo  (GPIO2)
//   TIMER_0 OPR_B → rotor winch (GPIO25)
//   TIMER_1 OPR_A → ESC 1       (GPIO15)
class McpwmActuators {
public:
    bool     begin();
    void     write(const ActuatorCommand& cmd);

    uint16_t esc1Us() const { return outEsc1Us_; }

private:
    bool     initialized_ = false;
    uint16_t outEsc1Us_   = 1500;  // démarre au neutre (ESC bidirectionnel)
    uint16_t prevSailUs_  = 0;   // last logged sail value (0 = not yet logged)
    uint16_t prevRotorUs_ = 0;   // last logged rotor value

    static uint16_t clamp(uint16_t v, uint16_t lo, uint16_t hi);
    static uint16_t slew(uint16_t current, uint16_t target, uint16_t step);
};
