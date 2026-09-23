#include "McpwmActuators.h"
#include "../config/BoardConfig.h"
#include "../config/Calibration.h"
#include "../config/DebugConfig.h"
#include "driver/mcpwm.h"

bool McpwmActuators::begin() {
    bool ok = true;
    bool g0 = (mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, BoardConfig::SAIL_SERVO_PIN)  == ESP_OK);
    bool g1 = (mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, BoardConfig::ROTOR_SERVO_PIN) == ESP_OK);
    bool g2 = (mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, BoardConfig::ESC1_PIN)        == ESP_OK);
    ok = g0 && g1 && g2;
    DBG_ACT("GPIO init: sail=%s rotor=%s esc1=%s",
        g0?"OK":"FAIL", g1?"OK":"FAIL", g2?"OK":"FAIL");

mcpwm_config_t cfg = {};
cfg.frequency = Calibration::PWM_FREQ_HZ;
cfg.cmpr_a = 0.0f;
cfg.cmpr_b = 0.0f;
cfg.counter_mode = MCPWM_UP_COUNTER;
cfg.duty_mode = MCPWM_DUTY_MODE_0;

bool t0 = (mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &cfg) == ESP_OK);
bool t1 = (mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &cfg) == ESP_OK);
ok &= t0 && t1;

DBG_ACT("timer init: T0=%s T1=%s freq=%uHz",
    t0?"OK":"FAIL", t1?"OK":"FAIL", (unsigned)Calibration::PWM_FREQ_HZ);

// Both servo channels use DUTY_MODE_0 (active-high pulse).
// Empirically: DUTY_MODE_0 produces a valid servo signal through
// the PCB's BC548 NPN level-shifter on both PWM1 and PWM2.
// DUTY_MODE_1 was previously applied to MCPWM_OPR_B under the
// assumption that the NPN common-emitter circuit required
// pre-inversion, but this caused the rotor servo to receive an
// unusable signal. Sail (OPR_A) was always MODE_0 and worked.
// Both channels must be MODE_0.

// Output safe starting positions immediately
mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, Calibration::SAIL_CENTER_US);
mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, Calibration::ROTOR_STOP_US);
mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, Calibration::ESC_STOP_US);
    DBG_ACT("safe positions: sail=%u rotor=%u esc=%u",
        (unsigned)Calibration::SAIL_CENTER_US,
        (unsigned)Calibration::ROTOR_STOP_US,
        (unsigned)Calibration::ESC_STOP_US);

    initialized_ = ok;
    return ok;
}

uint16_t McpwmActuators::clamp(uint16_t v, uint16_t lo, uint16_t hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

uint16_t McpwmActuators::slew(uint16_t current, uint16_t target, uint16_t step) {
    if (current < target) {
        const uint32_t next = static_cast<uint32_t>(current) + step;
        return (next >= target) ? target : static_cast<uint16_t>(next);
    }
    if (current > target) {
        return (current > static_cast<uint16_t>(target + step)) ? static_cast<uint16_t>(current - step) : target;
    }
    return current;
}

void McpwmActuators::write(const ActuatorCommand& cmd) {
    if (!initialized_) return;

    const uint16_t sail  = clamp(cmd.sailUs,  1000, 2000);
    // Safety net widened to the autonomous winch envelope (±110°). Manual mode
    // self-limits to ROTOR_MIN/MAX (±90°) in ManualController, so this only
    // grants the extra travel to autonomous steering.
    const uint16_t rotor = clamp(cmd.rotorUs,
                             Calibration::ROTOR_AUTO_MIN_US,
                             Calibration::ROTOR_AUTO_MAX_US);
    const uint16_t e1t   = clamp(cmd.esc1Us,  Calibration::ESC_REVERSE_MIN_US, Calibration::ESC_MAX_US);

    outEsc1Us_ = slew(outEsc1Us_, e1t, Calibration::ESC_SLEW_US);

    // Log sail / rotor changes (skip ESC — slew changes every tick and debugTick already covers them)
    if (sail != prevSailUs_) {
        DBG_ACT("sail  %u → %u µs", (unsigned)prevSailUs_, (unsigned)sail);
        prevSailUs_ = sail;
    }
    if (rotor != prevRotorUs_) {
        DBG_ACT("rotor %u → %u µs", (unsigned)prevRotorUs_, (unsigned)rotor);
        prevRotorUs_ = rotor;
    }

    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, sail);
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, rotor);
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, outEsc1Us_);
}
