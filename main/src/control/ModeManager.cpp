#include "ModeManager.h"
#include "../config/BoardConfig.h"

ControlMode ModeManager::decode(const RcFrame& frame) const {
    if (frame.ch5 == 0) return ControlMode::Failsafe;

    if (frame.ch5 <= BoardConfig::CH5_AUTO_THRESHOLD) return ControlMode::Automatic;
    if (frame.ch5 > BoardConfig::CH5_PROP_THRESHOLD)  return ControlMode::Manual;
    if (frame.ch5 >= BoardConfig::CH5_SAIL_LOW_US &&
        frame.ch5 <= BoardConfig::CH5_SAIL_HIGH_US)   return ControlMode::Sail;

    // Out-of-band value (between zones) — safe fallback (inert)
    return ControlMode::Sail;
}
