#pragma once
#include "../core/Types.h"

class ManualController {
public:
    void            reset();
    // Mode manuel unifié : CH2 voile (binaire), CH4 safran, CH3 propulseur (inverse).
    ActuatorCommand update(const RcFrame& frame);

private:
    // Sail: remembers last toggled direction between frames
    int8_t  sailState_            = +1;
    bool    sailStateInitialized_ = false;

    ActuatorCommand computeManual(const RcFrame& frame);

    static uint16_t clamp(uint16_t v, uint16_t lo, uint16_t hi);
    static uint16_t mapUs(uint16_t in, uint16_t inLo, uint16_t inHi,
                          uint16_t outLo, uint16_t outHi);
};
