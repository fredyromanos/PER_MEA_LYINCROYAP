#pragma once
#include "../core/Types.h"

class ModeManager {
public:
    ControlMode decode(const RcFrame& frame) const;
};
