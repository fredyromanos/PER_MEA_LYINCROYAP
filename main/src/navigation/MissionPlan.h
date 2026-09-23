#pragma once
#include "../core/Types.h"

struct MissionPlan {
    static constexpr uint8_t MAX_WAYPOINTS = 16;

    Waypoint    waypoints[MAX_WAYPOINTS];
    uint8_t     count = 0;
    MissionMode mode  = MissionMode::Linear;
};
