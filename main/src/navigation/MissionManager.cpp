#include "MissionManager.h"
#include "Navigator.h"
#include "../config/DebugConfig.h"

static const char* stateName(MissionState s) {
    switch (s) {
        case MissionState::Idle:      return "Idle";
        case MissionState::Running:   return "Running";
        case MissionState::Returning: return "Returning";
        case MissionState::Complete:  return "Complete";
    }
    return "?";
}

void MissionManager::loadMission(const MissionPlan& plan) {
    plan_  = plan;
    idx_   = 0;
    const char* modeStr = (plan.mode == MissionMode::Circuit) ? "Circuit" : "Linear";
    DBG_MISN("mission loaded: %u waypoints, mode=%s", (unsigned)plan.count, modeStr);
    for (uint8_t i = 0; i < plan.count; i++) {
        DBG_MISN("  wp[%u] lat=%.6f lon=%.6f r=%.0fm",
            (unsigned)i, plan.waypoints[i].lat,
            plan.waypoints[i].lon, (double)plan.waypoints[i].radiusM);
    }
}

void MissionManager::clearMission() {
    plan_.count = 0;
    idx_        = 0;
    state_      = MissionState::Idle;
    DBG_MISN("mission cleared → Idle");
}

void MissionManager::setHome(double lat, double lon) {
    homeLat_ = lat;
    homeLon_ = lon;
    homeSet_ = true;
    DBG_MISN("home set: lat=%.6f lon=%.6f", lat, lon);
}

void MissionManager::start() {
    if (plan_.count == 0) {
        DBG_MISN("start() ignored — no waypoints loaded");
        return;
    }
    idx_   = 0;
    state_ = MissionState::Running;
    DBG_MISN("mission START — %u waypoints, first=(%.6f,%.6f)",
        (unsigned)plan_.count, plan_.waypoints[0].lat, plan_.waypoints[0].lon);
}

void MissionManager::stop() {
    DBG_MISN("mission STOP  %s → Idle", stateName(state_));
    state_ = MissionState::Idle;
    idx_   = 0;
}

void MissionManager::emergencyReturn() {
    DBG_MISN("EMERGENCY RETURN  %s → Returning  home=%s",
        stateName(state_), homeSet_ ? "SET" : "NOT SET");
    state_ = MissionState::Returning;
}

bool MissionManager::update(const GpsPosition& pos, Waypoint& outTarget) {
    if (state_ == MissionState::Idle || state_ == MissionState::Complete) {
        return false;
    }

    if (state_ == MissionState::Returning) {
        if (!homeSet_) {
            DBG_MISN("Returning but no home set → Complete");
            state_ = MissionState::Complete;
            return false;
        }
        outTarget.lat     = homeLat_;
        outTarget.lon     = homeLon_;
        outTarget.radiusM = HOME_ARRIVAL_RADIUS_M;
        if (pos.valid) {
            const float d = Navigator::distanceM(pos.lat, pos.lon, homeLat_, homeLon_);
            if (d <= HOME_ARRIVAL_RADIUS_M) {
                DBG_MISN("home reached (dist=%.1fm) → Complete", (double)d);
                state_ = MissionState::Complete;
                return false;
            }
        }
        return true;
    }

    // Running
    if (plan_.count == 0) {
        DBG_MISN("Running but no waypoints → Idle");
        state_ = MissionState::Idle;
        return false;
    }

    if (pos.valid) {
        const float d = Navigator::distanceM(pos.lat, pos.lon,
                                             plan_.waypoints[idx_].lat,
                                             plan_.waypoints[idx_].lon);
        if (d <= plan_.waypoints[idx_].radiusM) {
            DBG_MISN("wp[%u] reached (dist=%.1fm)", (unsigned)idx_, (double)d);
            advanceWaypoint();
        }
    }

    if (state_ == MissionState::Running) {
        outTarget = plan_.waypoints[idx_];
        return true;
    }
    if (state_ == MissionState::Returning) {
        if (!homeSet_) {
            DBG_MISN("post-advance Returning but no home → Complete");
            state_ = MissionState::Complete;
            return false;
        }
        outTarget.lat     = homeLat_;
        outTarget.lon     = homeLon_;
        outTarget.radiusM = HOME_ARRIVAL_RADIUS_M;
        return true;
    }
    return false;
}

void MissionManager::advanceWaypoint() {
    const uint8_t prev = idx_;
    if (plan_.mode == MissionMode::Circuit) {
        idx_ = static_cast<uint8_t>((idx_ + 1) % plan_.count);
        DBG_MISN("waypoint advance (Circuit): wp[%u] → wp[%u]", (unsigned)prev, (unsigned)idx_);
    } else {
        if (idx_ + 1 < plan_.count) {
            idx_++;
            DBG_MISN("waypoint advance (Linear): wp[%u] → wp[%u]", (unsigned)prev, (unsigned)idx_);
        } else {
            idx_   = 0;
            state_ = MissionState::Returning;
            DBG_MISN("last waypoint reached — Running → Returning");
        }
    }
}
