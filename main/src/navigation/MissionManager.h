#pragma once
#include "../core/Types.h"
#include "MissionPlan.h"

class MissionManager {
public:
    // --- Mission loading (called by LoRa command handler in Phase 3) ---
    void loadMission(const MissionPlan& plan);
    void clearMission();

    // --- Home point (independent of mission — survives loadMission calls) ---
    // LoRa will call setHome() when the boat is deployed, or at any time.
    void setHome(double lat, double lon);
    bool hasHome() const { return homeSet_; }

    // --- Mission control ---
    void start();            // Idle → Running  (no-op if no waypoints loaded)
    void stop();             // any  → Idle
    void emergencyReturn();  // any  → Returning (bypasses remaining waypoints)

    // --- Navigation query — call every control tick when in Automatic mode ---
    // Fills outTarget with the current navigation goal.
    // Returns true while a target is active; false when Idle or Complete.
    bool update(const GpsPosition& pos, Waypoint& outTarget);

    // --- State accessors ---
    MissionState state()         const { return state_; }
    uint8_t      currentIndex()  const { return idx_; }
    uint8_t      waypointCount() const { return plan_.count; }
    MissionMode  mode()          const { return plan_.mode; }

private:
    // Radius used when homing — slightly larger than typical waypoint radius
    // to account for GPS drift near the shore.
    static constexpr float HOME_ARRIVAL_RADIUS_M = 15.0f;

    MissionPlan  plan_;
    MissionState state_   = MissionState::Idle;
    uint8_t      idx_     = 0;
    bool         homeSet_ = false;
    double       homeLat_ = 0.0;
    double       homeLon_ = 0.0;

    void advanceWaypoint();
};
