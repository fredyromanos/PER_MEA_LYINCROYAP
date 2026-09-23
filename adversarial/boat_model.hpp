#pragma once
// ─────────────────────────────────────────────────────────────────────────────
// Independent adversarial boat model for FALSIFYING autonomous readiness.
//
// This is NOT the "nice" digital twin. It drives the REAL firmware navigation.h
// through a boat model that DELIBERATELY contains the failure modes the twin
// omits, each behind a parameter that defaults to the twin's implicit assumption
// and is SWEPT away from it:
//   • windEstErrorDeg  — nav sees windTrue+err; physics uses windTrue (no wind sensor!)
//   • current(dir,spd) — course-over-ground ≠ heading (coastal tidal current)
//   • winchRateDegPerS — open-loop winch reaches commanded rudder at a finite rate, no feedback
//   • yawMismatch      — whether the nav's relWind/2 compensation actually cancels sail yaw
//                        (1.0 = twin's implicit assumption; 0.0 = no sail yaw at all)
//   • gps noise        — position/course measurement noise
//
// HONESTY NOTE: the disturbance forms (sail-yaw ∝ compensation·yawMismatch, linear
// rudder authority, simple polar) are ILLUSTRATIVE, not measured ground truth. The
// harness reports SENSITIVITY (where it breaks vs each parameter), never "it works".
// Real transfer functions must be measured on the bench/water (see ADVERSARIAL_FINDINGS.md).
//
// At the baseline (all disturbances 0, yawMismatch=1) this model reduces to the twin:
// the boat turns from the pure steering correction and converges — the control that
// proves the harness is not rigged to fail.
// ─────────────────────────────────────────────────────────────────────────────
#include "navigation.h"
#include <cmath>
#include <random>
#include <algorithm>

struct BoatParams {
    double windTrue         = 0.0;    // deg, direction wind comes FROM
    double windSpeed        = 4.0;    // m/s
    double currentDir       = 0.0;    // deg, direction current flows TOWARD
    double currentSpeed     = 0.0;    // m/s
    double windEstErrorDeg  = 0.0;    // nav is fed windTrue + this
    double gpsPosNoiseM     = 0.0;    // 1-sigma position noise (m)
    double gpsCourseNoiseDeg= 0.0;    // 1-sigma course noise (deg)
    double winchRateDegPerS = 1e9;    // rudder slew limit (1e9 = instant = twin)
    double yawMismatch      = 1.0;    // 1 = compensation perfectly cancels sail yaw (twin)
    double kRud             = 0.5;    // rudder authority (deg/s per deg), from twin
    double maxSpeed         = 2.5;    // m/s
    double waypointRadiusM  = 10.0;
};

class AdversarialBoat {
public:
    AdversarialBoat(const BoatParams& p, unsigned seed) : p_(p), rng_(seed) {
        nav_resetState(nav_);
    }

    void init(double lat, double lng, double heading, double wptLat, double wptLng) {
        lat_ = startLat_ = lat; lng_ = startLng_ = lng;
        heading_ = heading; speed_ = 0.0;
        courseOverGround_ = heading;   // at rest with no current, GPS course == heading
        wptLat_ = wptLat; wptLng_ = wptLng;
        actualRudder_ = 0.0; sailSign_ = +1; prevSailSign_ = +1;
        // firmware nav state (fed back each tick like AutoController)
        navSail_ = 0.0f; navRudder_ = 0.0f;
    }

    // One control+physics tick. dt in seconds. Returns false once converged.
    bool step(double dt) {
        // ── GPS as the firmware sees it (position + course over ground, noisy) ──
        double gpsLat = lat_ + gauss(p_.gpsPosNoiseM) / M_PER_DEG_LAT;
        double gpsLng = lng_ + gauss(p_.gpsPosNoiseM) /
                        (NAV_EARTH_RADIUS_M * std::cos(lat_ * M_PI / 180.0) * M_PI / 180.0);

        double dist, bearing;
        geo(gpsLat, gpsLng, wptLat_, wptLng_, dist, bearing);

        double gpsCourse = courseOverGround_ + gauss(p_.gpsCourseNoiseDeg);
        gpsCourse = norm360(gpsCourse);

        // ── REAL firmware navigation (the code under test) ──
        double windEst = norm360(p_.windTrue + p_.windEstErrorDeg);
        NavResult r = nav_handleNavigationWithState(
            nav_, gpsCourse, bearing, dist, windEst,
            navSail_, navRudder_, p_.waypointRadiusM,
            gpsLat, gpsLng, wptLat_, wptLng_, NAV_DEFAULT_CORRIDOR_HALF_WIDTH_M);

        if (r.waypointReached) { reached_ = true; return false; }

        navSail_   = r.sailAngle;      // fed back next tick (as AutoController does)
        navRudder_ = r.rudderAngle;
        sailSign_  = (r.sailAngle >= 0) ? +1 : -1;

        // ── Open-loop winch: slew actual rudder toward the command (no feedback) ──
        double cmd = clampCmd(r.rudderAngle);
        double maxStep = p_.winchRateDegPerS * dt;
        double d = cmd - actualRudder_;
        if (d >  maxStep) d =  maxStep;
        if (d < -maxStep) d = -maxStep;
        actualRudder_ += d;

        // ── Heading dynamics ──
        // Sail-induced yaw modelled as the rudder offset the nav's compensation is
        // meant to cancel: sailYawEquiv = compensation(relWind_true) * yawMismatch.
        // netSteer = actualRudder − sailYawEquiv:
        //   yawMismatch=1 & no wind error ⇒ netSteer = pure correction ⇒ twin behaviour.
        //   yawMismatch≠1 or wind error   ⇒ residual steer even when "aligned" ⇒ drift.
        double relWindTrue = nav_relativeAngle(heading_, p_.windTrue);
        double sailYawEquiv = clampCmd((relWindTrue / 2.0) * p_.yawMismatch);
        double netSteer = actualRudder_ - sailYawEquiv;

        double steerEff = std::min(1.0, speed_ / 0.5);
        double turnRate = -p_.kRud * netSteer * steerEff;   // deg/s
        heading_ = norm360(heading_ + turnRate * dt);

        // ── Speed from a simple polar on |relWind| (same shape as the twin) ──
        double absRW = std::fabs(relWindTrue);
        double polar;
        if      (absRW < 35)  polar = 0.0;
        else if (absRW < 60)  polar = 0.2 + 0.8 * (absRW - 35) / 25.0;
        else if (absRW < 110) polar = 1.0;
        else if (absRW < 150) polar = 1.0 - 0.3 * (absRW - 110) / 40.0;
        else                  polar = 0.7 - 0.3 * (absRW - 150) / 30.0;
        bool coherent = (sailSign_ > 0 && relWindTrue > 0) || (sailSign_ < 0 && relWindTrue < 0);
        double sailEff = coherent ? 0.9 : 0.0;
        double target = p_.maxSpeed * polar * sailEff * (p_.windSpeed / 5.0);
        target = std::min(target, p_.maxSpeed);
        speed_ += (target - speed_) * (target < speed_ ? 0.01 : 0.05);
        if (speed_ < 0.01) speed_ = 0.0;

        // ── Motion over ground = boat velocity + current ──
        double vN = speed_ * std::cos(heading_ * M_PI / 180.0);
        double vE = speed_ * std::sin(heading_ * M_PI / 180.0);
        vN += p_.currentSpeed * std::cos(p_.currentDir * M_PI / 180.0);
        vE += p_.currentSpeed * std::sin(p_.currentDir * M_PI / 180.0);
        courseOverGround_ = norm360(std::atan2(vE, vN) * 180.0 / M_PI);

        double stepDist = std::hypot(vN, vE) * dt;
        lat_ += (vN * dt) / M_PER_DEG_LAT;
        lng_ += (vE * dt) / (NAV_EARTH_RADIUS_M * std::cos(lat_ * M_PI / 180.0) * M_PI / 180.0);
        pathLen_ += stepDist;

        // book-keeping for metrics
        double trueDist, trueBrg;
        geo(lat_, lng_, wptLat_, wptLng_, trueDist, trueBrg);
        if (trueDist < closest_) closest_ = trueDist;
        double xte = std::fabs(nav_crossTrackErrorMeters(startLat_, startLng_, wptLat_, wptLng_, lat_, lng_));
        if (xte > maxXte_) maxXte_ = xte;
        if (sailSign_ != prevSailSign_) { tacks_++; prevSailSign_ = sailSign_; }
        return true;
    }

    // getters for metrics
    bool   reached()      const { return reached_; }
    double pathLen()      const { return pathLen_; }
    double closest()      const { return closest_; }
    double maxCrossTrack()const { return maxXte_; }
    int    tacks()        const { return tacks_; }
    double straightLen()  const {
        double e, n; nav_gpsDeltaMeters(startLat_, startLng_, wptLat_, wptLng_, e, n);
        return std::hypot(e, n);
    }
    double distToWpt()    const {
        double e, n; nav_gpsDeltaMeters(lat_, lng_, wptLat_, wptLng_, e, n);
        return std::hypot(e, n);
    }

private:
    // = NAV_EARTH_RADIUS_M·π/180 (that symbol is const, not constexpr, so use the literal)
    static constexpr double M_PER_DEG_LAT = 6371000.0 * M_PI / 180.0;

    static double norm360(double a){ while(a>=360)a-=360; while(a<0)a+=360; return a; }
    static double clampCmd(double d){
        double L = NAV_RUDDER_COMMAND_LIMIT_DEG;
        return d> L ? L : (d< -L ? -L : d);
    }
    double gauss(double sigma){ if(sigma<=0) return 0.0; std::normal_distribution<double> g(0,sigma); return g(rng_); }
    static void geo(double lat1,double lng1,double lat2,double lng2,double& dist,double& brg){
        double e,n; nav_gpsDeltaMeters(lat1,lng1,lat2,lng2,e,n);
        dist = std::hypot(e,n);
        brg  = norm360(std::atan2(e,n)*180.0/M_PI);
    }

    BoatParams p_;
    std::mt19937 rng_;
    NavState nav_{};
    double lat_=0,lng_=0,startLat_=0,startLng_=0,wptLat_=0,wptLng_=0;
    double heading_=0, courseOverGround_=0, speed_=0, actualRudder_=0;
    float  navSail_=0, navRudder_=0;
    int    sailSign_=+1, prevSailSign_=+1, tacks_=0;
    bool   reached_=false;
    double pathLen_=0, closest_=1e18, maxXte_=0;
};
