// ⚠ OBSOLÈTE pour la carte (2026-09-16) : la prédiction est désormais calculée côté
//   serveur (/api/predict → AutoBoat_Simulation/predict_path) avec le vrai navigation.h.
//   Ce fichier avait divergé du firmware (couloir 20 m, pas de compensation de barre,
//   cap piloté par la commande brute). Conservé seulement pour référence.
// ============================================================
//  nav.js — Réplique JS de navigation.h (firmware) + simulateur
//  de trajectoire pour la prédiction affichée sur la carte.
//
//  La LOGIQUE DE NAVIGATION est portée à l'identique depuis
//  main/src/navigation/navigation.h (ne pas diverger). Le seul
//  ajout est un modèle cinématique simple (cap qui suit le
//  gouvernail, vitesse constante) — le firmware ne définit pas
//  la dynamique du bateau, donc le TRACÉ est approché même si la
//  logique de barre/voile est exacte.
// ============================================================

(function (global) {
    "use strict";

    // --- Constantes (copiées de navigation.h) ---
    const NAV_DIRECT_DEAD_ZONE_DEG         = 5.0;
    const NAV_VDB_RUDDER_GAIN              = 0.8;
    const NAV_DIRECT_RUDDER_GAIN           = 0.5;
    const NAV_RUDDER_LIMIT_DEG             = 20.0;
    const NAV_DEFAULT_CORRIDOR_HALF_WIDTH_M = 20.0;
    const NAV_GEO_EPSILON_DEG              = 0.0000001;
    const NAV_EARTH_RADIUS_M               = 6371000.0;
    const NAV_UPWIND_FORBIDDEN_ANGLE_DEG   = 45.0;
    const NAV_DOWNWIND_FORBIDDEN_ANGLE_DEG = 20.0;
    const NAV_RUDDER_STEP_DEG              = 5.0;
    const NAV_SAIL_RIGHT_DEG               = 10.0;
    const NAV_SAIL_LEFT_DEG                = -10.0;
    const DEG2RAD = Math.PI / 180.0;

    // --- Paramètres du modèle cinématique (PROPRES au simulateur) ---
    const SIM_STEP_M           = 2.5;    // distance parcourue par pas
    const SIM_STEER_GAIN       = 0.45;   // cap (deg) gagné par degré de barre / pas
    const SIM_MAX_TURN_PER_STEP = 12.0;  // virage max par pas (deg)
    const SIM_MAX_STEPS        = 2500;
    const SIM_MAX_PATH_M       = 4000.0;

    // --- Helpers angulaires (navigation.h) ---
    function relativeAngle(refDeg, targetDeg) {
        let rel = targetDeg - refDeg;
        if (rel > 180) rel -= 360;
        else if (rel < -180) rel += 360;
        return rel;
    }
    function sameSign(a, b) { return (a >= 0 && b >= 0) || (a < 0 && b < 0); }
    function oppositeAngle(a) { const o = a + 180; return (o >= 360) ? o - 360 : o; }
    function angleSide(a) { return a < 0 ? -1 : 1; }
    function clampRudder(r) {
        if (r > NAV_RUDDER_LIMIT_DEG) return NAV_RUDDER_LIMIT_DEG;
        if (r < -NAV_RUDDER_LIMIT_DEG) return -NAV_RUDDER_LIMIT_DEG;
        return r;
    }
    function normalizeAngle(a) {
        while (a >= 360) a -= 360;
        while (a < 0) a += 360;
        return a;
    }
    function isBetweenOnShortestTurn(angleDeg, startDeg, endDeg) {
        const turn = relativeAngle(startDeg, endDeg);
        const from = relativeAngle(startDeg, angleDeg);
        if (turn >= 0) return from > 0 && from < turn;
        return from < 0 && from > turn;
    }

    // --- Géodésie (navigation.h + Navigator.cpp) ---
    function gpsDeltaMeters(refLat, refLng, lat, lng) {
        const latRad = refLat * DEG2RAD;
        const northM = (lat - refLat) * NAV_EARTH_RADIUS_M * DEG2RAD;
        const eastM  = (lng - refLng) * NAV_EARTH_RADIUS_M * Math.cos(latRad) * DEG2RAD;
        return { eastM, northM };
    }
    function distanceM(lat1, lon1, lat2, lon2) {
        const dLat = (lat2 - lat1) * DEG2RAD;
        const dLon = (lon2 - lon1) * DEG2RAD;
        const a = Math.sin(dLat / 2) ** 2 +
                  Math.cos(lat1 * DEG2RAD) * Math.cos(lat2 * DEG2RAD) * Math.sin(dLon / 2) ** 2;
        return NAV_EARTH_RADIUS_M * 2.0 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
    }
    function bearingDeg(lat1, lon1, lat2, lon2) {
        const dLon = (lon2 - lon1) * DEG2RAD;
        const y = Math.sin(dLon) * Math.cos(lat2 * DEG2RAD);
        const x = Math.cos(lat1 * DEG2RAD) * Math.sin(lat2 * DEG2RAD) -
                  Math.sin(lat1 * DEG2RAD) * Math.cos(lat2 * DEG2RAD) * Math.cos(dLon);
        return (Math.atan2(y, x) / DEG2RAD + 360.0) % 360.0;
    }
    // Avance d'une distance le long d'un cap (pour le modèle cinématique).
    function destinationPoint(lat, lon, bearing, distM) {
        const latRad = lat * DEG2RAD;
        const northM = distM * Math.cos(bearing * DEG2RAD);
        const eastM  = distM * Math.sin(bearing * DEG2RAD);
        const newLat = lat + (northM / NAV_EARTH_RADIUS_M) / DEG2RAD;
        const newLon = lon + (eastM / (NAV_EARTH_RADIUS_M * Math.cos(latRad))) / DEG2RAD;
        return { lat: newLat, lon: newLon };
    }

    function crossTrackErrorMeters(sLat, sLng, eLat, eLng, bLat, bLng) {
        const p = gpsDeltaMeters(sLat, sLng, eLat, eLng);
        const b = gpsDeltaMeters(sLat, sLng, bLat, bLng);
        const len = Math.sqrt(p.eastM * p.eastM + p.northM * p.northM);
        if (len < 0.1) return 0.0;
        return (p.northM * b.eastM - p.eastM * b.northM) / len;
    }

    function sideMovingTowardCrossTrack(sLat, sLng, eLat, eLng, axisDeg, safeAngleDeg,
                                        desiredCrossTrackSign, fallbackSide) {
        const p = gpsDeltaMeters(sLat, sLng, eLat, eLng);
        const len = Math.sqrt(p.eastM * p.eastM + p.northM * p.northM);
        if (len < 0.1) return fallbackSide;
        const pEu = p.eastM / len, pNu = p.northM / len;
        let bestSide = fallbackSide, bestScore = -1.0;
        for (let side = -1; side <= 1; side += 2) {
            const headRad = normalizeAngle(axisDeg + side * safeAngleDeg) * DEG2RAD;
            const crossVel = pNu * Math.sin(headRad) - pEu * Math.cos(headRad);
            const wantSign = (desiredCrossTrackSign > 0) ? (crossVel > 0) : (crossVel < 0);
            if (wantSign) {
                const score = Math.abs(crossVel);
                if (score > bestScore) { bestScore = score; bestSide = side; }
            }
        }
        return bestSide;
    }

    // --- État de navigation ---
    function newState() {
        return {
            corridor: { initialized: false, startLatDeg: 0, startLngDeg: 0, targetLatDeg: 0, targetLngDeg: 0 },
            upwindSide: 0,
            downwindSide: 0,
            empannagePhase: 0,           // 0=AUCUN,1=ALLER_UPWIND,2=CROISER,3=REVENIR_DOWNWIND
            empannageTargetSide: 0
        };
    }
    function resetCorridor(s) { s.corridor = { initialized: false, startLatDeg: 0, startLngDeg: 0, targetLatDeg: 0, targetLngDeg: 0 }; }
    function resetState(s) {
        resetCorridor(s);
        s.upwindSide = 0; s.downwindSide = 0; s.empannagePhase = 0; s.empannageTargetSide = 0;
    }
    function hasCorridor(bLat, bLng, wLat, wLng, halfW) {
        return halfW > 0.0 &&
            (Math.abs(wLat - bLat) > NAV_GEO_EPSILON_DEG || Math.abs(wLng - bLng) > NAV_GEO_EPSILON_DEG);
    }
    function updateCorridor(s, hc, bLat, bLng, wLat, wLng) {
        if (!hc) { resetCorridor(s); return; }
        const targetChanged =
            Math.abs(wLat - s.corridor.targetLatDeg) > NAV_GEO_EPSILON_DEG ||
            Math.abs(wLng - s.corridor.targetLngDeg) > NAV_GEO_EPSILON_DEG;
        if (s.corridor.initialized && !targetChanged) return;
        s.corridor = { initialized: true, startLatDeg: bLat, startLngDeg: bLng, targetLatDeg: wLat, targetLngDeg: wLng };
        s.upwindSide = 0; s.downwindSide = 0; s.empannagePhase = 0; s.empannageTargetSide = 0;
    }

    function applyTargetHeading(r, boatHeading, targetHeading, relativeWind, rudderGain, mode, logMessage) {
        const err = relativeAngle(boatHeading, targetHeading);
        r.sailAngle = (relativeWind < 0) ? NAV_SAIL_LEFT_DEG : NAV_SAIL_RIGHT_DEG;
        r.rudderAngle = clampRudder(-err * rudderGain);
        r.mode = mode;
        r.logMessage = logMessage;
    }

    function corridorSide(s, bLat, bLng, halfW, axisDeg, safeAngleDeg, currentSide) {
        if (!s.corridor.initialized) return currentSide;
        const cte = crossTrackErrorMeters(s.corridor.startLatDeg, s.corridor.startLngDeg,
            s.corridor.targetLatDeg, s.corridor.targetLngDeg, bLat, bLng);
        if (cte > halfW)
            return sideMovingTowardCrossTrack(s.corridor.startLatDeg, s.corridor.startLngDeg,
                s.corridor.targetLatDeg, s.corridor.targetLngDeg, axisDeg, safeAngleDeg, -1, currentSide);
        if (cte < -halfW)
            return sideMovingTowardCrossTrack(s.corridor.startLatDeg, s.corridor.startLngDeg,
                s.corridor.targetLatDeg, s.corridor.targetLngDeg, axisDeg, safeAngleDeg, 1, currentSide);
        return currentSide;
    }

    function startAvoidEmpannage(r, s, boatHeading, windDir, relativeWind, targetSide, logMessage) {
        s.empannageTargetSide = targetSide;
        s.empannagePhase = 1; // ALLER_LIMITE_UPWIND
        const targetHeading = normalizeAngle(windDir - s.downwindSide * NAV_UPWIND_FORBIDDEN_ANGLE_DEG);
        applyTargetHeading(r, boatHeading, targetHeading, relativeWind, NAV_DIRECT_RUDDER_GAIN, "avoid-gybe", logMessage);
    }

    function handleEmpannageLoop(r, s, boatHeading, windDir, oppositeWind, relativeWind) {
        let targetHeading;
        let mode = "avoid-gybe";
        let logMessage = "Avoid empannage: loop around through upwind";
        if (s.empannagePhase === 1) {
            targetHeading = normalizeAngle(windDir - s.downwindSide * NAV_UPWIND_FORBIDDEN_ANGLE_DEG);
            if (Math.abs(relativeAngle(boatHeading, targetHeading)) < 10.0) s.empannagePhase = 2;
        } else if (s.empannagePhase === 2) {
            targetHeading = normalizeAngle(windDir + s.downwindSide * NAV_UPWIND_FORBIDDEN_ANGLE_DEG);
            mode = "vdb";
            logMessage = "VDB: downwind loop crosses upwind axis";
            if (angleSide(relativeAngle(windDir, boatHeading)) === s.downwindSide) {
                s.downwindSide = s.empannageTargetSide;
                s.empannagePhase = 3;
            }
        } else {
            targetHeading = normalizeAngle(oppositeWind + s.empannageTargetSide * NAV_DOWNWIND_FORBIDDEN_ANGLE_DEG);
            if (angleSide(relativeAngle(oppositeWind, boatHeading)) === s.empannageTargetSide) {
                s.downwindSide = s.empannageTargetSide;
                s.empannagePhase = 0;
            }
        }
        applyTargetHeading(r, boatHeading, targetHeading, relativeWind, NAV_DIRECT_RUDDER_GAIN, mode, logMessage);
    }

    function handleForbiddenZone(r, s, boatHeading, bLat, bLng, windAxisDeg, relativeWind, hc, halfW, upwind) {
        const sideKey = upwind ? "upwindSide" : "downwindSide";
        const forbiddenAngle = upwind ? NAV_UPWIND_FORBIDDEN_ANGLE_DEG : NAV_DOWNWIND_FORBIDDEN_ANGLE_DEG;
        const gain = upwind ? NAV_VDB_RUDDER_GAIN : NAV_DIRECT_RUDDER_GAIN;
        const mode = upwind ? "upwind-zigzag" : "downwind-zigzag";
        const logMessage = upwind ? "Waypoint in upwind forbidden zone" : "Waypoint in downwind forbidden zone";
        if (s[sideKey] === 0)
            s[sideKey] = angleSide(relativeAngle(windAxisDeg, boatHeading));
        if (hc)
            s[sideKey] = corridorSide(s, bLat, bLng, halfW, windAxisDeg, forbiddenAngle, s[sideKey]);
        const targetHeading = normalizeAngle(windAxisDeg + s[sideKey] * forbiddenAngle);
        applyTargetHeading(r, boatHeading, targetHeading, relativeWind, gain, mode, logMessage);
    }

    function handleDownwindZigzag(r, s, boatHeading, bLat, bLng, windDir, oppositeWind, relativeWind, hc, halfW) {
        if (s.downwindSide === 0)
            s.downwindSide = angleSide(relativeAngle(oppositeWind, boatHeading));
        let desiredSide = s.downwindSide;
        if (hc)
            desiredSide = corridorSide(s, bLat, bLng, halfW, oppositeWind, NAV_DOWNWIND_FORBIDDEN_ANGLE_DEG, s.downwindSide);
        if (desiredSide !== s.downwindSide) {
            startAvoidEmpannage(r, s, boatHeading, windDir, relativeWind, desiredSide,
                "Avoid empannage: loop around through upwind");
            return;
        }
        handleForbiddenZone(r, s, boatHeading, bLat, bLng, oppositeWind, relativeWind, false, halfW, false);
    }

    function handleLoferAbattre(r, relativeWind, currentRudderAngle, lofer) {
        const windLeft = relativeWind < 0;
        r.sailAngle = windLeft ? NAV_SAIL_LEFT_DEG : NAV_SAIL_RIGHT_DEG;
        r.rudderAngle = currentRudderAngle + ((windLeft === lofer) ? NAV_RUDDER_STEP_DEG : -NAV_RUDDER_STEP_DEG);
        r.mode = lofer ? "lofer" : "abattre";
    }

    // Coeur de l'algorithme — réplique de nav_handleNavigationWithState.
    function handleNavigationWithState(state, boatHeading, wptHeading, wptDist, windDir,
                                       currentSail, currentRudder, reachedDist,
                                       bLat, bLng, wLat, wLng, halfW) {
        const r = {
            sailAngle: currentSail, rudderAngle: currentRudder,
            mode: "direct", logMessage: "", waypointReached: false
        };
        if (wptDist <= reachedDist) {
            r.waypointReached = true;
            resetState(state);
            return r;
        }
        const hc = hasCorridor(bLat, bLng, wLat, wLng, halfW);
        updateCorridor(state, hc, bLat, bLng, wLat, wLng);

        const oppositeWind = oppositeAngle(windDir);
        const relativeWind = relativeAngle(boatHeading, windDir);
        const relativeWpt  = relativeAngle(boatHeading, wptHeading);
        const wpUp   = Math.abs(relativeAngle(windDir, wptHeading)) < NAV_UPWIND_FORBIDDEN_ANGLE_DEG;
        const wpDown = Math.abs(relativeAngle(oppositeWind, wptHeading)) < NAV_DOWNWIND_FORBIDDEN_ANGLE_DEG;
        const crossesUp   = isBetweenOnShortestTurn(windDir, boatHeading, wptHeading);
        const crossesDown = isBetweenOnShortestTurn(oppositeWind, boatHeading, wptHeading);

        if (state.empannagePhase !== 0) {
            handleEmpannageLoop(r, state, boatHeading, windDir, oppositeWind, relativeWind);
        } else if (wpUp || crossesUp) {
            handleForbiddenZone(r, state, boatHeading, bLat, bLng, windDir, relativeWind, hc, halfW, true);
        } else if (wpDown) {
            handleDownwindZigzag(r, state, boatHeading, bLat, bLng, windDir, oppositeWind, relativeWind, hc, halfW);
        } else if (crossesDown) {
            if (state.downwindSide === 0)
                state.downwindSide = angleSide(relativeAngle(oppositeWind, boatHeading));
            startAvoidEmpannage(r, state, boatHeading, windDir, relativeWind, -state.downwindSide,
                "Avoid empannage: direct path crosses downwind axis");
        } else if (Math.abs(relativeWpt) <= NAV_DIRECT_DEAD_ZONE_DEG) {
            r.sailAngle = (relativeWind < 0) ? NAV_SAIL_LEFT_DEG : NAV_SAIL_RIGHT_DEG;
            r.rudderAngle = 0;
            r.logMessage = "Direct: waypoint aligned";
        } else {
            handleLoferAbattre(r, relativeWind, currentRudder, sameSign(relativeWind, relativeWpt));
        }
        r.rudderAngle = clampRudder(r.rudderAngle);
        return r;
    }

    // ============================================================
    //  Simulateur de trajectoire
    //  Renvoie un tableau [[lat,lon], ...] du bateau vers le waypoint.
    //  windDir = direction D'OÙ VIENT le vent (convention firmware).
    // ============================================================
    function predictTrajectory(boatLat, boatLon, boatHeading, windDir, wpLat, wpLon, reachedDist) {
        const rdist = (reachedDist && reachedDist > 0) ? reachedDist : 10.0;
        const state = newState();
        let lat = boatLat, lon = boatLon;
        // Si le cap n'est pas connu (bateau à l'arrêt), on part vers le waypoint.
        let heading = (boatHeading === null || boatHeading === undefined)
            ? bearingDeg(lat, lon, wpLat, wpLon) : boatHeading;
        let sail = NAV_SAIL_RIGHT_DEG;
        let rudder = 0.0;

        const path = [[lat, lon]];
        let travelled = 0.0;

        for (let i = 0; i < SIM_MAX_STEPS; i++) {
            const dist = distanceM(lat, lon, wpLat, wpLon);
            const brg  = bearingDeg(lat, lon, wpLat, wpLon);
            const r = handleNavigationWithState(state, heading, brg, dist, windDir,
                sail, rudder, rdist, lat, lon, wpLat, wpLon, NAV_DEFAULT_CORRIDOR_HALF_WIDTH_M);

            if (r.waypointReached) { path.push([wpLat, wpLon]); break; }

            // Persiste l'état d'angle comme le firmware (intégrateur lofer/abattre).
            sail = r.sailAngle;
            rudder = r.rudderAngle;

            // Modèle cinématique : le cap suit le gouvernail.
            let dH = -rudder * SIM_STEER_GAIN;
            if (dH > SIM_MAX_TURN_PER_STEP) dH = SIM_MAX_TURN_PER_STEP;
            if (dH < -SIM_MAX_TURN_PER_STEP) dH = -SIM_MAX_TURN_PER_STEP;
            heading = normalizeAngle(heading + dH);

            const next = destinationPoint(lat, lon, heading, SIM_STEP_M);
            lat = next.lat; lon = next.lon;
            travelled += SIM_STEP_M;
            path.push([lat, lon]);

            if (travelled > SIM_MAX_PATH_M) break;  // garde-fou (vent impossible, etc.)
        }
        return path;
    }

    global.AutoBoatNav = { predictTrajectory, bearingDeg, distanceM };
})(window);
