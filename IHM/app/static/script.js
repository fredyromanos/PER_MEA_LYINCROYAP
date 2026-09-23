///////////////////// Get elements///////////////////////////////////
let boatLatitudeDisplay = document.getElementById("boatLatitudeDisplay");
let boatLongitudeDisplay = document.getElementById("boatLongitudeDisplay");
let boatLastTimeDisplay = document.getElementById("boatLastTimeDisplay");
let routeFileInput = document.getElementById('routeFileInput');
let navigateButton = document.getElementById('navigateButton');
let boatHeadingDisplay = document.getElementById("boatHeadingDisplay");
let boatSailAngleDisplay = document.getElementById("boatSailAngleDisplay");
let boatRudderAngleDisplay = document.getElementById("boatRudderAngleDisplay");
let windObservationButton = document.getElementById('windObservationButton');
let windCommandButton = document.getElementById('windCommandButton');
let windDirectionDisplay = document.getElementById("windDirectionDisplay");
let waypointTotalDisplay = document.getElementById("waypointTotalDisplay");
let waypointCurrentDisplay = document.getElementById("waypointCurrentDisplay");
let boatModeDisplay = document.getElementById("boatModeDisplay");
let restartButton = document.getElementById('restartButton');
let startButton = document.getElementById('startButton');
let simScenarioSelect = document.getElementById('simScenarioSelect');
let resetCommunicationsButton = document.getElementById('resetCommunicationsButton');

// Panneau de santé / diagnostic
let healthLinkDot = document.getElementById("healthLinkDot");
let healthLinkValue = document.getElementById("healthLinkValue");
let healthGpsDot = document.getElementById("healthGpsDot");
let healthGpsValue = document.getElementById("healthGpsValue");
let healthBatDot = document.getElementById("healthBatDot");
let healthBatValue = document.getElementById("healthBatValue");
let healthRcDot = document.getElementById("healthRcDot");
let healthRcValue = document.getElementById("healthRcValue");
let healthModeDot = document.getElementById("healthModeDot");
let healthModeValue = document.getElementById("healthModeValue");

// Bascule Simulation / Réel
let modeToggle = document.getElementById("modeToggle");

// Reconnexion / reset transceiver
let resetTransceiverButton = document.getElementById("resetTransceiverButton");

// Mesure du vent — barre de progression
let windObsSection = document.getElementById("windObsSection");
let windObsBar = document.getElementById("windObsBar");
let windObsValue = document.getElementById("windObsValue");
let windObsCancelButton = document.getElementById("windObsCancelButton");

let boatMarker;     // The current boat location
let boatRoute;      // The route history for the boat
let pollingIntervalId = null;

// Suivi de fraîcheur des messages (détection perte de liaison)
let lastSeenTimestamp = null;   // dernier timestamp bateau vu
let lastFreshMs = null;         // instant (Date.now) du dernier message NEUF
let lastBoatMessage = null;     // dernier message non vide reçu
let healthWarning = document.getElementById("healthWarning");

// Etat de la batterie
let progressBattery = document.getElementById("progressBattery");
let percentBattery = document.getElementById("percentBattery");

// Puissance moteurs
let powerBar = document.getElementById("powerBar");
let powerValue = document.getElementById("powerValue");

// Vitesse du bateau
let prevLat = null;
let prevLon = null;
let prevTime = null;
let speed_kmh = document.getElementById('speed_kmh');
let speed_knots = document.getElementById('speed_knots');


/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////




/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////

// Polling the boat messages every seconds
function pollBoatMessages() {
    fetch("/api/messages").then(response => {
        if (!response.ok) {
            throw new Error('Network response was not ok');
        }
        return response.json(); // Parse JSON response
    }).then(data => {
        const message = data.message || {};
        //console.log('Data received:', data); // Handle the data

        // Détection de fraîcheur : un nouveau timestamp = message neuf reçu.
        // L'API renvoie toujours le dernier message connu, même s'il est ancien,
        // donc on compare le timestamp plutôt que la simple présence de données.
        if (data.timestamp && data.timestamp !== lastSeenTimestamp) {
            lastSeenTimestamp = data.timestamp;
            lastFreshMs = Date.now();
        }
        if (Object.keys(message).length > 0) {
            lastBoatMessage = message;
        }
        refreshLinkHealth();

        if (data.hasOwnProperty('timestamp')) {
            updateBoatLastTime(data.timestamp);
        }
        if (Object.keys(message).length === 0) {
            return;
        }

        updateHealth(message);
        if (message.hasOwnProperty('mode')) {
            updateBoatMode(message.mode);
        }
        if (message.hasOwnProperty('location')) {
            updateBoatLocation(message.location, data.timestamp);
        }
        if (message.hasOwnProperty('servos')) {
            updateBoatAngles(message.servos.sail, message.servos.rudder);
        }
        if (message.hasOwnProperty('heading')) {
            updateBoatHeading(message.heading, message.hv);
        }
        if (message.hasOwnProperty('wind')) {
            updateWindDirection(message.wind);
        }
        // Waypoints — format aplati "wt"/"wc" (ancien "waypoints":{total,current} en repli).
        if (message.hasOwnProperty('wt')) {
            updateWaypoints(message.wt, message.wc);
        } else if (message.hasOwnProperty('waypoints')) {
            updateWaypoints(message.waypoints.total, message.waypoints.current);
        }
        // Mesure du vent en cours : barre de progression.
        updateWindObs(message);

        // Alimente la prédiction de trajectoire (si la carte est prête).
        if (message.hasOwnProperty('location') && typeof feedTelemetry === 'function') {
            const loc = message.location;
            const wpIdx = message.hasOwnProperty('wc')
                ? message.wc
                : (message.waypoints ? message.waypoints.current : 0);
            // hv absent (ancien firmware) = cap supposé valide.
            const headingValid = !message.hasOwnProperty('hv') || Number(message.hv) !== 0;
            feedTelemetry(loc[0], loc[1], message.heading, message.wind, wpIdx, headingValid);
        }
        if (message.hasOwnProperty('battery')) {
            updateBattery(message.battery);
        } else if (message.hasOwnProperty('bat')) {
            updateBatteryVoltage(message.bat);
        }
        if (message.hasOwnProperty('motor_power')) {
            updateMotorPower(message.motor_power.power, message.motor_power.percent);
        }
    }).catch(error => {
        console.error('There was a problem with the fetch operation:', error);
    });
}

function startMessagePolling() {
    if (pollingIntervalId !== null) {
        return;
    }

    pollBoatMessages();
    pollingIntervalId = setInterval(pollBoatMessages, 1000);
}

function stopMessagePolling() {
    if (pollingIntervalId === null) {
        return;
    }

    clearInterval(pollingIntervalId);
    pollingIntervalId = null;
}

startMessagePolling();


/////////////////////////////////////////////////////////////////////////////
// Panneau de santé / diagnostic
/////////////////////////////////////////////////////////////////////////////

function setHealth(dot, valueEl, level, text) {
    dot.classList.remove("ok", "warn", "bad", "unknown", "radio");
    dot.classList.add(level);
    if (valueEl) {
        valueEl.textContent = text;
    }
}

// Liaison : basée sur l'âge du dernier message NEUF reçu.
// L'API renvoie toujours le dernier message connu (même ancien),
// donc on mesure le temps écoulé depuis le dernier changement de timestamp.
function refreshLinkHealth() {
    let level;

    if (lastFreshMs === null) {
        setHealth(healthLinkDot, healthLinkValue, "unknown", "en attente…");
        level = "unknown";
    } else {
        const ageMs = Date.now() - lastFreshMs;
        const ageS = Math.round(ageMs / 1000);

        if (ageMs < 3000) {
            // Liaison fraîche : on affiche le RSSI (injecté par le transceiver).
            const rssi = (lastBoatMessage && lastBoatMessage.rssi != null)
                ? Number(lastBoatMessage.rssi) : null;
            if (rssi === null) {
                setHealth(healthLinkDot, healthLinkValue, "ok", "OK");
            } else if (rssi <= -110) {
                setHealth(healthLinkDot, healthLinkValue, "warn", rssi + " dBm (faible)");
            } else {
                setHealth(healthLinkDot, healthLinkValue, "ok", rssi + " dBm");
            }
            level = "ok";
        } else if (ageMs < 10000) {
            setHealth(healthLinkDot, healthLinkValue, "warn", ageS + " s");
            level = "warn";
        } else {
            setHealth(healthLinkDot, healthLinkValue, "bad", "perdue (" + ageS + " s)");
            level = "bad";
        }
    }

    // Si la liaison est perdue, les autres paramètres ne sont plus à jour →
    // on les passe en jaune + avertissement. (L'attente initiale et un léger
    // retard ne déclenchent pas l'alerte.)
    const reliable = (level !== "bad");
    applyReliability(reliable);
}

// Marque les paramètres bateau comme fiables (vraies couleurs) ou non (jaune).
function applyReliability(reliable) {
    if (reliable) {
        if (healthWarning) {
            healthWarning.style.display = "none";
        }
        // Restaure les vraies couleurs à partir du dernier message connu.
        if (lastBoatMessage) {
            updateHealth(lastBoatMessage);
        }
    } else {
        if (healthWarning) {
            healthWarning.style.display = "";
        }
        // Force en jaune : les valeurs affichées ne sont plus fiables.
        [healthGpsDot, healthBatDot, healthRcDot, healthModeDot].forEach((dot) => {
            dot.classList.remove("ok", "bad", "unknown", "radio");
            dot.classList.add("warn");
        });
    }
}

// GPS, batterie, RC, contrôle : à partir du dernier message.
function updateHealth(message) {
    // GPS — basé sur fix / sat / hdop (champs envoyés par le bateau).
    if (message.hasOwnProperty("fix")) {
        const fix = Number(message.fix);
        const sat = Number(message.sat);
        const hdop = Number(message.hdop);

        // fa = âge de la dernière position (s), hv = source du cap (0 aucun, 1 GPS, 2 maintenu).
        const fa = message.hasOwnProperty("fa") ? Number(message.fa) : 0;
        const hv = message.hasOwnProperty("hv") ? Number(message.hv) : 1;
        const capTxt = hv === 0 ? " · cap indisponible" : (hv === 2 ? " · cap maintenu" : "");

        if (!fix) {
            // Fix perdu : le bateau met les actionneurs au neutre et reprend seul au retour du fix.
            const age = fa >= 99 ? "" : " (" + fa + " s)";
            setHealth(healthGpsDot, healthGpsValue, "bad", "pas de fix" + age + " · neutre");
        } else {
            // Fix présent : vert si précision correcte ET cap disponible, jaune sinon.
            const good = (sat >= 5 && hdop <= 2.5 && hv !== 0);
            setHealth(
                healthGpsDot,
                healthGpsValue,
                good ? "ok" : "warn",
                sat + " sat, hdop " + (Number.isNaN(hdop) ? "?" : hdop.toFixed(1)) + capTxt
            );
        }
    }

    // Batterie — seuils LiPo 2S.
    if (message.hasOwnProperty("bat")) {
        const v = Number(message.bat);
        let level = "unknown";
        if (!Number.isNaN(v)) {
            if (v >= 7.0) level = "ok";
            else if (v >= 6.6) level = "warn";
            else level = "bad";
        }
        setHealth(
            healthBatDot,
            healthBatValue,
            level,
            Number.isNaN(v) ? "—" : v.toFixed(2) + " V"
        );
    }

    // Radio (RC) — rc=1 si le récepteur fournit des impulsions.
    if (message.hasOwnProperty("rc")) {
        const rcOk = Number(message.rc) === 1;
        setHealth(
            healthRcDot,
            healthRcValue,
            rcOk ? "ok" : "bad",
            rcOk ? "OK" : "perdue"
        );
    }

    // Contrôle — déduit de "mode" (control_mode supprimé du heartbeat) :
    // standby ⟺ radio (manuel, céleste), route-ready/navigate ⟺ auto (vert).
    if (message.hasOwnProperty("mode")) {
        const auto = (message.mode !== "standby");
        setHealth(
            healthModeDot,
            healthModeValue,
            auto ? "ok" : "radio",
            auto ? "auto" : "radio"
        );
    }
}

// Rafraîchit la liaison même sans nouveau message (pour passer au rouge).
setInterval(refreshLinkHealth, 1000);


/////////////////////////////////////////////////////////////////////////////
// Bascule Simulation / Réel  (étape 1 : visuel uniquement)
/////////////////////////////////////////////////////////////////////////////

function applyModeVisibility(mode) {
    const simOnly = (mode === "sim");

    // Boutons réservés à la simulation
    if (startButton) {
        startButton.style.display = simOnly ? "" : "none";
    }
    if (simScenarioSelect) {
        simScenarioSelect.style.display = simOnly ? "" : "none";
    }
    if (resetCommunicationsButton) {
        resetCommunicationsButton.style.display = simOnly ? "" : "none";
    }

    // Bouton réservé au mode réel (reset matériel du transceiver)
    if (resetTransceiverButton) {
        resetTransceiverButton.style.display = simOnly ? "none" : "";
    }
}

function initModeToggle() {
    const saved = localStorage.getItem("autoboatMode") || "real";

    if (modeToggle) {
        modeToggle.checked = (saved === "sim");
    }
    applyModeVisibility(saved);

    if (modeToggle) {
        modeToggle.addEventListener("change", async () => {
            const mode = modeToggle.checked ? "sim" : "real";
            localStorage.setItem("autoboatMode", mode);
            applyModeVisibility(mode);

            // En mode réel, on relance serial_link sur le port réel.
            // En mode sim, l'utilisateur lance la pile via le bouton Start.
            if (mode === "real") {
                try {
                    const response = await fetch("/api/set-mode", {
                        method: "POST",
                        headers: { "Content-Type": "application/json" },
                        body: JSON.stringify({ mode: "real" })
                    });
                    if (!response.ok) {
                        console.error("set-mode réel échoué :", await response.json());
                        alert("Erreur lors du passage en mode réel.");
                    }
                } catch (error) {
                    console.error("Erreur réseau set-mode :", error);
                    alert("Impossible de contacter le serveur.");
                }
            }
        });
    }
}

initModeToggle();


/////////////////////////////////////////////////////////////////////////////
// Reconnexion PC ↔ transceiver + reset de la carte transceiver
/////////////////////////////////////////////////////////////////////////////

if (resetTransceiverButton) {
    resetTransceiverButton.addEventListener("click", async () => {
        if (!confirm("🔌 Réinitialiser le transceiver et reconnecter ?")) {
            return;
        }
        try {
            const response = await fetch("/api/reset-transceiver", {
                method: "POST",
                headers: { "Content-Type": "application/json" },
                body: JSON.stringify({})
            });

            const result = await response.json();

            if (!response.ok) {
                console.error("Erreur reset transceiver :", result);
                alert("Erreur lors de la réinitialisation du transceiver.");
                return;
            }

            console.log("Reset transceiver :", result);
            alert("Transceiver réinitialisé, reconnexion en cours.");

        } catch (error) {
            console.error("Erreur réseau reset transceiver :", error);
            alert("Impossible de contacter le serveur.");
        }
    });
}


/////////////////////////////////////////////////////////////////////////////
// Define a function to update the boat location on the map, and compute the speed of the boat based on the previous location and time
/////////////////////////////////////////////////////////////////////////////

// Update the boat location
// Icône bateau autonome (SVG inline, aucune image externe) : coque orientée par le cap,
// grisée quand le cap GPS n'est pas fiable (hv = 0 : cap inconnu, flèche non orientée).
let boatIconHeading = 0;
let boatIconHv = 0;
let boatCenteredOnce = false;

function boatIcon() {
    const known = Number(boatIconHv) !== 0;
    const fill = known ? "#e67e22" : "#95a5a6";
    const rot = known ? Number(boatIconHeading) || 0 : 0;
    const html =
        '<div class="boat-icon" style="transform: rotate(' + rot + 'deg);">' +
        '<svg width="34" height="34" viewBox="0 0 34 34">' +
        '<circle cx="17" cy="17" r="15" fill="white" fill-opacity="0.85" stroke="' + fill + '" stroke-width="2"/>' +
        '<path d="M17 4 L24 24 Q17 29 10 24 Z" fill="' + fill + '" stroke="#2c3e50" stroke-width="1.2"/>' +
        (known ? "" : '<text x="17" y="22" text-anchor="middle" font-size="12" font-weight="bold" fill="#2c3e50">?</text>') +
        "</svg></div>";
    return L.divIcon({ className: "boat-divicon", html: html, iconSize: [34, 34], iconAnchor: [17, 17] });
}

function refreshBoatIcon() {
    if (boatMarker) {
        boatMarker.setIcon(boatIcon());
    }
}

function updateBoatLocation(newLocation, timestamp) {
    const lat = newLocation[0];
    const lon = newLocation[1];
    const currentTime = new Date(timestamp).getTime() / 1000;

    if (prevLat !== null && prevLon !== null && prevTime !== null) {
        computeBoatSpeed(prevLat, prevLon, lat, lon, prevTime, currentTime);
    }

    prevLat = lat;
    prevLon = lon;
    prevTime = currentTime;

    const leafletMap = window.autoboatMap;

    if (!leafletMap || typeof leafletMap.addLayer !== "function") {
        console.error("Carte Leaflet indisponible dans script.js :", leafletMap);
        return;
    }

    const currentLatLng = [lat, lon];

    if (!boatMarker) {
        boatMarker = L.marker(currentLatLng, {
            icon: boatIcon(),
            zIndexOffset: 1000,          // toujours au-dessus des waypoints et tracés
            title: "Bateau"
        }).addTo(leafletMap);

        // Première position reçue : centrer la carte sur le bateau.
        if (!boatCenteredOnce && !(lat === 0 && lon === 0)) {
            leafletMap.setView(currentLatLng, Math.max(leafletMap.getZoom(), 17));
            boatCenteredOnce = true;
        }

        boatRoute = L.polyline([currentLatLng], {
            color: "blue",
            weight: 8,
            opacity: 0.5,
            clickable: false
        }).addTo(leafletMap);
    } else {
        boatMarker.setLatLng(currentLatLng);

        if (boatRoute) {
            boatRoute.addLatLng(currentLatLng);
        }
    }

    boatLatitudeDisplay.textContent = lat;
    boatLongitudeDisplay.textContent = lon;
}


function distanceGPS(lat1, lon1, lat2, lon2) {
    const R = 6371000; // rayon Terre en mètres

    const toRad = (deg) => deg * Math.PI / 180;

    const dLat = toRad(lat2 - lat1);
    const dLon = toRad(lon2 - lon1);

    const a =
        Math.sin(dLat / 2) ** 2 +
        Math.cos(toRad(lat1)) *
        Math.cos(toRad(lat2)) *
        Math.sin(dLon / 2) ** 2;

    const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));

    return R * c; // distance en mètres
}

function computeBoatSpeed(lat1, lon1, lat2, lon2, t1, t2) {
    const distance = distanceGPS(lat1, lon1, lat2, lon2); 
    const dt = t2 - t1; // secondes

    // Sécurité critique pour éviter le NaN
    if (dt <= 0 || isNaN(dt)) {
        return 0;
    }

    if (dt <= 0) return 0;

    // En simulation accélérée (x10…), le bateau parcourt `simSpeedup` secondes de trajet
    // par seconde réelle : on ramène la vitesse au temps simulé.
    const speedMs = (distance / dt) / (simSpeedup > 0 ? simSpeedup : 1);
    speed_kmh.textContent = (speedMs * 3.6).toFixed(2); // Affichage avec arrondis (toFixed(2)) pour que ce soit lisible
    speed_knots.textContent = (speedMs * 1.94384).toFixed(2);
}

// Facteur d'accélération de la simulation en cours (1 = temps réel / bateau réel).
let simSpeedup = 1;
async function refreshSimInfo() {
    try {
        const r = await fetch("/api/sim-info");
        const info = await r.json();
        simSpeedup = Number(info.speedup) || 1;
    } catch (e) {
        simSpeedup = 1;
    }
}
refreshSimInfo();
setInterval(refreshSimInfo, 5000);
    


/////////////////////////////////////////////////////////////////////////////
// Functions to update the boat information on the dashboard
/////////////////////////////////////////////////////////////////////////////

// Update the boat last time online
function updateBoatLastTime(newTime) {
    boatLastTimeDisplay.textContent = newTime;
}

// Update the boat heading
// hv : 0 = pas de cap fiable (bateau trop lent : cap GPS inexploitable), 1 = GPS, 2 = maintenu.
function updateBoatHeading(newHeading, hv) {
    boatIconHeading = Number(newHeading) || 0;
    boatIconHv = (hv === undefined) ? 1 : Number(hv);   // ancien firmware sans hv : cap supposé valide
    refreshBoatIcon();
    if (hv !== undefined && Number(hv) === 0) {
        boatHeadingDisplay.textContent = "— (cap GPS indisponible)";
        boatHeadingDisplay.style.opacity = "0.5";
        return;
    }
    boatHeadingDisplay.style.opacity = "";
    boatHeadingDisplay.textContent = Number(hv) === 2 ? newHeading + " (maintenu)" : newHeading;
}

// Update the boat angles (sail and rudder)
function updateBoatAngles(newSailAngle, newRudderAngle) {
    boatSailAngleDisplay.textContent = newSailAngle;
    boatRudderAngleDisplay.textContent = newRudderAngle;
}

// Update the wind direction
function updateWindDirection(newWindDirection) {
    windDirectionDisplay.textContent = newWindDirection;
    updateWindCompass(newWindDirection);
}

// Boussole : la flèche pointe vers la SOURCE du vent (convention firmware :
// "wind" = direction d'où vient le vent). Carte orientée nord en haut.
function updateWindCompass(wind) {
    const compass = document.getElementById("windCompass");
    const arrow = document.getElementById("wcArrowG");
    const value = document.getElementById("wcValue");
    const w = Number(wind);

    if (compass) {
        compass.classList.toggle("no-data", Number.isNaN(w));
    }
    if (Number.isNaN(w)) {
        if (value) value.textContent = "?";
        return;
    }
    if (arrow) {
        arrow.setAttribute("transform", "rotate(" + w + " 36 36)");
    }
    if (value) {
        value.textContent = Math.round(w);
    }
}


// Update the waypoints details (total and current)
function updateWaypoints(newWaypointTotal, newWaypointCurrent) {
    waypointTotalDisplay.textContent = newWaypointTotal;
    waypointCurrentDisplay.textContent = newWaypointCurrent;
}

// Update the boat mode
function updateBoatMode(newBoatMode) {
    boatModeDisplay.textContent = newBoatMode;
}

// Mesure du vent : affiche la barre tant que "wobs" est présent dans le message.
// Sans "wobs" → mesure inactive → section cachée.
// Si 0 % sans fix GPS → on indique l'attente plutôt qu'une barre figée à 0 %.
function updateWindObs(message) {
    if (!windObsSection) {
        return;
    }
    if (!message || !message.hasOwnProperty("wobs")) {
        windObsSection.style.display = "none";
        return;
    }

    const p = Math.max(0, Math.min(100, Number(message.wobs) || 0));
    const hasFix = Number(message.fix) === 1;

    windObsSection.style.display = "";
    if (windObsBar) {
        windObsBar.style.width = p + "%";
    }
    if (windObsValue) {
        if (p === 0 && !hasFix) {
            windObsValue.textContent = "attente fix GPS…";
        } else {
            windObsValue.textContent = p + "%";
        }
    }
}

/////////////////////////////////////////////////////////////////////////////
// Function to send the route waypoints to the boat, this is called after the user upload a new route
/////////////////////////////////////////////////////////////////////////////

async function sendRoute(waypoints) {
    console.log(waypoints);
    try {
        const response = await fetch('/api/send-route', {
            method: 'POST',
            headers: {
                "Content-Type": "application/json"
            },
            body: JSON.stringify({
                waypoints: waypoints
            })
        });

        const result = await response.json();

        if (!response.ok) {
            console.error("Erreur envoi route :", result);
            alert(result.message || "Erreur lors de l'envoi de la route.");
        }
    } catch (error) {
        console.error("Erreur réseau envoi route :", error);
        alert("Impossible de contacter le serveur.");
    }
}

/////////////////////////////////////////////////////////////////////////////
// Functions to update the battery and motor power information on the dashboard
/////////////////////////////////////////////////////////////////////////////

function updateBattery(percent)
{
    progressBattery.style.width = percent + "%";
    percentBattery.textContent = percent + "%";
}

function updateBatteryVoltage(volts)
{
    const voltage = Number(volts);
    if (Number.isNaN(voltage)) {
        return;
    }

    progressBattery.style.width = "0%";
    percentBattery.textContent = voltage.toFixed(2) + " V";
}

function updateMotorPower(power, percent)
{
    powerBar.style.width = percent + "%";
    powerValue.textContent = power.toFixed(1) + " W";
}

function resetBoatDisplay()
{
    const leafletMap = window.autoboatMap;

    if (leafletMap && boatMarker) {
        leafletMap.removeLayer(boatMarker);
    }

    if (leafletMap && boatRoute) {
        leafletMap.removeLayer(boatRoute);
    }

    boatMarker = null;
    boatRoute = null;
    prevLat = null;
    prevLon = null;
    prevTime = null;

    boatLastTimeDisplay.textContent = "?";
    boatLatitudeDisplay.textContent = "?";
    boatLongitudeDisplay.textContent = "?";
    boatHeadingDisplay.textContent = "?";
    boatSailAngleDisplay.textContent = "?";
    boatRudderAngleDisplay.textContent = "?";
    windDirectionDisplay.textContent = "?";
    waypointTotalDisplay.textContent = "?";
    waypointCurrentDisplay.textContent = "?";
    boatModeDisplay.textContent = "not-connected";
    speed_kmh.textContent = "0.00";
    speed_knots.textContent = "0.00";

    updateBattery(0);
    updateMotorPower(0, 0);
}

/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////

routeFileInput.addEventListener('change', (event) => {
    const file = event.target.files[0];

    if (file && file.type === "application/geo+json") {  // Ensure it's a GeoJSON file
        const reader = new FileReader();

        reader.onload = function (e) {
            const fileContent = e.target.result;
            // console.log("GeoJSON File Content:");
            // console.log(fileContent);
            let routeGeoJSON = parseGeoJSONRoute(fileContent);
            if (!routeGeoJSON) {
                return;
            }

            let waypoints = addRouteToMap(routeGeoJSON);
            sendRoute(waypoints);
        };

        reader.onerror = function () {
            alert("Error reading file");
        };

        // Read the file as text (GeoJSON is a JSON format, so it's read as text)
        reader.readAsText(file);
    } else {
        alert("Please upload a valid .geojson file");
    }
});

/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////

// Send the navigation order to the boat
navigateButton.addEventListener("click", () => {
    fetch('/api/navigate', {
        method: 'GET',
    });
});

// Send the wind observation order to the boat
windObservationButton.addEventListener("click", async () => {
    try {
        const response = await fetch('/api/wind-observation', { method: 'GET' });
        const result = await response.json().catch(() => ({}));

        if (!response.ok || result.status === "error") {
            console.error("Erreur mesure vent :", result);
            alert("Erreur lors du lancement de la mesure du vent.");
            return;
        }
        alert("Mesure du vent lancée.\n\nLe bateau doit être en mode Auto, avec un fix GPS, " +
              "et se déplacer (~30 m) pour que la mesure progresse.");
    } catch (error) {
        console.error("Erreur réseau mesure vent :", error);
        alert("Impossible de contacter le serveur.");
    }
});

// Annuler la mesure du vent (réutilise /api/stop : stopMission() met
// windObsActive_=false côté bateau). La section disparaîtra dès que le
// bateau cessera d'envoyer "wobs".
if (windObsCancelButton) {
    windObsCancelButton.addEventListener("click", async () => {
        if (!confirm("Annuler la mesure du vent ?")) {
            return;
        }
        try {
            const response = await fetch('/api/stop', { method: 'GET' });
            const result = await response.json().catch(() => ({}));
            if (!response.ok || result.status === "error") {
                console.error("Erreur annulation mesure :", result);
                alert("Erreur lors de l'annulation.");
            }
        } catch (error) {
            console.error("Erreur réseau annulation mesure :", error);
            alert("Impossible de contacter le serveur.");
        }
    });
}

restartButton.addEventListener("click", () => {
    if (confirm("⚠️ Redémarrer ? ⚠️")) {
        fetch('/api/restart', {
            method: 'GET',
        });
    }
});

startButton.addEventListener("click", async () => {
    const scenario = simScenarioSelect ? simScenarioSelect.value : "classic";
    if (confirm("✅  Démarrer (" + scenario + ") ? ✅ ")) {
        try {
            const response = await fetch('/api/start', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ scenario: scenario }),
            });

            const result = await response.json();

            if (!response.ok) {
                console.error("Erreur start :", result);
                alert("Erreur lors du démarrage.");
                return;
            }

            // Nouvelle simulation : on efface le tracé du bateau de la session précédente.
            if (boatRoute) {
                boatRoute.setLatLngs([]);
            }
            prevLat = prevLon = prevTime = null;
            simSpeedup = Number(result.speedup) || 1;

            // Mission préchargée : afficher ses waypoints (la route est déjà envoyée au bateau).
            if (result.mission && typeof clearRoute === "function" && typeof addWaypoint === "function") {
                clearRoute();
                result.mission.waypoints.forEach((w) => addWaypoint(w.lat, w.lon));
                if (window.autoboatMap && result.mission.waypoints.length > 0) {
                    const pts = result.mission.waypoints.map((w) => [w.lat, w.lon]);
                    window.autoboatMap.fitBounds(pts, { padding: [80, 80] });
                }
            }

            startMessagePolling();
            console.log("Start OK :", result);
            alert(result.mission
                ? "Mission lancée : " + result.mission.title + " (route, vent et navigation envoyés, x" + simSpeedup + ")."
                : "Système démarré.");

        } catch (error) {
            console.error("Erreur réseau start :", error);
            alert("Impossible de démarrer le système.");
        }
    }
});

resetCommunicationsButton.addEventListener("click", async () => {
    if (confirm("Réinitialiser toutes les communications ?")) {
        try {
            const response = await fetch('/api/reset-communications', {
                method: 'POST',
            });

            const result = await response.json();

            if (!response.ok) {
                console.error("Erreur reset communications :", result);
                alert("Erreur lors de la réinitialisation des communications.");
                return;
            }

            stopMessagePolling();
            resetBoatDisplay();
            console.log("Communications réinitialisées :", result);
            alert("Communications réinitialisées.");

        } catch (error) {
            console.error("Erreur réseau reset communications :", error);
            alert("Impossible de réinitialiser les communications.");
        }
    }
});

windCommandButton.addEventListener("click", () => {
    const direction = prompt("Direction du vent (en degrés) :")
    if (direction != null) {
        fetch('/api/wind-command/' + direction, {
            method: 'GET',
        });

    }
});

/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////

// Function to parse GeoJSON route and add it to the map if it's a LineString geometry type
function parseGeoJSONRoute(GeoJSONString) {
    try {
    let routeGeoJSON = JSON.parse(GeoJSONString);

    // Vérifier que c'est une FeatureCollection avec au moins une feature
        if (routeGeoJSON.type !== "FeatureCollection" || routeGeoJSON.features.length === 0) {
            alert("Please upload a valid .geojson file");
            return null;
        }

        // Vérifier que la première feature est un LineString
        if (routeGeoJSON.features[0].geometry.type !== "LineString") {
            alert("Please upload a valid .geojson file");
            return null;
        }

    return routeGeoJSON;
    } catch (error) {
    console.error("Error parsing GeoJSON:", error);
    alert("Please upload a valid .geojson file");
    return null;
    }
}

/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////

// Add the geojson LineString route to the map, this is the theoretical route of the boat
function addRouteToMap(routeGeoJSON) {
    const coordinates = routeGeoJSON.features[0].geometry.coordinates;
    const waypoints = coordinates.map((waypoint) => ({
        lat: waypoint[1],
        lon: waypoint[0],
        radius_m: 5.0
    }));

    if (typeof clearRoute === "function" && typeof addWaypoint === "function") {
        clearRoute();
        waypoints.forEach((waypoint) => {
            addWaypoint(waypoint.lat, waypoint.lon, waypoint.radius_m);
        });
    }

    return waypoints;
}
