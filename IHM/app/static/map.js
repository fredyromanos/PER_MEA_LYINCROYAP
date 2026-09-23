console.log("map.js chargé");

// ============================================================
// Configuration
// ============================================================

const API_BASE = "/api";
const DEFAULT_WAYPOINT_RADIUS_M = 5.0;

const state = {
    map: null,
    waypoints: [],
    waypointMarkers: [],
    routeLine: null,
    // Prédiction de trajectoire
    showTrajectory: false,
    trajectoryLine: null,
    lastTelemetry: null      // {lat, lon, heading, wind, wpIndex}
};


// ============================================================
// Initialisation générale
// ============================================================

document.addEventListener("DOMContentLoaded", () => {
    console.log("DOM chargé");

    initMap();
    bindUiEvents();
});


// ============================================================
// Initialisation carte Leaflet
// ============================================================

function initMap() {
    const mapContainer = document.getElementById("map");

    if (!mapContainer) {
        console.error("Erreur : élément HTML #map introuvable.");
        return;
    }

    if (typeof L === "undefined") {
        console.error("Erreur : Leaflet n'est pas chargé.");
        return;
    }
    

    // Vue initiale sur l'eau (Rade de Brest, point de départ des simulations) ;
    // la carte se recentre de toute façon sur la première position reçue du bateau.
    state.map = L.map("map").setView([48.340, -4.520], 16);
    window.autoboatMap = state.map;

    // Libère le coin sup. gauche pour la boussole de vent.
    state.map.zoomControl.setPosition("topright");

    L.tileLayer("https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png", {
        maxZoom: 19,
        attribution: "&copy; OpenStreetMap contributors"
    }).addTo(state.map);

    state.map.on("click", (event) => {
        addWaypoint(
            event.latlng.lat,
            event.latlng.lng,
            DEFAULT_WAYPOINT_RADIUS_M
        );
    });

    console.log("Carte Leaflet initialisée");
}


// ============================================================
// addEventListener sur les boutons HTML
// ============================================================

function bindUiEvents() {
    const sendRouteBtn = document.getElementById("send-route-btn");
    const clearRouteBtn = document.getElementById("clear-route-btn");

    if (sendRouteBtn) {
        sendRouteBtn.addEventListener("click", sendRouteToBoat);
    } else {
        console.warn("Bouton #send-route-btn introuvable.");
    }

    if (clearRouteBtn) {
        clearRouteBtn.addEventListener("click", clearRoute);
    } else {
        console.warn("Bouton #clear-route-btn introuvable.");
    }

    const trajectoryBtn = document.getElementById("trajectoryButton");
    if (trajectoryBtn) {
        trajectoryBtn.addEventListener("click", () => {
            setTrajectoryEnabled(!state.showTrajectory);
            trajectoryBtn.classList.toggle("btn-primary", state.showTrajectory);
        });
    }

    // Touche Échap → efface TOUS les waypoints sélectionnés.
    document.addEventListener("keydown", (event) => {
        if (event.key !== "Escape") {
            return;
        }
        // Ne pas interférer avec une saisie en cours.
        const tag = (document.activeElement && document.activeElement.tagName) || "";
        if (tag === "INPUT" || tag === "TEXTAREA") {
            return;
        }
        if (state.waypoints.length > 0) {
            clearRoute();
            console.log("Waypoints effacés (Échap)");
        }
    });

    console.log("Événements UI connectés");
}


// ============================================================
// Prédiction de trajectoire (serveur : predict_path, code firmware)
// ============================================================

function setTrajectoryEnabled(on) {
    state.showTrajectory = on;
    if (on) {
        renderTrajectory();
    } else {
        clearTrajectory();
    }
}

function clearTrajectory() {
    if (state.trajectoryLine && state.map) {
        state.map.removeLayer(state.trajectoryLine);
    }
    state.trajectoryLine = null;
}

// Appelée à chaque télémétrie depuis script.js.
function feedTelemetry(lat, lon, heading, wind, wpIndex, headingValid = true) {
    state.lastTelemetry = { lat, lon, heading, wind, wpIndex, headingValid };
    if (state.showTrajectory) {
        renderTrajectory();
    }
}

// Prédiction calculée côté serveur (/api/predict → AutoBoat_Simulation/predict_path) avec
// le VRAI navigation.h du firmware et la physique de la simulation statique. L'ancienne
// réplique JS (nav.js) avait divergé du firmware et prédisait des détours inexistants.
let trajectoryRequestId = 0;
let trajectoryInFlight = false;

async function renderTrajectory() {
    if (!state.map || !state.showTrajectory) {
        return;
    }

    const t = state.lastTelemetry;
    if (!t || t.lat === undefined || (t.lat === 0 && t.lon === 0)) {
        clearTrajectory();
        return;   // pas de position bateau
    }
    if (t.headingValid === false) {
        clearTrajectory();
        return;   // pas de cap fiable : une prédiction serait fausse
    }
    // Route restante à partir du waypoint courant (index "wc" de la télémétrie).
    const idx = (typeof t.wpIndex === "number") ? t.wpIndex : 0;
    if (!state.waypoints || idx < 0 || idx >= state.waypoints.length) {
        clearTrajectory();
        return;   // pas de route chargée / index hors limites
    }
    if (trajectoryInFlight) {
        return;   // une prédiction est déjà en cours (télémétrie à 1 Hz)
    }

    const requestId = ++trajectoryRequestId;
    trajectoryInFlight = true;
    try {
        const response = await fetch("/api/predict", {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify({
                lat: t.lat, lon: t.lon, heading: Number(t.heading) || 0, wind: Number(t.wind) || 0,
                waypoints: state.waypoints.slice(idx).map((w) => ({ lat: w.lat, lon: w.lon }))
            })
        });
        const result = await response.json();
        if (requestId !== trajectoryRequestId || !state.showTrajectory) {
            return;   // réponse périmée
        }
        clearTrajectory();
        if (!response.ok || !result.path || result.path.length < 2) {
            console.warn("Prédiction indisponible :", result);
            return;
        }
        state.trajectoryLine = L.polyline(result.path, {
            color: "#8e44ad",
            weight: 3,
            opacity: 0.85,
            dashArray: result.reached ? "6, 6" : "2, 8",   // pointillé serré = route non atteinte
            clickable: false
        }).addTo(state.map);
    } catch (err) {
        console.warn("Erreur prédiction :", err);
    } finally {
        trajectoryInFlight = false;
    }
}


// ============================================================
// Gestion des waypoints
// ============================================================

function addWaypoint(lat, lon, radiusM = DEFAULT_WAYPOINT_RADIUS_M) {
    const waypoint = {
        lat: lat,
        lon: lon,
        radius_m: radiusM
    };

    state.waypoints.push(waypoint);

    updateRouteDisplay();
    updateWaypointList();

    console.log("Waypoint ajouté :", waypoint);
}


function removeWaypoint(index) {
    if (index < 0 || index >= state.waypoints.length) {
        console.warn("Index waypoint invalide :", index);
        return;
    }

    state.waypoints.splice(index, 1);

    updateRouteDisplay();
    updateWaypointList();
}


function clearRoute() {
    state.waypoints = [];

    clearMapRouteDisplay();
    updateWaypointList();
    clearTrajectory();   // la trajectoire prévue n'a plus de cible

    console.log("Route effacée");
}


// ============================================================
// Affichage carte
// ============================================================

function updateRouteDisplay() {
    if (!state.map) {
        console.error("Carte non initialisée.");
        return;
    }

    clearMapRouteDisplay();

    state.waypoints.forEach((waypoint, index) => {
        const marker = L.marker([waypoint.lat, waypoint.lon])
            .addTo(state.map)
            .bindPopup(`
                <strong>Waypoint ${index + 1}</strong><br>
                Lat : ${waypoint.lat.toFixed(8)}<br>
                Lon : ${waypoint.lon.toFixed(8)}<br>
                Rayon : ${waypoint.radius_m.toFixed(1)} m
            `);

        state.waypointMarkers.push(marker);
    });

    if (state.waypoints.length >= 2) {
        const coordinates = state.waypoints.map((wp) => [wp.lat, wp.lon]);

        state.routeLine = L.polyline(coordinates, {
            weight: 4
        }).addTo(state.map);
    }
}


function clearMapRouteDisplay() {
    if (!state.map) {
        return;
    }

    state.waypointMarkers.forEach((marker) => {
        state.map.removeLayer(marker);
    });

    state.waypointMarkers = [];

    if (state.routeLine !== null) {
        state.map.removeLayer(state.routeLine);
        state.routeLine = null;
    }
}


// ============================================================
// Liste HTML des waypoints
// ============================================================

function updateWaypointList() {
    const list = document.getElementById("waypoints-list");

    if (!list) {
        return;
    }

    list.innerHTML = "";

    state.waypoints.forEach((waypoint, index) => {
        const item = document.createElement("li");

        const text = document.createElement("span");
        text.textContent =
            `WP${index + 1}: ${waypoint.lat.toFixed(5)}, ${waypoint.lon.toFixed(5)}`;

        const deleteButton = document.createElement("button");
        deleteButton.textContent = "✖";
        deleteButton.title = "Supprimer ce waypoint";
        deleteButton.addEventListener("click", () => {
            removeWaypoint(index);
        });

        item.appendChild(text);
        item.appendChild(deleteButton);

        list.appendChild(item);
    });
}


// ============================================================
// Communication backend
// ============================================================

async function sendRouteToBoat() {
    if (state.waypoints.length === 0) {
        alert("Aucun waypoint à envoyer.");
        return;
    }

    try {
        const response = await fetch(`${API_BASE}/send-route`, {
            method: "POST",
            headers: {
                "Content-Type": "application/json"
            },
            body: JSON.stringify({
                waypoints: state.waypoints
            })
        });

        const result = await response.json();

        if (!response.ok) {
            console.error("Erreur backend :", result);
            alert("Erreur lors de l'envoi de la route.");
            return;
        }

        console.log("Route envoyée :", result);
        alert("Route envoyée au bateau.");

    } catch (error) {
        console.error("Erreur réseau :", error);
        alert("Impossible de contacter le serveur.");
    }
}


async function startNavigation() {
    try {
        const response = await fetch(`${API_BASE}/navigate`, {
            method: "GET"
        });

        const result = await response.json();

        if (!response.ok) {
            console.error("Erreur backend :", result);
            alert("Erreur lors du démarrage de la navigation.");
            return;
        }

        console.log("Navigation démarrée :", result);
        alert("Navigation démarrée.");

    } catch (error) {
        console.error("Erreur réseau :", error);
        alert("Impossible de démarrer la navigation.");
    }
}


async function startSystem() {
    try {
        const response = await fetch(`${API_BASE}/start`, {
            method: "POST"
        });

        const result = await response.json();

        if (!response.ok) {
            console.error("Erreur start :", result);
            alert("Erreur lors du lancement système.");
            return;
        }

        console.log("Start OK :", result);
        alert("Système lancé.");

    } catch (error) {
        console.error("Erreur réseau :", error);
        alert("Impossible de lancer le système.");
    }
}


async function restartBoat() {
    try {
        const response = await fetch(`${API_BASE}/restart`, {
            method: "GET"
        });

        const result = await response.json();

        if (!response.ok) {
            console.error("Erreur restart :", result);
            alert("Erreur lors du redémarrage bateau.");
            return;
        }

        console.log("Restart envoyé :", result);
        alert("Commande restart envoyée.");

    } catch (error) {
        console.error("Erreur réseau :", error);
        alert("Impossible de redémarrer le bateau.");
    }
}