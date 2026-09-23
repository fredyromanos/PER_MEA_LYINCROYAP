#include "sim_html_export.hpp"
#include <iostream>
#include <iomanip>
#include <sstream>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265359
#endif

void HTMLExporter::exportAllScenarios(
    const std::string& filename,
    const std::vector<ScenarioData>& scenarios)
{
    if (scenarios.empty()) {
        std::cerr << "[HTML] No scenarios to export." << std::endl;
        return;
    }

    std::ofstream f(filename);
    if (!f.is_open()) {
        std::cerr << "[HTML] Error: cannot open " << filename << std::endl;
        return;
    }

    writeHeader(f);
    writeStyles(f);
    f << "</head>\n<body>\n";
    writeHTMLBody(f, scenarios);
    f << "<script>\n";
    writeAllScenariosJSON(f, scenarios);
    writeJavaScript(f);
    f << "</script>\n";
    writeFooter(f);

    f.close();
    std::cout << "[HTML] Interactive map exported: " << filename 
              << " (" << scenarios.size() << " scenarios)" << std::endl;
}

// ─────────────────────────────────────────────
// HTML <head>
// ─────────────────────────────────────────────
void HTMLExporter::writeHeader(std::ofstream& f) {
    f << R"(<!DOCTYPE html>
<html lang="fr">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>AutoBoat - Simulation Multi-Scénarios</title>
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css" />
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
)";
}

// ─────────────────────────────────────────────
// CSS
// ─────────────────────────────────────────────
void HTMLExporter::writeStyles(std::ofstream& f) {
    f << R"(<style>
* { margin:0; padding:0; box-sizing:border-box; }
body { font-family: 'Segoe UI', system-ui, sans-serif; background:#1a1a2e; color:#eee; overflow:hidden; }
#map { position:absolute; top:0; left:0; width:100%; height:100%; z-index:1; }

/* ── Barre scénarios (haut) ── */
#scenario-bar {
    position:absolute; top:0; left:0; right:0; z-index:2000;
    background:rgba(15,15,30,0.97); display:flex; align-items:center;
    padding:0 12px; height:44px; gap:4px;
    border-bottom:1px solid rgba(255,255,255,0.08);
    backdrop-filter:blur(10px);
}
#scenario-bar .logo { font-size:15px; font-weight:700; color:#8be9fd; margin-right:16px; letter-spacing:1px; }
.scenario-tab {
    background:none; border:none; color:#888; padding:8px 16px; cursor:pointer;
    font-size:12px; font-weight:600; letter-spacing:0.5px; text-transform:uppercase;
    border-radius:8px 8px 0 0; transition: all 0.2s;
    border-bottom:2px solid transparent; position:relative;
}
.scenario-tab:hover { color:#ccc; background:rgba(255,255,255,0.05); }
.scenario-tab.active { color:#8be9fd; border-bottom-color:#8be9fd; background:rgba(139,233,253,0.08); }

/* ── Panneau info ── */
#info-panel {
    position:absolute; top:56px; right:12px; z-index:1000;
    background:rgba(20,20,40,0.92); border-radius:12px; padding:16px 20px;
    min-width:260px; backdrop-filter:blur(8px);
    border:1px solid rgba(255,255,255,0.1);
    box-shadow:0 8px 32px rgba(0,0,0,0.4);
}
#info-panel h2 { font-size:14px; color:#8be9fd; margin-bottom:10px; letter-spacing:1px; text-transform:uppercase; }
.info-row { display:flex; justify-content:space-between; padding:3px 0; font-size:13px; }
.info-label { color:#999; }
.info-value { color:#fff; font-weight:600; font-variant-numeric:tabular-nums; }
.info-divider { border-top:1px solid rgba(255,255,255,0.08); margin:8px 0; }

/* ── Mode badge ── */
.mode-badge {
    display:inline-block; padding:2px 10px; border-radius:10px; font-size:11px;
    font-weight:700; letter-spacing:0.5px; text-transform:uppercase;
}
.mode-vdb      { background:#ff5555; color:#fff; }
.mode-lofer    { background:#50fa7b; color:#222; }
.mode-abattre  { background:#ffb86c; color:#222; }
.mode-obs      { background:#6272a4; color:#fff; }
.mode-standby  { background:#44475a; color:#999; }

/* ── Jauges ── */
#gauges {
    position:absolute; bottom:80px; right:12px; z-index:1000;
    background:rgba(20,20,40,0.92); border-radius:12px; padding:14px 18px;
    min-width:220px; backdrop-filter:blur(8px);
    border:1px solid rgba(255,255,255,0.1);
}
.gauge { margin:8px 0; }
.gauge-label { font-size:11px; color:#999; margin-bottom:3px; }
.gauge-bar-bg {
    width:100%; height:14px; background:#2a2a4a; border-radius:7px; position:relative; overflow:hidden;
}
.gauge-bar {
    height:100%; border-radius:7px; transition:width 0.15s, background 0.15s;
    position:absolute; top:0;
}
.gauge-value { font-size:11px; color:#ccc; text-align:center; margin-top:2px; }

/* ── Vent ── */
#wind-indicator {
    position:absolute; top:56px; left:60px; z-index:1000;
    background:rgba(20,20,40,0.92); border-radius:12px; padding:14px;
    text-align:center; backdrop-filter:blur(8px);
    border:1px solid rgba(255,255,255,0.1);
}
#wind-indicator .label { font-size:10px; color:#999; text-transform:uppercase; letter-spacing:1px; }
#wind-arrow {
    width:50px; height:50px; margin:6px auto;
    transition: transform 0.3s ease;
}
#wind-speed-val { font-size:13px; font-weight:600; }
#wind-dir-val { font-size:11px; color:#999; }

/* ── Contrôles ── */
#controls {
    position:absolute; bottom:16px; left:50%; transform:translateX(-50%); z-index:1000;
    background:rgba(20,20,40,0.95); border-radius:14px; padding:12px 24px;
    display:flex; align-items:center; gap:14px; backdrop-filter:blur(8px);
    border:1px solid rgba(255,255,255,0.1);
    box-shadow:0 8px 32px rgba(0,0,0,0.5);
}
#controls button {
    background:none; border:1px solid rgba(255,255,255,0.2); color:#fff;
    border-radius:8px; padding:6px 14px; cursor:pointer; font-size:14px;
    transition:background 0.2s;
}
#controls button:hover { background:rgba(255,255,255,0.1); }
#controls button.active { background:#8be9fd; color:#1a1a2e; border-color:#8be9fd; }
#timeline {
    width:320px; accent-color:#8be9fd; cursor:pointer;
}
#time-display { font-size:13px; font-variant-numeric:tabular-nums; min-width:70px; text-align:center; }
.speed-btn { min-width:40px; text-align:center; font-size:12px !important; }
.speed-btn.active { background:#ff79c6 !important; border-color:#ff79c6 !important; }

/* ── Légende ── */
#legend {
    position:absolute; bottom:80px; left:12px; z-index:1000;
    background:rgba(20,20,40,0.92); border-radius:12px; padding:12px 16px;
    backdrop-filter:blur(8px); border:1px solid rgba(255,255,255,0.1);
}
#legend h3 { font-size:11px; color:#999; text-transform:uppercase; letter-spacing:1px; margin-bottom:6px; }
.legend-item { display:flex; align-items:center; gap:8px; font-size:12px; padding:2px 0; }
.legend-color { width:24px; height:4px; border-radius:2px; }
</style>
)";
}

// ─────────────────────────────────────────────
// Embedded JSON data — ALL scenarios
// ─────────────────────────────────────────────
void HTMLExporter::writeAllScenariosJSON(std::ofstream& f,
                                          const std::vector<ScenarioData>& scenarios) {
    f << "const SCENARIOS = [\n";
    
    for (size_t sc = 0; sc < scenarios.size(); sc++) {
        const auto& s = scenarios[sc];
        if (sc > 0) f << ",\n";
        
        f << "{\n";
        f << "name: \"" << s.name << "\",\n";
        f << "windDir: " << std::fixed << std::setprecision(2) << s.windDir << ",\n";
        f << "windSpeed: " << s.windSpeed << ",\n";
        
        // ── Trajectory data ──
        f << "data: [\n";
        size_t step = 1;
        if (s.history.size() > 2000) step = s.history.size() / 2000;
        
        bool first = true;
        for (size_t i = 0; i < s.history.size(); i += step) {
            const auto& h = s.history[i];
            if (!first) f << ",\n";
            first = false;

            std::string mode;
            switch(h.navMode) {
                case 1: mode = "vdb"; break;
                case 2: mode = "direct"; break;
                case 3: mode = "lofer"; break;
                case 4: mode = "abattre"; break;
                case 6: mode = "upwind-zigzag"; break;
                case 7: mode = "downwind-zigzag"; break;
                case 8: mode = "avoid-gybe"; break;
                default: mode = "observation"; break;
            }

            f << std::fixed
              << "{t:" << h.time
              << ",lat:" << std::setprecision(8) << h.latitude
              << ",lng:" << h.longitude
              << ",hdg:" << std::setprecision(2) << h.heading
              << ",spd:" << h.speed
              << ",sail:" << h.sailAngle
              << ",rud:" << h.rudderAngle
              << ",wd:" << h.windDirection
              << ",ws:" << h.windSpeed
              << ",m:\"" << mode << "\"}";
        }
        // Always include the last point
        if (step > 1 && !s.history.empty()) {
            const auto& h = s.history.back();
            std::string mode;
            switch(h.navMode) {
                case 1: mode = "vdb"; break;
                case 2: mode = "direct"; break;
                case 3: mode = "lofer"; break;
                case 4: mode = "abattre"; break;
                case 6: mode = "upwind-zigzag"; break;
                case 7: mode = "downwind-zigzag"; break;
                case 8: mode = "avoid-gybe"; break;
                default: mode = "observation"; break;
            }
            f << ",\n" << std::fixed
              << "{t:" << h.time
              << ",lat:" << std::setprecision(8) << h.latitude
              << ",lng:" << h.longitude
              << ",hdg:" << std::setprecision(2) << h.heading
              << ",spd:" << h.speed
              << ",sail:" << h.sailAngle
              << ",rud:" << h.rudderAngle
              << ",wd:" << h.windDirection
              << ",ws:" << h.windSpeed
              << ",m:\"" << mode << "\"}";
        }
        f << "\n],\n";
        
        // ── Waypoints ──
        f << "waypoints: [\n";
        for (size_t i = 0; i < s.waypoints.size(); i++) {
            if (i > 0) f << ",\n";
            f << std::fixed << std::setprecision(8)
              << "{lat:" << s.waypoints[i].first << ",lng:" << s.waypoints[i].second
              << ",id:" << i << "}";
        }
        f << "\n]\n";
        
        f << "}";  // end scenario object
    }
    
    f << "\n];\n\n";
    f << "let currentScenarioIdx = 0;\n\n";
}

// ─────────────────────────────────────────────
// HTML body structure
// ─────────────────────────────────────────────
void HTMLExporter::writeHTMLBody(std::ofstream& f, const std::vector<ScenarioData>& scenarios) {
    // Scenario tabs bar
    f << "<div id=\"scenario-bar\">\n";
    f << "  <span class=\"logo\">&#9973; AutoBoat</span>\n";
    for (size_t i = 0; i < scenarios.size(); i++) {
        f << "  <button class=\"scenario-tab" << (i == 0 ? " active" : "") 
          << "\" onclick=\"switchScenario(" << i << ")\" id=\"tab-" << i << "\">"
          << scenarios[i].name << "</button>\n";
    }
    f << "</div>\n";
    
    f << "<div id=\"map\"></div>\n";
    
    // Info panel
    f << "<div id=\"info-panel\">\n";
    f << "  <h2 id=\"scenario-title\">" << scenarios[0].name << "</h2>\n";
    f << "  <div class=\"info-row\"><span class=\"info-label\">Temps</span><span class=\"info-value\" id=\"v-time\">0.0s</span></div>\n";
    f << "  <div class=\"info-row\"><span class=\"info-label\">Position</span><span class=\"info-value\" id=\"v-pos\">--</span></div>\n";
    f << "  <div class=\"info-row\"><span class=\"info-label\">Cap</span><span class=\"info-value\" id=\"v-heading\">--</span></div>\n";
    f << "  <div class=\"info-row\"><span class=\"info-label\">Vitesse</span><span class=\"info-value\" id=\"v-speed\">--</span></div>\n";
    f << "  <div class=\"info-divider\"></div>\n";
    f << "  <div class=\"info-row\"><span class=\"info-label\">Mode</span><span id=\"v-mode\"><span class=\"mode-badge mode-obs\">--</span></span></div>\n";
    f << "  <div class=\"info-row\"><span class=\"info-label\">Waypoint</span><span class=\"info-value\" id=\"v-wpt\">--</span></div>\n";
    f << "  <div class=\"info-row\"><span class=\"info-label\">Distance WPT</span><span class=\"info-value\" id=\"v-dist\">--</span></div>\n";
    f << "</div>\n";
    
    // Wind indicator
    f << "<div id=\"wind-indicator\">\n";
    f << "  <div class=\"label\">Vent</div>\n";
    f << "  <svg id=\"wind-arrow\" viewBox=\"0 0 50 50\">\n";
    f << "    <defs><marker id=\"arrowhead\" markerWidth=\"6\" markerHeight=\"4\" refX=\"3\" refY=\"2\" orient=\"auto\">\n";
    f << "      <polygon points=\"0 0, 6 2, 0 4\" fill=\"#8be9fd\"/>\n";
    f << "    </marker></defs>\n";
    f << "    <line x1=\"25\" y1=\"42\" x2=\"25\" y2=\"10\" stroke=\"#8be9fd\" stroke-width=\"2.5\" marker-end=\"url(#arrowhead)\"/>\n";
    f << "  </svg>\n";
    f << "  <div id=\"wind-speed-val\">-- m/s</div>\n";
    f << "  <div id=\"wind-dir-val\">--&deg;</div>\n";
    f << "</div>\n";
    
    // Gauges
    f << "<div id=\"gauges\">\n";
    f << "  <div class=\"gauge\">\n";
    f << "    <div class=\"gauge-label\">&#9660; Gouvernail</div>\n";
    f << "    <div class=\"gauge-bar-bg\">\n";
    f << "      <div class=\"gauge-bar\" id=\"rudder-bar\" style=\"width:50%;left:50%;background:#ff5555;\"></div>\n";
    f << "      <div style=\"position:absolute;left:50%;top:0;width:1px;height:100%;background:rgba(255,255,255,0.3);\"></div>\n";
    f << "    </div>\n";
    f << "    <div class=\"gauge-value\" id=\"rudder-val\">0&deg;</div>\n";
    f << "  </div>\n";
    f << "  <div class=\"gauge\">\n";
    f << "    <div class=\"gauge-label\">&#9973; Voile</div>\n";
    f << "    <div class=\"gauge-bar-bg\">\n";
    f << "      <div class=\"gauge-bar\" id=\"sail-bar\" style=\"width:50%;left:50%;background:#50fa7b;\"></div>\n";
    f << "      <div style=\"position:absolute;left:50%;top:0;width:1px;height:100%;background:rgba(255,255,255,0.3);\"></div>\n";
    f << "    </div>\n";
    f << "    <div class=\"gauge-value\" id=\"sail-val\">0&deg;</div>\n";
    f << "  </div>\n";
    f << "  <div class=\"gauge\">\n";
    f << "    <div class=\"gauge-label\">&#128168; Vitesse</div>\n";
    f << "    <div class=\"gauge-bar-bg\">\n";
    f << "      <div class=\"gauge-bar\" id=\"speed-bar\" style=\"width:0%;left:0%;background:#ffb86c;\"></div>\n";
    f << "    </div>\n";
    f << "    <div class=\"gauge-value\" id=\"speed-val\">0 m/s</div>\n";
    f << "  </div>\n";
    f << "</div>\n";
    
    // Legend
    f << "<div id=\"legend\">\n";
    f << "  <h3>Navigation</h3>\n";
    f << "  <div class=\"legend-item\"><span class=\"legend-color\" style=\"background:#6272a4\"></span> Observation</div>\n";
    f << "  <div class=\"legend-item\"><span class=\"legend-color\" style=\"background:#ff5555\"></span> VDB (virement)</div>\n";
    f << "  <div class=\"legend-item\"><span class=\"legend-color\" style=\"background:#50fa7b\"></span> LOFER (remonter)</div>\n";
    f << "  <div class=\"legend-item\"><span class=\"legend-color\" style=\"background:#ffb86c\"></span> ABATTRE (descendre)</div>\n";
    f << "  <h3 style=\"margin-top:8px;\">Bateau</h3>\n";
    f << "  <div class=\"legend-item\"><span class=\"legend-color\" style=\"background:#f1fa8c\"></span> Voile</div>\n";
    f << "  <div class=\"legend-item\"><span class=\"legend-color\" style=\"background:#ff5555\"></span> Gouvernail</div>\n";
    f << "</div>\n";
    
    // Controls
    f << "<div id=\"controls\">\n";
    f << "  <button id=\"btn-play\" onclick=\"togglePlay()\">&#9654;</button>\n";
    f << "  <input type=\"range\" id=\"timeline\" min=\"0\" max=\"100\" value=\"0\" oninput=\"onSlider(this.value)\">\n";
    f << "  <span id=\"time-display\">0.0s</span>\n";
    f << "  <button class=\"speed-btn\" onclick=\"setSpeed(1)\" id=\"sp1\">&times;1</button>\n";
    f << "  <button class=\"speed-btn\" onclick=\"setSpeed(2)\" id=\"sp2\">&times;2</button>\n";
    f << "  <button class=\"speed-btn\" onclick=\"setSpeed(5)\" id=\"sp5\">&times;5</button>\n";
    f << "  <button class=\"speed-btn active\" onclick=\"setSpeed(10)\" id=\"sp10\">&times;10</button>\n";
    f << "  <button onclick=\"toggleTrail()\">&#128065; Trail</button>\n";
    f << "  <button id=\"btn-wind\" onclick=\"toggleWindOverlay()\">&#127788; Vent</button>\n";
    f << "  <button onclick=\"resetView()\">&#8962;</button>\n";
    f << "</div>\n";
}

// ─────────────────────────────────────────────
// JavaScript (Leaflet + animation logic)
// ─────────────────────────────────────────────
void HTMLExporter::writeJavaScript(std::ofstream& f) {
    f << R"JS(
// ═══════════════════════════════════════════
//  AutoBoat Multi-Scenario Visualization
// ═══════════════════════════════════════════

const MODE_COLORS = {
    observation: '#6272a4',
    vdb:         '#ff5555',
    lofer:       '#50fa7b',
    abattre:     '#ffb86c',
    direct:      '#bd93f9',
    'upwind-zigzag': '#8be9fd',
    'downwind-zigzag': '#f1fa8c',
    'avoid-gybe': '#ff79c6',
    standby:     '#44475a'
};

// ── Active scenario shortcuts ──
function S() { return SCENARIOS[currentScenarioIdx]; }
function DATA() { return S().data; }
function WAYPOINTS() { return S().waypoints; }

// ── Map init ──
const map = L.map('map', { zoomControl: false });
L.control.zoom({ position: 'bottomleft' }).addTo(map);

const tiles = L.tileLayer('https://{s}.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}{r}.png', {
    attribution: '&copy; <a href="https://carto.com/">CARTO</a> &copy; <a href="https://www.openstreetmap.org/copyright">OSM</a>',
    maxZoom: 20, subdomains: 'abcd'
}).addTo(map);

tiles.on('tileerror', function() {
    map.eachLayer(l => { if (l instanceof L.TileLayer) map.removeLayer(l); });
    L.tileLayer('https://tile.openstreetmap.de/{z}/{x}/{y}.png', {
        attribution: '&copy; OpenStreetMap', maxZoom: 19
    }).addTo(map);
});

// ── Dynamic layers ──
let wptMarkers = [];
let startMarker = null;
let trailSegments = [];
let trailVisible = true;
let animTrail = null;
let boatMarker = null;
let headingLine = null;
let windLine = null;
let wptLine = null;
let sailLine = null;
let rudderLine = null;
let windOverlayVisible = true;

// ── Animation state ──
let currentFrame = 0;
let playing = false;
let playSpeed = 10;
let animTimer = null;

const slider = document.getElementById('timeline');

// ── Boat icon ──
function boatIcon(heading) {
    const svg = `<svg width="32" height="32" viewBox="0 0 32 32" xmlns="http://www.w3.org/2000/svg">
        <g transform="rotate(${heading}, 16, 16)">
            <polygon points="16,2 24,28 16,22 8,28" fill="#8be9fd" stroke="#fff" stroke-width="1.5" opacity="0.9"/>
        </g>
    </svg>`;
    return L.divIcon({ html: svg, className: '', iconSize: [32, 32], iconAnchor: [16, 16] });
}

// ── Compute map bounds for active scenario ──
function scenarioBounds() {
    const data = DATA();
    if (!data.length) return [[48.33, -4.53], [48.35, -4.51]];
    const b = data.reduce((b, d) => {
        b.minLat = Math.min(b.minLat, d.lat);
        b.maxLat = Math.max(b.maxLat, d.lat);
        b.minLng = Math.min(b.minLng, d.lng);
        b.maxLng = Math.max(b.maxLng, d.lng);
        return b;
    }, {minLat:90, maxLat:-90, minLng:180, maxLng:-180});
    return [[b.minLat - 0.002, b.minLng - 0.002], [b.maxLat + 0.002, b.maxLng + 0.002]];
}

// ── Clear all dynamic layers ──
function clearLayers() {
    wptMarkers.forEach(m => map.removeLayer(m));
    wptMarkers = [];
    if (startMarker) { map.removeLayer(startMarker); startMarker = null; }
    trailSegments.forEach(s => map.removeLayer(s));
    trailSegments = [];
    if (animTrail) { map.removeLayer(animTrail); animTrail = null; }
    if (boatMarker) { map.removeLayer(boatMarker); boatMarker = null; }
    if (headingLine) { map.removeLayer(headingLine); headingLine = null; }
    if (windLine) { map.removeLayer(windLine); windLine = null; }
    if (wptLine) { map.removeLayer(wptLine); wptLine = null; }
    if (sailLine) { map.removeLayer(sailLine); sailLine = null; }
    if (rudderLine) { map.removeLayer(rudderLine); rudderLine = null; }
}

// ── Build layers for active scenario ──
function buildScenarioLayers() {
    const data = DATA();
    const wpts = WAYPOINTS();

    // Waypoint markers
    wpts.forEach(w => {
        const m = L.circleMarker([w.lat, w.lng], {
            radius: 10, color: '#ff79c6', fillColor: '#ff79c6', fillOpacity: 0.3, weight: 2
        }).addTo(map).bindTooltip('WPT ' + w.id, { permanent: true, direction: 'top', className: '' });
        wptMarkers.push(m);
    });

    // Start marker
    if (data.length > 0) {
        startMarker = L.circleMarker([data[0].lat, data[0].lng], {
            radius: 7, color: '#8be9fd', fillColor: '#8be9fd', fillOpacity: 0.7, weight: 2
        }).addTo(map).bindTooltip('Start', { direction: 'bottom' });
    }

    // Trail
    buildTrail();

    // Animated trail
    animTrail = L.polyline([], { color: '#fff', weight: 2, opacity: 0.5, dashArray: '4 6' }).addTo(map);

    // Boat marker
    if (data.length > 0) {
        boatMarker = L.marker([data[0].lat, data[0].lng], { icon: boatIcon(data[0].hdg), zIndexOffset: 1000 }).addTo(map);
    }

    // Lines
    headingLine = L.polyline([], { color: '#8be9fd', weight: 1.5, opacity: 0.6, dashArray: '6 4' }).addTo(map);
    windLine = L.polyline([], { color: '#8be9fd', weight: 2, opacity: 0.4, dashArray: '3 5' });
    if (windOverlayVisible) windLine.addTo(map);
    wptLine = L.polyline([], { color: '#ff79c6', weight: 2, opacity: 0.5, dashArray: '6 4' }).addTo(map);
    sailLine = L.polyline([], { color: '#f1fa8c', weight: 3, opacity: 0.85 }).addTo(map);
    rudderLine = L.polyline([], { color: '#ff5555', weight: 2.5, opacity: 0.85 }).addTo(map);

    // Slider
    slider.max = Math.max(0, data.length - 1);
    slider.value = 0;
}

// ── Build trail segments colored by mode ──
function buildTrail() {
    trailSegments.forEach(s => map.removeLayer(s));
    trailSegments = [];
    const data = DATA();
    if (data.length < 2) return;

    let segStart = 0;
    for (let i = 1; i < data.length; i++) {
        if (data[i].m !== data[segStart].m || i === data.length - 1) {
            const end = (i === data.length - 1) ? i + 1 : i + 1;
            const pts = data.slice(segStart, end).map(d => [d.lat, d.lng]);
            const color = MODE_COLORS[data[segStart].m] || '#666';
            const line = L.polyline(pts, { color, weight: 3, opacity: 0.7 });
            if (trailVisible) line.addTo(map);
            trailSegments.push(line);
            segStart = i;
        }
    }
}

// ═══════════════════════════════════════════
//  SWITCH SCENARIO
// ═══════════════════════════════════════════
function switchScenario(idx) {
    if (idx === currentScenarioIdx) return;
    
    // Stop playback
    stopAnim();
    playing = false;
    document.getElementById('btn-play').textContent = '▶';
    document.getElementById('btn-play').classList.remove('active');
    
    // Update tabs
    document.querySelectorAll('.scenario-tab').forEach(t => t.classList.remove('active'));
    document.getElementById('tab-' + idx).classList.add('active');
    
    currentScenarioIdx = idx;
    currentFrame = 0;
    
    // Rebuild
    clearLayers();
    buildScenarioLayers();
    map.fitBounds(scenarioBounds());
    
    // Update title
    document.getElementById('scenario-title').textContent = S().name;
    
    updateFrame(0);
}

// ═══════════════════════════════════════════
//  UPDATE FRAME
// ═══════════════════════════════════════════
function updateFrame(i) {
    const data = DATA();
    if (i < 0 || i >= data.length) return;
    currentFrame = i;
    const d = data[i];

    // Boat position & icon
    if (boatMarker) {
        boatMarker.setLatLng([d.lat, d.lng]);
        boatMarker.setIcon(boatIcon(d.hdg));
    }

    // Heading line
    if (headingLine) {
        const dist = 0.001;
        const hdgRad = d.hdg * Math.PI / 180;
        headingLine.setLatLngs([
            [d.lat, d.lng],
            [d.lat + dist * Math.cos(hdgRad), d.lng + dist * Math.sin(hdgRad)]
        ]);
    }

    // Sail line (yellow) — rigid wing sail (weathervane + aileron)
    // The sail is free to rotate: it aligns with the wind flow.
    // The aileron (d.sail = ±10°) deflects the TRAILING EDGE,
    // which tilts the wing body in the OPPOSITE direction.
    // Wind blows FROM d.wd, so wind FLOW goes towards (d.wd + 180)°.
    if (sailLine) {
        const windFlowDeg = (d.wd + 180) % 360;
        const sailDeg = windFlowDeg - d.sail;  // body tilts opposite to aileron
        const sailRad = sailDeg * Math.PI / 180;
        const sLen = 0.0006;
        sailLine.setLatLngs([
            [d.lat, d.lng],
            [d.lat + sLen * Math.cos(sailRad), d.lng + sLen * Math.sin(sailRad)]
        ]);
    }

    // Rudder line (red) - at the back of the boat, offset by rudderAngle
    if (rudderLine) {
        const backRad = (d.hdg + 180) * Math.PI / 180;
        const backD = 0.00012;
        const bLat = d.lat + backD * Math.cos(backRad);
        const bLng = d.lng + backD * Math.sin(backRad);
        const rudDeg = d.hdg + 180 + d.rud;
        const rudRad = rudDeg * Math.PI / 180;
        const rLen = 0.0003;
        rudderLine.setLatLngs([
            [bLat, bLng],
            [bLat + rLen * Math.cos(rudRad), bLng + rLen * Math.sin(rudRad)]
        ]);
    }

    // Wind overlay on map
    if (windOverlayVisible && windLine) {
        const wdRad = d.wd * Math.PI / 180;
        const wLen = 0.002;
        windLine.setLatLngs([
            [d.lat + wLen * Math.cos(wdRad), d.lng + wLen * Math.sin(wdRad)],
            [d.lat - wLen * Math.cos(wdRad), d.lng - wLen * Math.sin(wdRad)]
        ]);
    }

    // Animated trail
    if (animTrail) {
        const trailPts = data.slice(0, i + 1).map(p => [p.lat, p.lng]);
        animTrail.setLatLngs(trailPts);
    }

    // Slider
    slider.value = i;

    // ── Info panel ──
    document.getElementById('v-time').textContent = (d.t / 1000).toFixed(1) + 's';
    document.getElementById('v-pos').textContent = d.lat.toFixed(5) + ', ' + d.lng.toFixed(5);
    document.getElementById('v-heading').textContent = d.hdg.toFixed(1) + '°';
    document.getElementById('v-speed').textContent = d.spd.toFixed(2) + ' m/s';
    document.getElementById('time-display').textContent = (d.t / 1000).toFixed(1) + 's';

    // Mode badge
    const modeEl = document.getElementById('v-mode');
    const modeClass = 'mode-' + (d.m === 'observation' ? 'obs' : d.m);
    const modeLabel = d.m.toUpperCase();
    modeEl.innerHTML = `<span class="mode-badge ${modeClass}">${modeLabel}</span>`;

    // Nearest waypoint + line to WPT
    const wpts = WAYPOINTS();
    if (wpts.length > 0) {
        let minDist = Infinity, nearId = 0;
        wpts.forEach(w => {
            const dd = Math.sqrt(Math.pow(w.lat - d.lat, 2) + Math.pow(w.lng - d.lng, 2)) * 111000;
            if (dd < minDist) { minDist = dd; nearId = w.id; }
        });
        document.getElementById('v-wpt').textContent = 'WPT ' + nearId;
        document.getElementById('v-dist').textContent = minDist < 1000 ? minDist.toFixed(0) + 'm' : (minDist/1000).toFixed(2) + 'km';
        const w = wpts[nearId];
        if (w && wptLine) wptLine.setLatLngs([[d.lat, d.lng], [w.lat, w.lng]]);
    } else {
        document.getElementById('v-wpt').textContent = '--';
        document.getElementById('v-dist').textContent = '--';
        if (wptLine) wptLine.setLatLngs([]);
    }

    // ── Wind arrow ──
    document.getElementById('wind-arrow').style.transform = `rotate(${d.wd + 180}deg)`;
    document.getElementById('wind-speed-val').textContent = d.ws.toFixed(1) + ' m/s';
    document.getElementById('wind-dir-val').textContent = d.wd.toFixed(0) + '°';

    // ── Gauges ──
    const rudPct = (d.rud / 20) * 50;
    const rudBar = document.getElementById('rudder-bar');
    if (d.rud >= 0) {
        rudBar.style.left = '50%';
        rudBar.style.width = Math.abs(rudPct) + '%';
    } else {
        rudBar.style.left = (50 + rudPct) + '%';
        rudBar.style.width = Math.abs(rudPct) + '%';
    }
    rudBar.style.background = MODE_COLORS[d.m] || '#ff5555';
    document.getElementById('rudder-val').textContent = d.rud.toFixed(1) + '°';

    const sailPct = (d.sail / 10) * 50;
    const sailBar = document.getElementById('sail-bar');
    if (d.sail >= 0) {
        sailBar.style.left = '50%';
        sailBar.style.width = Math.abs(sailPct) + '%';
    } else {
        sailBar.style.left = (50 + sailPct) + '%';
        sailBar.style.width = Math.abs(sailPct) + '%';
    }
    sailBar.style.background = '#50fa7b';
    document.getElementById('sail-val').textContent = d.sail.toFixed(1) + '°';

    const spdPct = Math.min(100, (d.spd / 3) * 100);
    document.getElementById('speed-bar').style.width = spdPct + '%';
    document.getElementById('speed-val').textContent = d.spd.toFixed(2) + ' m/s';
}

// ═══════════════════════════════════════════
//  PLAYBACK CONTROLS
// ═══════════════════════════════════════════
function togglePlay() {
    playing = !playing;
    document.getElementById('btn-play').textContent = playing ? '⏸' : '▶';
    document.getElementById('btn-play').classList.toggle('active', playing);
    if (playing) startAnim();
    else stopAnim();
}

function startAnim() {
    stopAnim();
    const interval = Math.max(10, 50 / playSpeed);
    animTimer = setInterval(() => {
        if (currentFrame >= DATA().length - 1) {
            togglePlay();
            return;
        }
        updateFrame(currentFrame + 1);
    }, interval);
}

function stopAnim() {
    if (animTimer) { clearInterval(animTimer); animTimer = null; }
}

function setSpeed(s) {
    playSpeed = s;
    [1,2,5,10].forEach(v => {
        const el = document.getElementById('sp' + v);
        if (el) el.classList.toggle('active', v === s);
    });
    if (playing) startAnim();
}

function onSlider(val) { updateFrame(parseInt(val)); }

function toggleTrail() {
    trailVisible = !trailVisible;
    trailSegments.forEach(s => {
        if (trailVisible) s.addTo(map);
        else map.removeLayer(s);
    });
}

function toggleWindOverlay() {
    windOverlayVisible = !windOverlayVisible;
    const btn = document.getElementById('btn-wind');
    if (btn) btn.classList.toggle('active', windOverlayVisible);
    if (windLine) {
        if (windOverlayVisible) windLine.addTo(map);
        else map.removeLayer(windLine);
    }
}

function resetView() {
    map.fitBounds(scenarioBounds());
    stopAnim();
    playing = false;
    document.getElementById('btn-play').textContent = '▶';
    document.getElementById('btn-play').classList.remove('active');
    updateFrame(0);
}

// ── Keyboard shortcuts ──
document.addEventListener('keydown', e => {
    if (e.code === 'Space') { e.preventDefault(); togglePlay(); }
    if (e.code === 'ArrowRight') updateFrame(Math.min(DATA().length - 1, currentFrame + 1));
    if (e.code === 'ArrowLeft') updateFrame(Math.max(0, currentFrame - 1));
    if (e.key === '1') setSpeed(1);
    if (e.key === '2') setSpeed(2);
    if (e.key === '5') setSpeed(5);
    if (e.key === '0') setSpeed(10);
    if (e.key === 'w') toggleWindOverlay();
    // Scenario switching with number keys (Shift+1..5)
    if (e.shiftKey && e.key === '!') switchScenario(0);
    if (e.shiftKey && e.key === '@') switchScenario(1);
    if (e.shiftKey && e.key === '#') switchScenario(2);
    if (e.shiftKey && e.key === '$') switchScenario(3);
    if (e.shiftKey && e.key === '%') switchScenario(4);
});

// ── Init first scenario ──
buildScenarioLayers();
map.fitBounds(scenarioBounds());
updateFrame(0);
setSpeed(10);
)JS";
}

// ─────────────────────────────────────────────
// Footer
// ─────────────────────────────────────────────
void HTMLExporter::writeFooter(std::ofstream& f) {
    f << "</body>\n</html>\n";
}
