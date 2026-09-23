#!/bin/bash
# Arrête TOUS les processus de l'IHM AutoBoat de façon fiable :
# serial_link.py, webserver.py + workers uvicorn (port 5000), MongoDB (Docker).
# Pas de "set -e" : pkill renvoie 1 quand rien ne correspond, c'est normal.

IHM_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
COMPOSE_FILE="$IHM_DIR/docker-compose.yml"
SERVER_PORT="${SERVER_PORT:-5000}"

GREEN='\033[0;32m'; YELLOW='\033[1;33m'; RED='\033[0;31m'; NC='\033[0m'
ok()   { echo -e "${GREEN}[OK]${NC}  $*"; }
info() { echo -e "${YELLOW}[..]${NC}  $*"; }
err()  { echo -e "${RED}[ERR]${NC} $*"; }

# Tue un motif de processus : SIGTERM, attente, puis SIGKILL si récalcitrant.
kill_pattern() {
    local pat="$1"
    pgrep -f "$pat" >/dev/null 2>&1 || return 0
    pkill -TERM -f "$pat" 2>/dev/null
    for _ in 1 2 3 4 5 6; do
        pgrep -f "$pat" >/dev/null 2>&1 || return 0
        sleep 0.3
    done
    pkill -KILL -f "$pat" 2>/dev/null
    sleep 0.3
}

echo ""
echo "════════════════════════════════════════"
echo "   AutoBoat IHM — arrêt complet"
echo "════════════════════════════════════════"

# ── 1. serial_link.py ─────────────────────────────────────────────────────────
info "Arrêt serial_link.py..."
kill_pattern "app/serial_link.py"
if pgrep -f "app/serial_link.py" >/dev/null 2>&1; then
    err "serial_link.py toujours présent (PID: $(pgrep -f 'app/serial_link.py' | tr '\n' ' '))"
else
    ok "serial_link.py arrêté"
fi

# ── 1b. Simulateurs + ports virtuels (lancés par le bouton Start) ─────────────
# Ils sont lancés en session séparée : ils survivraient à l'arrêt du webserver.
info "Arrêt simulateurs (simu_boat / simu_gps) et socat..."
kill_pattern "AutoBoat_Simulation/simu_gps"
kill_pattern "AutoBoat_Simulation/simu_boat"
kill_pattern "socat -d -d PTY,link=/tmp/ttySimu"
ok "simulateurs arrêtés"

# ── 2. webserver.py + workers uvicorn ─────────────────────────────────────────
# uvicorn (reload=True) lance un worker enfant qui DÉTIENT le port : on le tue
# aussi en libérant le port, sinon "Address already in use" au redémarrage.
info "Arrêt webserver.py / uvicorn (port $SERVER_PORT)..."
kill_pattern "app/webserver.py"
fuser -k "${SERVER_PORT}/tcp" 2>/dev/null
kill_pattern "webserver:app"
sleep 0.3
if pgrep -f "app/webserver.py" >/dev/null 2>&1 || fuser "${SERVER_PORT}/tcp" >/dev/null 2>&1; then
    err "webserver toujours présent / port $SERVER_PORT occupé"
else
    ok "webserver arrêté"
fi

# ── 3. MongoDB (Docker) ───────────────────────────────────────────────────────
# "down" arrête ET supprime le conteneur (le volume de données persiste),
# ce qui évite qu'il reste en vie à cause de restart:unless-stopped.
info "Arrêt MongoDB (Docker)..."
docker compose -f "$COMPOSE_FILE" down --remove-orphans >/dev/null 2>&1
docker rm -f autoboat-mongo >/dev/null 2>&1
if docker ps --filter "name=autoboat-mongo" --format '{{.Names}}' | grep -q autoboat-mongo; then
    err "MongoDB (autoboat-mongo) toujours actif"
else
    ok "MongoDB arrêté"
fi

echo "════════════════════════════════════════"
ok "IHM arrêtée"
echo "   Relancer avec : ./start_ihm.sh [/dev/ttyUSBx]"
echo "════════════════════════════════════════"
echo ""
