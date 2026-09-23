#!/bin/bash
# Démarre la pile complète AutoBoat IHM, À PARTIR DE ZÉRO :
# nettoyage total (stop_ihm.sh) puis MongoDB, serial_link.py, webserver.py, navigateur.
#
# Usage :
#   ./start_ihm.sh                 — port transceiver auto-détecté (numéro de série)
#   ./start_ihm.sh /dev/ttyUSB0    — force le port série
#
# Pas de "set -e" : on gère les erreurs explicitement (plus robuste ici).

IHM_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_PYTHON="$IHM_DIR/.venv/bin/python"
COMPOSE_FILE="$IHM_DIR/docker-compose.yml"
SERIAL_LOG="/tmp/autoboat_serial.log"
SERVER_LOG="/tmp/autoboat_webserver.log"

GREEN='\033[0;32m'; YELLOW='\033[1;33m'; RED='\033[0;31m'; NC='\033[0m'
ok()   { echo -e "${GREEN}[OK]${NC}  $*"; }
info() { echo -e "${YELLOW}[..]${NC}  $*"; }
err()  { echo -e "${RED}[ERR]${NC} $*"; }

echo ""
echo "════════════════════════════════════════"
echo "   AutoBoat IHM — démarrage"
echo "════════════════════════════════════════"

# ── Port série ────────────────────────────────────────────────────────────────
if [[ $# -ge 1 ]]; then
    export SERIAL_PORT_OVERRIDE="$1"
    export SIMULATION="false"
    ok "Port série forcé (argument CLI) : $SERIAL_PORT_OVERRIDE"
else
    info "Port série : auto-détection du transceiver par numéro de série"
    info "Pour forcer un port : $0 /dev/ttyUSBx"
fi

# ── Vérification du venv ──────────────────────────────────────────────────────
if [[ ! -x "$VENV_PYTHON" ]]; then
    err "Environnement virtuel introuvable : $VENV_PYTHON"
    err "Créer avec : cd $IHM_DIR && python3 -m venv .venv && .venv/bin/pip install -r requirements.txt"
    exit 1
fi

# ── 0. Nettoyage complet (réutilise stop_ihm.sh — source unique de vérité) ────
info "Nettoyage des instances précédentes..."
bash "$IHM_DIR/stop_ihm.sh" >/dev/null 2>&1
sleep 1

# Logs frais (évite de lire d'anciennes lignes).
: > "$SERIAL_LOG"
: > "$SERVER_LOG"

cd "$IHM_DIR" || { err "cd $IHM_DIR a échoué"; exit 1; }

# ── 1. MongoDB ────────────────────────────────────────────────────────────────
info "Démarrage MongoDB (Docker)..."
docker compose -f "$COMPOSE_FILE" up -d >/dev/null 2>&1
for i in $(seq 1 20); do
    if "$VENV_PYTHON" -c "import socket; socket.create_connection(('localhost', 27017), 1)" 2>/dev/null; then
        ok "MongoDB prêt sur :27017"
        break
    fi
    sleep 1
    if [[ $i -eq 20 ]]; then
        err "MongoDB n'a pas démarré. Vérifier : docker compose -f $COMPOSE_FILE logs"
        exit 1
    fi
done

# ── 2. serial_link.py (-u : logs non bufferisés, lisibles en direct) ──────────
info "Démarrage serial_link.py..."
nohup "$VENV_PYTHON" -u app/serial_link.py >> "$SERIAL_LOG" 2>&1 &
SERIAL_PID=$!
sleep 1.5
if kill -0 "$SERIAL_PID" 2>/dev/null; then
    ok "serial_link.py  PID $SERIAL_PID  →  $SERIAL_LOG"
else
    err "serial_link.py a planté au démarrage. Dernières lignes :"
    tail -10 "$SERIAL_LOG"
    exit 1
fi

# ── 3. webserver.py (-u) ──────────────────────────────────────────────────────
info "Démarrage webserver.py..."
nohup "$VENV_PYTHON" -u app/webserver.py >> "$SERVER_LOG" 2>&1 &
SERVER_PID=$!
for i in $(seq 1 15); do
    if "$VENV_PYTHON" -c "import urllib.request; urllib.request.urlopen('http://localhost:5000', timeout=1)" 2>/dev/null; then
        ok "Serveur web  PID $SERVER_PID  →  http://localhost:5000"
        break
    fi
    sleep 1
    if [[ $i -eq 15 ]]; then
        err "Le serveur web n'a pas répondu. Dernières lignes :"
        tail -10 "$SERVER_LOG"
        exit 1
    fi
done

# ── 4. Navigateur ────────────────────────────────────────────────────────────
info "Ouverture du navigateur..."
xdg-open http://localhost:5000 >/dev/null 2>&1 \
    || open http://localhost:5000 >/dev/null 2>&1 \
    || err "Ouvrir manuellement : http://localhost:5000"

echo ""
echo "════════════════════════════════════════"
ok "IHM opérationnelle"
echo "   Navigateur  →  http://localhost:5000"
echo "   Logs (en direct) :"
echo "     tail -f $SERIAL_LOG"
echo "     tail -f $SERVER_LOG"
echo "   Arrêter : ./stop_ihm.sh"
echo "════════════════════════════════════════"
