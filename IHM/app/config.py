import os
import glob
from dotenv import load_dotenv

load_dotenv()

SIMULATION = os.getenv("SIMULATION", "true").lower() == "true"
DEFAULT_BAUD = 115200

# Numéro de série CP2104 du transceiver (gravé en usine, unique par puce).
# Permet de retrouver le port quel que soit /dev/ttyUSBn (ordre de branchement).
TRANSCEIVER_SERIAL = os.getenv("TRANSCEIVER_SERIAL", "01C00B54")


def find_transceiver_port():
    """Cherche le port du transceiver par numéro de série dans
    /dev/serial/by-id/. Renvoie le chemin réel (ex: /dev/ttyUSB2) ou None.
    Indépendant de udev : /dev/serial/by-id existe par défaut sous Linux."""
    serial = (TRANSCEIVER_SERIAL or "").strip()
    by_id_dir = "/dev/serial/by-id"

    if not serial or not os.path.isdir(by_id_dir):
        return None

    for link in glob.glob(os.path.join(by_id_dir, "*")):
        if serial in os.path.basename(link):
            try:
                return os.path.realpath(link)
            except OSError:
                return link
    return None


def resolve_serial_port():
    """Résout le port série à utiliser selon le mode et la priorité :
      1. SERIAL_PORT_OVERRIDE  (forcé via start_ihm.sh <port>)
      2. auto-détection par numéro de série (mode réel)
      3. SERIAL_PORT_REAL / SERIAL_PORT_SIM du .env (repli)
    Renvoie (port, source)."""
    if SIMULATION:
        return os.getenv("SERIAL_PORT_SIM", "/tmp/ttyV1"), "simulation"

    override = os.getenv("SERIAL_PORT_OVERRIDE")
    if override:
        return override, "argument CLI"

    detected = find_transceiver_port()
    if detected:
        return detected, f"auto-détection (série {TRANSCEIVER_SERIAL})"

    return os.getenv("SERIAL_PORT_REAL", "/dev/ttyUSB0"), "repli .env"


SERIAL_PORT, _PORT_SOURCE = resolve_serial_port()
print(f"{'🛠️ SIMULATION' if SIMULATION else '🚢 RÉEL'} — Port: {SERIAL_PORT}  [{_PORT_SOURCE}]")

MONGO_URI = os.getenv("MONGO_URI")
DB_NAME = os.getenv("DB_NAME")
BAUD_RATE = int(os.getenv("BAUD_RATE", DEFAULT_BAUD))
SERVER_PORT = int(os.getenv("SERVER_PORT", 5000))
