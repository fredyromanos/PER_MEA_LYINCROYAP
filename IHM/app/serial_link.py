import serial
import json
import time

from utils import db
import config

def extract_json_payload(line):
    start = line.find("{")
    end = line.rfind("}")

    if start == -1 or end == -1 or end < start:
        return None

    candidate = line[start:end + 1]

    try:
        return json.loads(candidate)
    except json.JSONDecodeError:
        return None


def connect():
    # Ré-résout le port à CHAUD à chaque tentative : si l'USB se ré-énumère
    # (ttyUSB0→ttyUSB2...), on retrouve le transceiver par son numéro de série
    # au lieu de rester collé à un nœud /dev mort.
    port, source = config.resolve_serial_port()
    try:
        # dsrdtr/rtscts=False : évite de réinitialiser l'ESP32 (toggle DTR/RTS)
        # à chaque (re)connexion — important quand l'USB se ré-énumère souvent.
        ser = serial.Serial(port, config.BAUD_RATE, timeout=1,
                            dsrdtr=False, rtscts=False)
        print(f"Listening on {port} [{source}] at {config.BAUD_RATE} baud...")
        return ser
    except serial.SerialException as e:
        print(f"Serial error ({port}): {e}")
        return None


def read(ser, collection):
    line = ser.readline().decode("utf-8", errors="replace").strip()
    if line:
        payload = extract_json_payload(line)

        if payload is None:
            print(f"Ignored non-JSON serial line: {line}")
            return

        origin = payload.get("origin", "boat")
        destination = "boat" if origin == "server" else "server"

        db.push(
            origin=origin,
            destination=destination,
            data=json.dumps(payload, separators=(",", ":")),
            status="received",
            collection=collection
        )


# Le lien LoRa est half-duplex : un envoi unique entre souvent en collision
# avec le heartbeat du bateau (TX bloquant ~450 ms toutes les secondes) et se
# perd. On répète donc chaque commande quelques fois, espacées, pour qu'au
# moins une tombe dans une fenêtre d'écoute. Toutes les commandes du protocole
# sont idempotentes, donc la répétition est sans danger.
COMMAND_BURST_COUNT = 3
COMMAND_BURST_GAP_S = 0.35


def send(ser, collection):
    message = db.get_pending_message(collection)
    if message:
        data = json.dumps(message["data"], separators=(",", ":")) + "\n"
        payload = data.encode("utf-8")
        for i in range(COMMAND_BURST_COUNT):
            ser.write(payload)
            print(f"🚀 Envoi vers la carte ({i + 1}/{COMMAND_BURST_COUNT}) : {data.strip()}")
            time.sleep(COMMAND_BURST_GAP_S)



def main():
    collection = db.sync_client()
    ser = None
    while True:
        try:
            if ser is None:
                ser = connect()
                if ser is None:
                    time.sleep(1.0)   # port absent → on réessaie (re-détection)
                    continue
            read(ser, collection)
            send(ser, collection)

        except KeyboardInterrupt:
            print("Exiting...")
            if ser:
                ser.close()
            break

        except Exception as e:
            # Déconnexion / ré-énumération USB / caractère invalide :
            # on ferme et on reconnecte (avec re-détection du port).
            print(f"Error: {e} — reconnexion...")
            try:
                if ser:
                    ser.close()
            except Exception:
                pass
            ser = None
            time.sleep(1.0)


if __name__ == "__main__":
    main()
