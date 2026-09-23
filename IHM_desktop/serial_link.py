"""
serial_link.py — Enlace serie directo con el transceiver (sin MongoDB ni server).

Sustituye, en un solo proceso, a IHM/app/serial_link.py + config.py + el bus
MongoDB. Un hilo lee el puerto serie, extrae el JSON del heartbeat y lo publica
en una cola que la GUI consume. Las órdenes salientes se envían en ráfaga 3×
(anti-colisión LoRa half-duplex), idéntico al original.
"""

import glob
import json
import os
import queue
import threading
import time

import serial
import serial.tools.list_ports

DEFAULT_BAUD = 115200
# Nº de serie CP2104 del transceiver (grabado en fábrica, único por chip).
DEFAULT_TRANSCEIVER_SERIAL = "01C00B54"

# El enlace LoRa es half-duplex: un envío único suele colisionar con el
# heartbeat del barco (TX bloqueante ~450 ms cada segundo) y se pierde.
# Repetimos cada orden unas veces, espaciadas. Todas son idempotentes.
COMMAND_BURST_COUNT = 3
COMMAND_BURST_GAP_S = 0.35


def find_transceiver_port(serial_number=DEFAULT_TRANSCEIVER_SERIAL):
    """Busca el puerto del transceiver por nº de serie en /dev/serial/by-id/.
    Devuelve la ruta real (ej. /dev/ttyUSB2) o None."""
    sn = (serial_number or "").strip()
    by_id = "/dev/serial/by-id"
    if sn and os.path.isdir(by_id):
        for link in glob.glob(os.path.join(by_id, "*")):
            if sn in os.path.basename(link):
                try:
                    return os.path.realpath(link)
                except OSError:
                    return link
    # Repli: buscar por nº de serie en la lista de puertos pyserial.
    if sn:
        for p in serial.tools.list_ports.comports():
            if p.serial_number and sn in p.serial_number:
                return p.device
    return None


def list_serial_ports():
    """Lista de (device, descripción) de los puertos serie disponibles."""
    out = []
    for p in serial.tools.list_ports.comports():
        desc = p.description or ""
        if p.serial_number:
            desc += f" [{p.serial_number}]"
        out.append((p.device, desc))
    return out


def extract_json_payload(line):
    start = line.find("{")
    end = line.rfind("}")
    if start == -1 or end == -1 or end < start:
        return None
    try:
        return json.loads(line[start:end + 1])
    except json.JSONDecodeError:
        return None


def reset_esp32(port, baud=DEFAULT_BAUD):
    """Hard-reset del transceiver vía DTR/RTS (como esptool):
    RTS pilota EN (reset), DTR pilota IO0. Mantiene EN bajo 100 ms y suelta →
    arranca el programa (no bootloader, IO0 alto)."""
    ser = serial.Serial(port, baud)
    try:
        ser.setDTR(False)   # IO0 = ALTO (arranque normal)
        ser.setRTS(True)    # EN  = BAJO (reset activo)
        time.sleep(0.1)
        ser.setRTS(False)   # EN  = ALTO (reset soltado)
        time.sleep(0.1)
    finally:
        ser.close()


class SerialLink:
    """Hilo de enlace serie. Publica eventos en `events` (queue.Queue):
        ("boat",   dict)   — mensaje del barco (campo 'message' ya extraído)
        ("status", dict)   — {"connected": bool, "port": str, "info": str}
        ("error",  str)    — mensaje de error
    Las órdenes salientes se encolan con send_command().
    """

    def __init__(self, events, serial_number=DEFAULT_TRANSCEIVER_SERIAL,
                 baud=DEFAULT_BAUD, forced_port=None):
        self.events = events
        self.serial_number = serial_number
        self.baud = baud
        self.forced_port = forced_port
        self._outgoing = queue.Queue()
        self._stop = threading.Event()
        self._thread = None
        self._ser = None
        self.current_port = None

    # ---- API pública ----
    def start(self):
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2.0)

    def set_port(self, port_or_none):
        """Fuerza un puerto (o None para volver a auto-detección).
        Provoca una reconexión limpia."""
        self.forced_port = port_or_none
        self._close_serial()

    def send_command(self, message):
        """Encola una orden de alto nivel (dict 'message'). Se envuelve en el
        sobre del protocolo y se envía en ráfaga."""
        envelope = {"origin": "server", "type": "command", "message": message}
        self._outgoing.put(json.dumps(envelope, separators=(",", ":")))

    # ---- Interno ----
    def _resolve_port(self):
        if self.forced_port:
            return self.forced_port, "puerto forzado"
        detected = find_transceiver_port(self.serial_number)
        if detected:
            return detected, f"auto-detección (serie {self.serial_number})"
        return None, "no encontrado"

    def _close_serial(self):
        if self._ser:
            try:
                self._ser.close()
            except Exception:
                pass
        self._ser = None
        self.current_port = None

    def _connect(self):
        port, source = self._resolve_port()
        if port is None:
            self.events.put(("status", {"connected": False, "port": None,
                                        "info": "transceiver no detectado"}))
            return False
        try:
            # dsrdtr/rtscts=False: no resetea el ESP32 al (re)conectar.
            self._ser = serial.Serial(port, self.baud, timeout=1,
                                      dsrdtr=False, rtscts=False)
            self.current_port = port
            self.events.put(("status", {"connected": True, "port": port,
                                        "info": source}))
            return True
        except serial.SerialException as e:
            self.events.put(("status", {"connected": False, "port": port,
                                        "info": f"error: {e}"}))
            return False

    def _read_once(self):
        line = self._ser.readline().decode("utf-8", errors="replace").strip()
        if not line:
            return
        payload = extract_json_payload(line)
        if payload is None:
            return
        # Solo nos interesan los mensajes del barco para la GUI.
        if payload.get("origin", "boat") == "server":
            return
        message = payload.get("message", {})
        if isinstance(message, dict):
            self.events.put(("boat", {"message": message,
                                      "timestamp": time.time()}))

    def _send_pending(self):
        try:
            data = self._outgoing.get_nowait()
        except queue.Empty:
            return
        payload = (data + "\n").encode("utf-8")
        for _ in range(COMMAND_BURST_COUNT):
            if self._stop.is_set():
                break
            self._ser.write(payload)
            time.sleep(COMMAND_BURST_GAP_S)

    def _run(self):
        while not self._stop.is_set():
            try:
                if self._ser is None:
                    if not self._connect():
                        time.sleep(1.0)
                        continue
                self._read_once()
                self._send_pending()
            except Exception as e:
                self.events.put(("error", str(e)))
                self._close_serial()
                self.events.put(("status", {"connected": False, "port": None,
                                            "info": "reconexión…"}))
                time.sleep(1.0)
        self._close_serial()
