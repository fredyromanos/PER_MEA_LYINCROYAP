#!/usr/bin/env python3
"""
ihm_desktop.py — IHM de escritorio del AutoBoat (sin navegador ni servidor).

Réplica funcional de la IHM web (IHM/) en una sola app de escritorio con
CustomTkinter (tema oscuro moderno) + tkintermapview: mapa OSM, telemetría en
vivo, panel de diagnóstico, brújula de viento, medición de viento, waypoints
por clic, import GeoJSON, predicción de trayectoria y todas las órdenes al
barco — hablando directo con el transceiver por USB. Sin MongoDB, FastAPI,
Docker ni navegador.

Lanzar:  ./run.sh        (o  .venv/bin/python ihm_desktop.py)
"""

import json
import math
import queue
import threading
import time
import tkinter as tk
from tkinter import filedialog, messagebox, simpledialog

import customtkinter as ctk
import tkintermapview

import nav
from serial_link import SerialLink, list_serial_ports, reset_esp32

# --- Constantes de presentación ---
BREST = (48.360687, -4.565710)
DEFAULT_ZOOM = 17
DEFAULT_WAYPOINT_RADIUS_M = 5.0
MAX_LORA_PACKET_BYTES = 255
POLL_MS = 100
HEALTH_MS = 1000

# Paleta
COL_OK = "#2ecc71"
COL_WARN = "#f39c12"
COL_BAD = "#e74c3c"
COL_UNKNOWN = "#5a5a5a"
COL_RADIO = "#5dade2"
COL_ACCENT = "#3b8ed0"
COL_CARD = "#242424"
COL_OVERLAY = "#1c1c1c"
COL_MUTED = "#8a8a8a"

ctk.set_appearance_mode("dark")
ctk.set_default_color_theme("blue")


class AutoBoatIHM:
    def __init__(self, root):
        self.root = root
        self.root.title("AutoBoat — IHM Desktop")
        self.root.geometry("1380x860")
        self.root.minsize(1100, 700)

        # Estado de telemetría / enlace
        self.events = queue.Queue()
        self.link = SerialLink(self.events)
        self.last_message = None
        self.last_fresh_ms = None
        self.last_seen_ts = None
        self.rx_count = 0
        self.connected = False
        self.conn_port = None
        self._last_status_info = ""
        self._port_map = {}

        # Velocidad
        self.prev_lat = self.prev_lon = self.prev_time = None

        # Waypoints / mapa
        self.waypoints = []
        self.waypoint_markers = []
        self.route_line = None
        self.boat_marker = None
        self.boat_track = []
        self.boat_path = None
        self.show_trajectory = False
        self.trajectory_line = None
        self.last_wp_index = 0

        self._build_ui()
        self.link.start()
        self.root.after(POLL_MS, self._poll_events)
        self.root.after(HEALTH_MS, self._tick_health)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    # ====================================================================
    # Construcción de la interfaz
    # ====================================================================
    def _build_ui(self):
        self._build_header()
        self._build_command_bar()

        # Panel derecho: cabecera fija (État système) + resto scrollable
        right_col = ctk.CTkFrame(self.root, width=360, fg_color="transparent")
        right_col.pack(side="right", fill="y", padx=(0, 8), pady=8)
        right_col.pack_propagate(False)
        self._build_side_panel(right_col)

        # Mapa (centro, ocupa el resto)
        center = ctk.CTkFrame(self.root, corner_radius=12)
        center.pack(side="left", fill="both", expand=True, padx=8, pady=8)

        self.map = tkintermapview.TkinterMapView(center, corner_radius=12)
        self.map.pack(fill="both", expand=True, padx=4, pady=4)
        self.map.set_position(*BREST)
        self.map.set_zoom(DEFAULT_ZOOM)
        self.map.add_left_click_map_command(self._on_map_click)

        self._build_overlays(center)

    def _build_header(self):
        header = ctk.CTkFrame(self.root, height=58, corner_radius=12)
        header.pack(side="top", fill="x", padx=8, pady=(8, 0))

        ctk.CTkLabel(header, text="⛵  AutoBoat",
                     font=ctk.CTkFont(size=20, weight="bold")).pack(
                         side="left", padx=16)
        ctk.CTkLabel(header, text="IHM Desktop", text_color=COL_MUTED,
                     font=ctk.CTkFont(size=12)).pack(side="left")

        # Estado de conexión (pill)
        self.conn_label = ctk.CTkLabel(header, text="●  déconnecté",
                                       fg_color="#3a2a1a", corner_radius=14,
                                       text_color="#e67e22", width=320, height=28,
                                       font=ctk.CTkFont(size=12, weight="bold"))
        self.conn_label.pack(side="right", padx=16)

        # Selector de puerto
        ctk.CTkButton(header, text="Appliquer", width=80, command=self._apply_port).pack(
            side="right", padx=(0, 8), pady=12)
        ctk.CTkButton(header, text="⟳", width=34, command=self._refresh_ports).pack(
            side="right", pady=12)
        self.port_var = ctk.StringVar(value="auto")
        self.port_menu = ctk.CTkOptionMenu(header, variable=self.port_var, width=300,
                                           values=["auto"])
        self.port_menu.pack(side="right", padx=8, pady=12)
        ctk.CTkLabel(header, text="Port", text_color=COL_MUTED).pack(
            side="right", padx=(0, 2))
        self._refresh_ports()

    def _build_overlays(self, parent):
        # Brújula de viento (arriba-izquierda)
        comp = ctk.CTkFrame(parent, corner_radius=12, fg_color=COL_OVERLAY)
        comp.place(x=16, y=16)
        self.compass = tk.Canvas(comp, width=108, height=128, highlightthickness=0,
                                 bg=COL_OVERLAY)
        self.compass.pack(padx=6, pady=6)
        self._draw_compass(None)

        # Medición de viento (bajo la brújula) — oculta por defecto
        self.windobs_frame = ctk.CTkFrame(parent, corner_radius=12, fg_color=COL_OVERLAY,
                                          width=170)
        ctk.CTkLabel(self.windobs_frame, text="📏  Mesure du vent",
                     font=ctk.CTkFont(size=12, weight="bold")).pack(
                         anchor="w", padx=12, pady=(10, 4))
        self.windobs_bar = ctk.CTkProgressBar(self.windobs_frame, width=150,
                                              progress_color=COL_OK)
        self.windobs_bar.set(0)
        self.windobs_bar.pack(padx=12)
        self.windobs_value = ctk.CTkLabel(self.windobs_frame, text="0%",
                                          font=ctk.CTkFont(size=12, weight="bold"))
        self.windobs_value.pack(anchor="e", padx=12, pady=(2, 0))
        ctk.CTkButton(self.windobs_frame, text="✖  Annuler", fg_color=COL_BAD,
                      hover_color="#c0392b", height=28,
                      command=self._cancel_windobs).pack(fill="x", padx=12, pady=10)
        self._windobs_placed = False

    # ---- Tarjetas del panel ----
    def _card(self, parent, title):
        frame = ctk.CTkFrame(parent, corner_radius=12, fg_color=COL_CARD)
        frame.pack(fill="x", pady=6)
        ctk.CTkLabel(frame, text=title, anchor="w",
                     font=ctk.CTkFont(size=13, weight="bold")).pack(
                         fill="x", padx=14, pady=(10, 4))
        body = ctk.CTkFrame(frame, fg_color="transparent")
        body.pack(fill="x", padx=14, pady=(0, 10))
        return body

    def _info_row(self, parent, label, key, default="?"):
        row = ctk.CTkFrame(parent, fg_color="transparent")
        row.pack(fill="x", pady=1)
        ctk.CTkLabel(row, text=label, text_color=COL_MUTED, anchor="w",
                     font=ctk.CTkFont(size=12)).pack(side="left")
        val = ctk.CTkLabel(row, text=default, anchor="e",
                           font=ctk.CTkFont(size=13, weight="bold"))
        val.pack(side="right")
        self.lbl[key] = val

    def _health_row(self, parent, key, label):
        row = ctk.CTkFrame(parent, fg_color="transparent")
        row.pack(fill="x", pady=3)
        dot = ctk.CTkFrame(row, width=14, height=14, corner_radius=7,
                           fg_color=COL_UNKNOWN)
        dot.pack(side="left", padx=(0, 10))
        dot.pack_propagate(False)
        ctk.CTkLabel(row, text=label, anchor="w",
                     font=ctk.CTkFont(size=12, weight="bold"), width=92).pack(side="left")
        val = ctk.CTkLabel(row, text="—", text_color=COL_MUTED, anchor="e",
                           font=ctk.CTkFont(size=12))
        val.pack(side="right")
        self.health_rows[key] = (dot, val)

    def _build_side_panel(self, parent):
        # --- FIJO arriba: estado del sistema (semáforos + frescura de la liaison) ---
        # Se mantiene siempre visible para saber si los datos son actuales o viejos.
        self.health_rows = {}
        health = self._card(parent, "État système")
        for key, label in (("link", "Liaison"), ("gps", "GPS"), ("bat", "Batterie"),
                           ("rc", "Radio (RC)"), ("mode", "Contrôle")):
            self._health_row(health, key, label)
        self.health_warning = ctk.CTkLabel(
            health, text="⚠  Liaison perdue — valeurs non fiables.",
            fg_color="#3a2f1a", text_color="#f0c674", corner_radius=8,
            font=ctk.CTkFont(size=11), wraplength=290)

        # --- Resto: scrollable ---
        self.scroll = ctk.CTkScrollableFrame(parent, fg_color="transparent")
        self.scroll.pack(fill="both", expand=True, pady=(2, 0))
        scroll = self.scroll

        # Batería (barra)
        batcard = self._card(scroll, "Batterie")
        self.battery_bar = ctk.CTkProgressBar(batcard, progress_color=COL_OK)
        self.battery_bar.set(0)
        self.battery_bar.pack(fill="x", pady=(2, 4))
        self.battery_text = ctk.CTkLabel(batcard, text="—",
                                         font=ctk.CTkFont(size=13, weight="bold"))
        self.battery_text.pack(anchor="e")

        # Telemetría
        self.lbl = {}
        info = self._card(scroll, "Bateau")
        self._info_row(info, "Dernier msg", "last_time")
        self._info_row(info, "Mode", "mode", "not-connected")
        self._info_row(info, "Vitesse", "speed")
        self._info_row(info, "Cap", "heading")
        self._info_row(info, "Voile", "sail")
        self._info_row(info, "Safran", "rudder")
        self._info_row(info, "Vent", "wind")
        self._info_row(info, "Latitude", "lat")
        self._info_row(info, "Longitude", "lon")
        self._info_row(info, "WP total", "wt")
        self._info_row(info, "WP actuel", "wc")

        # Waypoints (sin scroll propio: la lista crece y scrollea el panel entero)
        wpcard = self._card(scroll, "Waypoints")
        self.wp_list_frame = ctk.CTkFrame(wpcard, fg_color="#1c1c1c", corner_radius=8)
        self.wp_list_frame.pack(fill="x")
        ctk.CTkLabel(self.wp_list_frame, text="Clic sur la carte pour ajouter",
                     text_color=COL_MUTED, font=ctk.CTkFont(size=11)).pack(pady=10)

        # Rueda del mouse: en Linux/X11 son Button-4/5 (CustomTkinter solo enlaza
        # <MouseWheel>, de ahí que no scrolleara). Enlazamos a mano el canvas y
        # todo el árbol del panel.
        canvas = self.scroll._parent_canvas
        canvas.bind("<Button-4>", lambda e: self._wheel(e, canvas))
        canvas.bind("<Button-5>", lambda e: self._wheel(e, canvas))
        canvas.bind("<MouseWheel>", lambda e: self._wheel(e, canvas))
        self._bind_wheel(self.scroll, canvas)

    def _wheel(self, event, canvas):
        if canvas.yview() == (0.0, 1.0):
            return   # nada que scrollear
        num = getattr(event, "num", None)
        if num == 4:
            canvas.yview_scroll(-1, "units")
        elif num == 5:
            canvas.yview_scroll(1, "units")
        elif getattr(event, "delta", 0):
            canvas.yview_scroll(-1 if event.delta > 0 else 1, "units")

    def _bind_wheel(self, widget, canvas):
        """Enlaza la rueda (Linux Button-4/5 + MouseWheel) en el widget y todos
        sus descendientes, para que el scroll funcione esté el cursor donde esté."""
        widget.bind("<Button-4>", lambda e: self._wheel(e, canvas))
        widget.bind("<Button-5>", lambda e: self._wheel(e, canvas))
        widget.bind("<MouseWheel>", lambda e: self._wheel(e, canvas))
        for child in widget.winfo_children():
            self._bind_wheel(child, canvas)

    def _build_command_bar(self):
        bar = ctk.CTkFrame(self.root, corner_radius=12)
        bar.pack(side="bottom", fill="x", padx=8, pady=8)
        inner = ctk.CTkFrame(bar, fg_color="transparent")
        inner.pack(padx=12, pady=10)

        def B(text, cmd, color=None, hover=None):
            b = ctk.CTkButton(inner, text=text, command=cmd, width=120, height=36,
                              corner_radius=10, font=ctk.CTkFont(size=13),
                              fg_color=color, hover_color=hover)
            b.pack(side="left", padx=5)
            return b

        B("🚢  Naviguer", self._cmd_navigate, COL_ACCENT, "#2f7bb5")
        B("⏹  Stop", self._cmd_stop, COL_BAD, "#c0392b")
        B("📍  Envoyer route", self._send_route, "#16a085", "#138d75")
        B("🗑  Effacer route", self._clear_route, "#555", "#666")
        B("📂  Importer", self._import_geojson, "#555", "#666")
        B("📏  Mesure vent", self._cmd_wind_observation, "#555", "#666")
        B("🌬️  Envoi vent", self._cmd_wind_command, "#555", "#666")
        self.traj_btn = B("🧭  Trajectoire", self._toggle_trajectory, "#555", "#666")
        B("🔌  Reset TRX", self._reset_transceiver, "#555", "#666")
        B("🔄  Redémarrer", self._cmd_restart, COL_BAD, "#c0392b")

    # ====================================================================
    # Bucle de eventos (cola del hilo serie → GUI)
    # ====================================================================
    def _poll_events(self):
        try:
            while True:
                kind, payload = self.events.get_nowait()
                if kind == "boat":
                    self._on_boat_message(payload)
                elif kind == "status":
                    self._on_status(payload)
        except queue.Empty:
            pass
        self.root.after(POLL_MS, self._poll_events)

    def _on_status(self, st):
        self.connected = bool(st.get("connected"))
        self.conn_port = st.get("port")
        self._last_status_info = st.get("info", "")
        self._update_conn_display()

    def _update_conn_display(self):
        if not self.connected:
            self.conn_label.configure(text=f"●  déconnecté ({self._last_status_info})",
                                      fg_color="#3a2a1a", text_color="#e67e22")
            return
        port = self.conn_port or ""
        mode = "SIM" if "/tmp/tty" in port else "RÉEL"
        if self.rx_count == 0:
            self.conn_label.configure(
                text=f"●  {mode} — en attente du bateau…",
                fg_color="#3a361a", text_color="#f1c40f")
        else:
            self.conn_label.configure(
                text=f"●  {mode} — {port}  ·  RX {self.rx_count}",
                fg_color="#19331f", text_color=COL_OK)

    def _on_boat_message(self, payload):
        message = payload.get("message", {})
        ts = payload.get("timestamp")

        self.rx_count += 1
        if ts and ts != self.last_seen_ts:
            self.last_seen_ts = ts
            self.last_fresh_ms = time.time() * 1000
        if message:
            self.last_message = message
        self._refresh_link_health()
        self._update_conn_display()

        if not message:
            return

        self._update_health(message)
        if "mode" in message:
            self.lbl["mode"].configure(text=str(message["mode"]))
        if "location" in message:
            self._update_location(message["location"], ts)
        if "servos" in message:
            s = message["servos"]
            self.lbl["sail"].configure(text=f"{s.get('sail', '?')}°")
            self.lbl["rudder"].configure(text=f"{s.get('rudder', '?')}°")
        if "heading" in message:
            self.lbl["heading"].configure(text=f"{message['heading']}°")
        if "wind" in message:
            self.lbl["wind"].configure(text=f"{message['wind']}°")
            self._draw_compass(message["wind"])
        if "wt" in message:
            self.lbl["wt"].configure(text=str(message["wt"]))
            self.lbl["wc"].configure(text=str(message.get("wc", "?")))
        elif "waypoints" in message:
            self.lbl["wt"].configure(text=str(message["waypoints"].get("total", "?")))
            self.lbl["wc"].configure(text=str(message["waypoints"].get("current", "?")))
        self._update_windobs(message)

        if "bat" in message:
            self._update_battery(message["bat"])

        if "location" in message:
            loc = message["location"]
            self.last_wp_index = (message.get("wc") if "wc" in message
                                  else message.get("waypoints", {}).get("current", 0))
            self._cur_telemetry = (loc[0], loc[1], message.get("heading"),
                                   message.get("wind"))
            if self.show_trajectory:
                self._render_trajectory()

    # ====================================================================
    # Salud / diagnóstico
    # ====================================================================
    def _set_health(self, key, color, text):
        dot, val = self.health_rows[key]
        dot.configure(fg_color=color)
        val.configure(text=text)

    def _tick_health(self):
        self._refresh_link_health()
        self.root.after(HEALTH_MS, self._tick_health)

    def _refresh_link_health(self):
        if self.last_fresh_ms is None:
            self._set_health("link", COL_UNKNOWN, "en attente…")
            self._apply_reliability(True)
            return
        age_ms = time.time() * 1000 - self.last_fresh_ms
        age_s = round(age_ms / 1000)
        if age_ms < 3000:
            rssi = None
            if self.last_message and self.last_message.get("rssi") is not None:
                rssi = float(self.last_message["rssi"])
            if rssi is None:
                self._set_health("link", COL_OK, "OK")
            elif rssi <= -110:
                self._set_health("link", COL_WARN, f"{rssi:.0f} dBm")
            else:
                self._set_health("link", COL_OK, f"{rssi:.0f} dBm")
            self._apply_reliability(True)
        elif age_ms < 10000:
            self._set_health("link", COL_WARN, f"{age_s} s")
            self._apply_reliability(True)
        else:
            self._set_health("link", COL_BAD, f"perdue ({age_s} s)")
            self._apply_reliability(False)

    def _apply_reliability(self, reliable):
        if reliable:
            self.health_warning.pack_forget()
            if self.last_message:
                self._update_health(self.last_message)
        else:
            self.health_warning.pack(fill="x", pady=(6, 2))
            for key in ("gps", "bat", "rc", "mode"):
                self.health_rows[key][0].configure(fg_color=COL_WARN)

    def _update_health(self, message):
        if "fix" in message:
            fix = int(message.get("fix", 0))
            sat = int(message.get("sat", 0)) if message.get("sat") is not None else 0
            try:
                hdop = float(message.get("hdop"))
            except (TypeError, ValueError):
                hdop = float("nan")
            if not fix:
                self._set_health("gps", COL_BAD, "pas de fix")
            else:
                good = (sat >= 5 and hdop <= 2.5)
                hdop_s = "?" if math.isnan(hdop) else f"{hdop:.1f}"
                self._set_health("gps", COL_OK if good else COL_WARN,
                                 f"{sat} sat · hdop {hdop_s}")
        if "bat" in message:
            try:
                v = float(message["bat"])
                color = COL_OK if v >= 7.0 else (COL_WARN if v >= 6.6 else COL_BAD)
                self._set_health("bat", color, f"{v:.2f} V")
            except (TypeError, ValueError):
                self._set_health("bat", COL_UNKNOWN, "—")
        if "rc" in message:
            rc_ok = int(message.get("rc", 0)) == 1
            self._set_health("rc", COL_OK if rc_ok else COL_BAD,
                             "OK" if rc_ok else "perdue")
        if "mode" in message:
            auto = (message["mode"] != "standby")
            self._set_health("mode", COL_OK if auto else COL_RADIO,
                             "auto" if auto else "radio")

    # ====================================================================
    # Telemetría
    # ====================================================================
    def _update_location(self, loc, ts):
        lat, lon = loc[0], loc[1]
        cur_t = ts if ts else time.time()
        if self.prev_lat is not None and self.prev_time is not None:
            dist = nav.distance_m(self.prev_lat, self.prev_lon, lat, lon)
            dt = cur_t - self.prev_time
            if dt > 0:
                self.lbl["speed"].configure(
                    text=f"{dist/dt*3.6:.1f} km/h · {dist/dt*1.94384:.1f} kn")
        self.prev_lat, self.prev_lon, self.prev_time = lat, lon, cur_t

        self.lbl["lat"].configure(text=f"{lat:.6f}")
        self.lbl["lon"].configure(text=f"{lon:.6f}")
        self.lbl["last_time"].configure(text=time.strftime("%H:%M:%S"))

        if self.boat_marker is None:
            self.boat_marker = self.map.set_marker(lat, lon, text="⛵",
                                                   marker_color_circle=COL_ACCENT,
                                                   marker_color_outside="#1f3a5f")
        else:
            self.boat_marker.set_position(lat, lon)
        self.boat_track.append((lat, lon))
        if len(self.boat_track) >= 2:
            if self.boat_path is None:
                self.boat_path = self.map.set_path(self.boat_track[-2:],
                                                   color="#3498db", width=6)
            else:
                self.boat_path.add_position(lat, lon)

    def _update_battery(self, volts):
        try:
            v = float(volts)
        except (TypeError, ValueError):
            return
        self.battery_text.configure(text=f"{v:.2f} V")
        # 2S LiPo: 6.0 V (vacío) → 8.4 V (lleno)
        frac = max(0.0, min(1.0, (v - 6.0) / (8.4 - 6.0)))
        color = COL_OK if v >= 7.0 else (COL_WARN if v >= 6.6 else COL_BAD)
        self.battery_bar.configure(progress_color=color)
        self.battery_bar.set(frac)

    def _draw_compass(self, wind):
        c = self.compass
        c.delete("all")
        cx, cy, r = 54, 54, 36
        c.create_oval(cx - r, cy - r, cx + r, cy + r, outline="#4a4a4a", width=2,
                      fill="#222b33")
        for txt, dx, dy in (("N", 0, -r - 7), ("S", 0, r + 7),
                            ("E", r + 7, 0), ("O", -r - 7, 0)):
            c.create_text(cx + dx, cy + dy, text=txt, fill="#9a9a9a",
                          font=("Arial", 8, "bold"))
        valid = wind is not None and not (isinstance(wind, float) and math.isnan(wind))
        try:
            w = float(wind) if valid else None
        except (TypeError, ValueError):
            w = None
        if w is None:
            c.create_text(cx, 116, text="Vent  ?°", fill="#cfcfcf",
                          font=("Arial", 10, "bold"))
            return
        ang = math.radians(w)
        ex = cx + (r - 6) * math.sin(ang)
        ey = cy - (r - 6) * math.cos(ang)
        c.create_line(cx, cy, ex, ey, fill=COL_BAD, width=3, arrow=tk.LAST)
        c.create_oval(cx - 3, cy - 3, cx + 3, cy + 3, fill="#dddddd", outline="")
        c.create_text(cx, 116, text=f"Vent  {round(w)}°", fill="#ffffff",
                      font=("Arial", 10, "bold"))

    def _update_windobs(self, message):
        if "wobs" not in message:
            if self._windobs_placed:
                self.windobs_frame.place_forget()
                self._windobs_placed = False
            return
        try:
            p = max(0, min(100, int(message["wobs"])))
        except (TypeError, ValueError):
            p = 0
        has_fix = int(message.get("fix", 0)) == 1
        self.windobs_frame.place(x=16, y=164)
        self._windobs_placed = True
        self.windobs_bar.set(p / 100.0)
        self.windobs_value.configure(
            text="attente fix GPS…" if (p == 0 and not has_fix) else f"{p}%")

    # ====================================================================
    # Mapa: waypoints, ruta, trayectoria
    # ====================================================================
    def _on_map_click(self, coords):
        self._add_waypoint(coords[0], coords[1])

    def _add_waypoint(self, lat, lon, radius_m=DEFAULT_WAYPOINT_RADIUS_M):
        self.waypoints.append({"lat": lat, "lon": lon, "radius_m": radius_m})
        self._redraw_route()

    def _remove_waypoint(self, index):
        if 0 <= index < len(self.waypoints):
            del self.waypoints[index]
            self._redraw_route()

    def _clear_route(self):
        self.waypoints = []
        self._redraw_route()
        self._clear_trajectory()

    def _redraw_route(self):
        for m in self.waypoint_markers:
            m.delete()
        self.waypoint_markers = []
        if self.route_line is not None:
            self.route_line.delete()
            self.route_line = None

        for i, wp in enumerate(self.waypoints):
            m = self.map.set_marker(wp["lat"], wp["lon"], text=f"WP{i+1}")
            self.waypoint_markers.append(m)
        if len(self.waypoints) >= 2:
            self.route_line = self.map.set_path(
                [(wp["lat"], wp["lon"]) for wp in self.waypoints],
                color="#e67e22", width=4)

        # Lista lateral
        for child in self.wp_list_frame.winfo_children():
            child.destroy()
        if not self.waypoints:
            ctk.CTkLabel(self.wp_list_frame, text="Clic sur la carte pour ajouter",
                         text_color=COL_MUTED, font=ctk.CTkFont(size=11)).pack(pady=10)
        else:
            for i, wp in enumerate(self.waypoints):
                row = ctk.CTkFrame(self.wp_list_frame, fg_color="transparent")
                row.pack(fill="x", pady=1)
                ctk.CTkLabel(row, text=f"WP{i+1}  {wp['lat']:.5f}, {wp['lon']:.5f}",
                             anchor="w", font=ctk.CTkFont(size=11)).pack(side="left")
                ctk.CTkButton(row, text="✕", width=26, height=22, fg_color=COL_BAD,
                              hover_color="#c0392b",
                              command=lambda idx=i: self._remove_waypoint(idx)).pack(
                                  side="right")
        # Re-enlaza la rueda a las filas nuevas (sin add → sin duplicar).
        self._bind_wheel(self.scroll, self.scroll._parent_canvas)

    def _toggle_trajectory(self):
        self.show_trajectory = not self.show_trajectory
        self.traj_btn.configure(fg_color=COL_ACCENT if self.show_trajectory else "#555")
        if self.show_trajectory:
            self._render_trajectory()
        else:
            self._clear_trajectory()

    def _clear_trajectory(self):
        if self.trajectory_line is not None:
            self.trajectory_line.delete()
            self.trajectory_line = None

    def _render_trajectory(self):
        self._clear_trajectory()
        t = getattr(self, "_cur_telemetry", None)
        if not t or (t[0] == 0 and t[1] == 0):
            return
        idx = self.last_wp_index if isinstance(self.last_wp_index, int) else 0
        if idx < 0 or idx >= len(self.waypoints):
            return
        wp = self.waypoints[idx]
        path = nav.predict_trajectory(t[0], t[1], t[2], t[3],
                                      wp["lat"], wp["lon"], wp["radius_m"])
        if not path or len(path) < 2:
            return
        self.trajectory_line = self.map.set_path(path, color="#8e44ad", width=3)

    # ====================================================================
    # Órdenes al barco
    # ====================================================================
    def _send(self, message):
        compact = json.dumps({"origin": "server", "type": "command",
                              "message": message}, separators=(",", ":"))
        if len(compact.encode("utf-8")) > MAX_LORA_PACKET_BYTES:
            messagebox.showerror("Trop long",
                                 f"Commande trop longue pour un paquet LoRa "
                                 f"({len(compact.encode('utf-8'))} > {MAX_LORA_PACKET_BYTES} o).")
            return False
        self.link.send_command(message)
        return True

    def _cmd_navigate(self):
        self._send({"navigate": True})

    def _cmd_stop(self):
        self._send({"stop": True})

    def _cmd_restart(self):
        if messagebox.askyesno("Redémarrer", "⚠️ Redémarrer le bateau ?"):
            self._send({"restart": True})

    def _cmd_wind_observation(self):
        if self._send({"wind-observation": True}):
            messagebox.showinfo(
                "Mesure du vent",
                "Mesure lancée.\n\nLe bateau doit être en mode Auto, avec un fix "
                "GPS, et se déplacer (~30 m) pour que la mesure progresse.")

    def _cancel_windobs(self):
        if messagebox.askyesno("Annuler", "Annuler la mesure du vent ?"):
            self._send({"stop": True})

    def _cmd_wind_command(self):
        val = simpledialog.askinteger("Direction du vent",
                                      "Direction du vent (en degrés) :",
                                      minvalue=0, maxvalue=359)
        if val is not None:
            self._send({"wind-command": {"value": val}})

    def _send_route(self):
        if not self.waypoints:
            messagebox.showwarning("Route", "Aucun waypoint à envoyer.")
            return
        points = ",".join(f"{wp['lat']:.8f},{wp['lon']:.8f}" for wp in self.waypoints)
        message = {"waypoints": {"number": len(self.waypoints), "points": points}}
        if self._send(message):
            messagebox.showinfo("Route", "Route envoyée au bateau.")

    def _import_geojson(self):
        path = filedialog.askopenfilename(
            title="Importer une route GeoJSON",
            filetypes=[("GeoJSON", "*.geojson"), ("Tous", "*.*")])
        if not path:
            return
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = json.load(f)
            if data.get("type") != "FeatureCollection" or not data.get("features"):
                raise ValueError("FeatureCollection attendue.")
            geom = data["features"][0]["geometry"]
            if geom["type"] != "LineString":
                raise ValueError("LineString attendue.")
            coords = geom["coordinates"]
        except Exception as e:
            messagebox.showerror("GeoJSON", f"Fichier .geojson invalide.\n{e}")
            return
        self._clear_route()
        for lon, lat in coords:
            self._add_waypoint(lat, lon)
        self._send_route()

    # ====================================================================
    # Puerto / transceiver
    # ====================================================================
    def _refresh_ports(self):
        self._port_map = {"auto": None,
                          "/tmp/ttyV1 (simulation)": "/tmp/ttyV1"}
        options = ["auto"]
        for dev, desc in list_serial_ports():
            label = f"{dev} — {desc}"
            options.append(label)
            self._port_map[label] = dev
        options.append("/tmp/ttyV1 (simulation)")
        self.port_menu.configure(values=options)

    def _apply_port(self):
        sel = self.port_var.get().strip()
        self.link.set_port(self._port_map.get(sel))

    def _reset_transceiver(self):
        if not messagebox.askyesno("Reset TRX",
                                   "🔌 Réinitialiser le transceiver et reconnecter ?"):
            return
        port = self.link.current_port or self.link.forced_port
        if not port:
            from serial_link import find_transceiver_port
            port = find_transceiver_port(self.link.serial_number)
        if not port:
            messagebox.showerror("Reset TRX", "Transceiver introuvable.")
            return

        def worker():
            try:
                self.link.stop()
                time.sleep(0.5)
                reset_esp32(port)
                time.sleep(0.5)
            except Exception as e:
                self.events.put(("error", str(e)))
            finally:
                self.link.start()
        threading.Thread(target=worker, daemon=True).start()

    def _on_close(self):
        try:
            self.link.stop()
        except Exception:
            pass
        self.root.destroy()


def main():
    root = ctk.CTk()
    AutoBoatIHM(root)
    root.mainloop()


if __name__ == "__main__":
    main()
