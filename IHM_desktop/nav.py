"""
nav.py — Réplica Python de navigation.h (firmware) + simulador de trayectoria.

Port 1:1 de IHM/app/static/nav.js (que a su vez replica
main/src/navigation/navigation.h). La LÓGICA DE NAVEGACIÓN es idéntica al
firmware — no divergir. El único añadido es un modelo cinemático simple
(el cap sigue al timón, velocidad constante): el TRAZADO es aproximado
aunque la lógica de barra/vela es exacta.

Convención: windDir = dirección DE DÓNDE VIENE el viento (igual que el firmware).
"""

import math

# --- Constantes (copiadas de navigation.h) ---
NAV_DIRECT_DEAD_ZONE_DEG = 5.0
NAV_VDB_RUDDER_GAIN = 0.8
NAV_DIRECT_RUDDER_GAIN = 0.5
NAV_RUDDER_LIMIT_DEG = 20.0
NAV_DEFAULT_CORRIDOR_HALF_WIDTH_M = 20.0
NAV_GEO_EPSILON_DEG = 0.0000001
NAV_EARTH_RADIUS_M = 6371000.0
NAV_UPWIND_FORBIDDEN_ANGLE_DEG = 45.0
NAV_DOWNWIND_FORBIDDEN_ANGLE_DEG = 20.0
NAV_RUDDER_STEP_DEG = 5.0
NAV_SAIL_RIGHT_DEG = 10.0
NAV_SAIL_LEFT_DEG = -10.0
DEG2RAD = math.pi / 180.0

# --- Parámetros del modelo cinemático (PROPIOS del simulador) ---
SIM_STEP_M = 2.5
SIM_STEER_GAIN = 0.45
SIM_MAX_TURN_PER_STEP = 12.0
SIM_MAX_STEPS = 2500
SIM_MAX_PATH_M = 4000.0


# --- Helpers angulares ---
def relative_angle(ref_deg, target_deg):
    rel = target_deg - ref_deg
    if rel > 180:
        rel -= 360
    elif rel < -180:
        rel += 360
    return rel


def same_sign(a, b):
    return (a >= 0 and b >= 0) or (a < 0 and b < 0)


def opposite_angle(a):
    o = a + 180
    return o - 360 if o >= 360 else o


def angle_side(a):
    return -1 if a < 0 else 1


def clamp_rudder(r):
    if r > NAV_RUDDER_LIMIT_DEG:
        return NAV_RUDDER_LIMIT_DEG
    if r < -NAV_RUDDER_LIMIT_DEG:
        return -NAV_RUDDER_LIMIT_DEG
    return r


def normalize_angle(a):
    while a >= 360:
        a -= 360
    while a < 0:
        a += 360
    return a


def is_between_on_shortest_turn(angle_deg, start_deg, end_deg):
    turn = relative_angle(start_deg, end_deg)
    frm = relative_angle(start_deg, angle_deg)
    if turn >= 0:
        return 0 < frm < turn
    return turn < frm < 0


# --- Geodesia ---
def gps_delta_meters(ref_lat, ref_lng, lat, lng):
    lat_rad = ref_lat * DEG2RAD
    north_m = (lat - ref_lat) * NAV_EARTH_RADIUS_M * DEG2RAD
    east_m = (lng - ref_lng) * NAV_EARTH_RADIUS_M * math.cos(lat_rad) * DEG2RAD
    return east_m, north_m


def distance_m(lat1, lon1, lat2, lon2):
    d_lat = (lat2 - lat1) * DEG2RAD
    d_lon = (lon2 - lon1) * DEG2RAD
    a = (math.sin(d_lat / 2) ** 2 +
         math.cos(lat1 * DEG2RAD) * math.cos(lat2 * DEG2RAD) * math.sin(d_lon / 2) ** 2)
    return NAV_EARTH_RADIUS_M * 2.0 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def bearing_deg(lat1, lon1, lat2, lon2):
    d_lon = (lon2 - lon1) * DEG2RAD
    y = math.sin(d_lon) * math.cos(lat2 * DEG2RAD)
    x = (math.cos(lat1 * DEG2RAD) * math.sin(lat2 * DEG2RAD) -
         math.sin(lat1 * DEG2RAD) * math.cos(lat2 * DEG2RAD) * math.cos(d_lon))
    return (math.atan2(y, x) / DEG2RAD + 360.0) % 360.0


def destination_point(lat, lon, bearing, dist_m):
    lat_rad = lat * DEG2RAD
    north_m = dist_m * math.cos(bearing * DEG2RAD)
    east_m = dist_m * math.sin(bearing * DEG2RAD)
    new_lat = lat + (north_m / NAV_EARTH_RADIUS_M) / DEG2RAD
    new_lon = lon + (east_m / (NAV_EARTH_RADIUS_M * math.cos(lat_rad))) / DEG2RAD
    return new_lat, new_lon


def cross_track_error_meters(s_lat, s_lng, e_lat, e_lng, b_lat, b_lng):
    p_e, p_n = gps_delta_meters(s_lat, s_lng, e_lat, e_lng)
    b_e, b_n = gps_delta_meters(s_lat, s_lng, b_lat, b_lng)
    length = math.sqrt(p_e * p_e + p_n * p_n)
    if length < 0.1:
        return 0.0
    return (p_n * b_e - p_e * b_n) / length


def side_moving_toward_cross_track(s_lat, s_lng, e_lat, e_lng, axis_deg, safe_angle_deg,
                                   desired_cross_track_sign, fallback_side):
    p_e, p_n = gps_delta_meters(s_lat, s_lng, e_lat, e_lng)
    length = math.sqrt(p_e * p_e + p_n * p_n)
    if length < 0.1:
        return fallback_side
    p_eu, p_nu = p_e / length, p_n / length
    best_side, best_score = fallback_side, -1.0
    for side in (-1, 1):
        head_rad = normalize_angle(axis_deg + side * safe_angle_deg) * DEG2RAD
        cross_vel = p_nu * math.sin(head_rad) - p_eu * math.cos(head_rad)
        want_sign = (cross_vel > 0) if desired_cross_track_sign > 0 else (cross_vel < 0)
        if want_sign:
            score = abs(cross_vel)
            if score > best_score:
                best_score = score
                best_side = side
    return best_side


# --- Estado de navegación ---
class NavState:
    def __init__(self):
        self.corridor = dict(initialized=False, start_lat=0.0, start_lng=0.0,
                             target_lat=0.0, target_lng=0.0)
        self.upwind_side = 0
        self.downwind_side = 0
        self.empannage_phase = 0       # 0=AUCUN,1=ALLER_UPWIND,2=CROISER,3=REVENIR_DOWNWIND
        self.empannage_target_side = 0

    def reset_corridor(self):
        self.corridor = dict(initialized=False, start_lat=0.0, start_lng=0.0,
                             target_lat=0.0, target_lng=0.0)

    def reset(self):
        self.reset_corridor()
        self.upwind_side = 0
        self.downwind_side = 0
        self.empannage_phase = 0
        self.empannage_target_side = 0


class NavResult:
    def __init__(self, sail, rudder):
        self.sail_angle = sail
        self.rudder_angle = rudder
        self.mode = "direct"
        self.log_message = ""
        self.waypoint_reached = False


def has_corridor(b_lat, b_lng, w_lat, w_lng, half_w):
    return half_w > 0.0 and (abs(w_lat - b_lat) > NAV_GEO_EPSILON_DEG or
                             abs(w_lng - b_lng) > NAV_GEO_EPSILON_DEG)


def update_corridor(s, hc, b_lat, b_lng, w_lat, w_lng):
    if not hc:
        s.reset_corridor()
        return
    target_changed = (abs(w_lat - s.corridor["target_lat"]) > NAV_GEO_EPSILON_DEG or
                      abs(w_lng - s.corridor["target_lng"]) > NAV_GEO_EPSILON_DEG)
    if s.corridor["initialized"] and not target_changed:
        return
    s.corridor = dict(initialized=True, start_lat=b_lat, start_lng=b_lng,
                      target_lat=w_lat, target_lng=w_lng)
    s.upwind_side = 0
    s.downwind_side = 0
    s.empannage_phase = 0
    s.empannage_target_side = 0


def apply_target_heading(r, boat_heading, target_heading, relative_wind, rudder_gain, mode, log_message):
    err = relative_angle(boat_heading, target_heading)
    r.sail_angle = NAV_SAIL_LEFT_DEG if relative_wind < 0 else NAV_SAIL_RIGHT_DEG
    r.rudder_angle = clamp_rudder(-err * rudder_gain)
    r.mode = mode
    r.log_message = log_message


def corridor_side(s, b_lat, b_lng, half_w, axis_deg, safe_angle_deg, current_side):
    if not s.corridor["initialized"]:
        return current_side
    cte = cross_track_error_meters(s.corridor["start_lat"], s.corridor["start_lng"],
                                   s.corridor["target_lat"], s.corridor["target_lng"], b_lat, b_lng)
    if cte > half_w:
        return side_moving_toward_cross_track(s.corridor["start_lat"], s.corridor["start_lng"],
                                              s.corridor["target_lat"], s.corridor["target_lng"],
                                              axis_deg, safe_angle_deg, -1, current_side)
    if cte < -half_w:
        return side_moving_toward_cross_track(s.corridor["start_lat"], s.corridor["start_lng"],
                                              s.corridor["target_lat"], s.corridor["target_lng"],
                                              axis_deg, safe_angle_deg, 1, current_side)
    return current_side


def start_avoid_empannage(r, s, boat_heading, wind_dir, relative_wind, target_side, log_message):
    s.empannage_target_side = target_side
    s.empannage_phase = 1
    target_heading = normalize_angle(wind_dir - s.downwind_side * NAV_UPWIND_FORBIDDEN_ANGLE_DEG)
    apply_target_heading(r, boat_heading, target_heading, relative_wind,
                         NAV_DIRECT_RUDDER_GAIN, "avoid-gybe", log_message)


def handle_empannage_loop(r, s, boat_heading, wind_dir, opposite_wind, relative_wind):
    mode = "avoid-gybe"
    log_message = "Avoid empannage: loop around through upwind"
    if s.empannage_phase == 1:
        target_heading = normalize_angle(wind_dir - s.downwind_side * NAV_UPWIND_FORBIDDEN_ANGLE_DEG)
        if abs(relative_angle(boat_heading, target_heading)) < 10.0:
            s.empannage_phase = 2
    elif s.empannage_phase == 2:
        target_heading = normalize_angle(wind_dir + s.downwind_side * NAV_UPWIND_FORBIDDEN_ANGLE_DEG)
        mode = "vdb"
        log_message = "VDB: downwind loop crosses upwind axis"
        if angle_side(relative_angle(wind_dir, boat_heading)) == s.downwind_side:
            s.downwind_side = s.empannage_target_side
            s.empannage_phase = 3
    else:
        target_heading = normalize_angle(opposite_wind + s.empannage_target_side * NAV_DOWNWIND_FORBIDDEN_ANGLE_DEG)
        if angle_side(relative_angle(opposite_wind, boat_heading)) == s.empannage_target_side:
            s.downwind_side = s.empannage_target_side
            s.empannage_phase = 0
    apply_target_heading(r, boat_heading, target_heading, relative_wind,
                         NAV_DIRECT_RUDDER_GAIN, mode, log_message)


def handle_forbidden_zone(r, s, boat_heading, b_lat, b_lng, wind_axis_deg, relative_wind, hc, half_w, upwind):
    side_key = "upwind_side" if upwind else "downwind_side"
    forbidden_angle = NAV_UPWIND_FORBIDDEN_ANGLE_DEG if upwind else NAV_DOWNWIND_FORBIDDEN_ANGLE_DEG
    gain = NAV_VDB_RUDDER_GAIN if upwind else NAV_DIRECT_RUDDER_GAIN
    mode = "upwind-zigzag" if upwind else "downwind-zigzag"
    log_message = ("Waypoint in upwind forbidden zone" if upwind
                   else "Waypoint in downwind forbidden zone")
    if getattr(s, side_key) == 0:
        setattr(s, side_key, angle_side(relative_angle(wind_axis_deg, boat_heading)))
    if hc:
        setattr(s, side_key, corridor_side(s, b_lat, b_lng, half_w, wind_axis_deg,
                                           forbidden_angle, getattr(s, side_key)))
    target_heading = normalize_angle(wind_axis_deg + getattr(s, side_key) * forbidden_angle)
    apply_target_heading(r, boat_heading, target_heading, relative_wind, gain, mode, log_message)


def handle_downwind_zigzag(r, s, boat_heading, b_lat, b_lng, wind_dir, opposite_wind, relative_wind, hc, half_w):
    if s.downwind_side == 0:
        s.downwind_side = angle_side(relative_angle(opposite_wind, boat_heading))
    desired_side = s.downwind_side
    if hc:
        desired_side = corridor_side(s, b_lat, b_lng, half_w, opposite_wind,
                                     NAV_DOWNWIND_FORBIDDEN_ANGLE_DEG, s.downwind_side)
    if desired_side != s.downwind_side:
        start_avoid_empannage(r, s, boat_heading, wind_dir, relative_wind, desired_side,
                              "Avoid empannage: loop around through upwind")
        return
    handle_forbidden_zone(r, s, boat_heading, b_lat, b_lng, opposite_wind, relative_wind, False, half_w, False)


def handle_lofer_abattre(r, relative_wind, current_rudder_angle, lofer):
    wind_left = relative_wind < 0
    r.sail_angle = NAV_SAIL_LEFT_DEG if wind_left else NAV_SAIL_RIGHT_DEG
    step = NAV_RUDDER_STEP_DEG if (wind_left == lofer) else -NAV_RUDDER_STEP_DEG
    r.rudder_angle = current_rudder_angle + step
    r.mode = "lofer" if lofer else "abattre"


def handle_navigation_with_state(state, boat_heading, wpt_heading, wpt_dist, wind_dir,
                                 current_sail, current_rudder, reached_dist,
                                 b_lat, b_lng, w_lat, w_lng, half_w):
    r = NavResult(current_sail, current_rudder)
    if wpt_dist <= reached_dist:
        r.waypoint_reached = True
        state.reset()
        return r

    hc = has_corridor(b_lat, b_lng, w_lat, w_lng, half_w)
    update_corridor(state, hc, b_lat, b_lng, w_lat, w_lng)

    opposite_wind = opposite_angle(wind_dir)
    relative_wind = relative_angle(boat_heading, wind_dir)
    relative_wpt = relative_angle(boat_heading, wpt_heading)
    wp_up = abs(relative_angle(wind_dir, wpt_heading)) < NAV_UPWIND_FORBIDDEN_ANGLE_DEG
    wp_down = abs(relative_angle(opposite_wind, wpt_heading)) < NAV_DOWNWIND_FORBIDDEN_ANGLE_DEG
    crosses_up = is_between_on_shortest_turn(wind_dir, boat_heading, wpt_heading)
    crosses_down = is_between_on_shortest_turn(opposite_wind, boat_heading, wpt_heading)

    if state.empannage_phase != 0:
        handle_empannage_loop(r, state, boat_heading, wind_dir, opposite_wind, relative_wind)
    elif wp_up or crosses_up:
        handle_forbidden_zone(r, state, boat_heading, b_lat, b_lng, wind_dir, relative_wind, hc, half_w, True)
    elif wp_down:
        handle_downwind_zigzag(r, state, boat_heading, b_lat, b_lng, wind_dir, opposite_wind, relative_wind, hc, half_w)
    elif crosses_down:
        if state.downwind_side == 0:
            state.downwind_side = angle_side(relative_angle(opposite_wind, boat_heading))
        start_avoid_empannage(r, state, boat_heading, wind_dir, relative_wind, -state.downwind_side,
                              "Avoid empannage: direct path crosses downwind axis")
    elif abs(relative_wpt) <= NAV_DIRECT_DEAD_ZONE_DEG:
        r.sail_angle = NAV_SAIL_LEFT_DEG if relative_wind < 0 else NAV_SAIL_RIGHT_DEG
        r.rudder_angle = 0
        r.log_message = "Direct: waypoint aligned"
    else:
        handle_lofer_abattre(r, relative_wind, current_rudder, same_sign(relative_wind, relative_wpt))

    r.rudder_angle = clamp_rudder(r.rudder_angle)
    return r


def predict_trajectory(boat_lat, boat_lon, boat_heading, wind_dir, wp_lat, wp_lon, reached_dist):
    """Devuelve [(lat, lon), ...] del barco hacia el waypoint."""
    rdist = reached_dist if (reached_dist and reached_dist > 0) else 10.0
    state = NavState()
    lat, lon = boat_lat, boat_lon
    heading = (bearing_deg(lat, lon, wp_lat, wp_lon)
               if boat_heading is None else boat_heading)
    sail = NAV_SAIL_RIGHT_DEG
    rudder = 0.0

    path = [(lat, lon)]
    travelled = 0.0

    for _ in range(SIM_MAX_STEPS):
        dist = distance_m(lat, lon, wp_lat, wp_lon)
        brg = bearing_deg(lat, lon, wp_lat, wp_lon)
        r = handle_navigation_with_state(state, heading, brg, dist, wind_dir,
                                         sail, rudder, rdist, lat, lon, wp_lat, wp_lon,
                                         NAV_DEFAULT_CORRIDOR_HALF_WIDTH_M)
        if r.waypoint_reached:
            path.append((wp_lat, wp_lon))
            break

        sail = r.sail_angle
        rudder = r.rudder_angle

        d_h = -rudder * SIM_STEER_GAIN
        if d_h > SIM_MAX_TURN_PER_STEP:
            d_h = SIM_MAX_TURN_PER_STEP
        if d_h < -SIM_MAX_TURN_PER_STEP:
            d_h = -SIM_MAX_TURN_PER_STEP
        heading = normalize_angle(heading + d_h)

        lat, lon = destination_point(lat, lon, heading, SIM_STEP_M)
        travelled += SIM_STEP_M
        path.append((lat, lon))

        if travelled > SIM_MAX_PATH_M:
            break

    return path
