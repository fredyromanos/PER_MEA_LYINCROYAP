"""
Source de prévision de vent Windy.com — donne à l'opérateur une direction de
vent RÉELLE (prévision) à recopier dans la commande existante
`/api/wind-command/{direction}`, au lieu de deviner une valeur.

Module volontairement PUR autant que possible : la seule fonction impure est
fetch_point_forecast() (appel réseau), et elle est injectable (paramètre
`fetcher`) pour être testée hors-ligne avec un faux appel ou une réponse
Windy brute déjà enregistrée. Tout le reste (parsing de la réponse, maths
u/v -> direction/vitesse, sélection du timestamp le plus proche) est stdlib
pur et testable sans réseau.

Bibliothèque standard uniquement (urllib, json, os, math, time) — pas de
dépendance `requests` ni `python-dotenv` ajoutée ici.

SÉCURITÉ CLÉ API : WINDY_API_KEY n'est jamais loggée, printée, ni incluse
dans un message d'erreur. load_windy_api_key() la lit depuis os.environ,
avec IHM/.env comme repli (mini-parseur maison, cf. _parse_env_file). Si la
clé est absente, l'erreur renvoyée ne contient QUE le chemin de fichier
recherché — jamais une valeur de clé (il n'y en a pas, elle est absente).
"""
import json
import math
import os
import time
import urllib.error
import urllib.request
from typing import Any, Callable, Optional

WINDY_ENDPOINT = "https://api.windy.com/api/point-forecast/v2"
DEFAULT_TIMEOUT_S = 10


# ============================================================
# Erreurs — sous-classées pour que l'endpoint puisse choisir un code HTTP
# adapté sans avoir à parser le texte du message.
# ============================================================

class WindyError(Exception):
    """Erreur "propre" du module : message toujours sûr (jamais la clé API)."""


class WindyConfigError(WindyError):
    """Clé API absente / mal configurée."""


class WindyValidationError(WindyError):
    """lat/lon non numériques ou hors bornes."""


class WindyNetworkError(WindyError):
    """Échec réseau : URLError / HTTPError / timeout."""


class WindyParseError(WindyError):
    """Réponse Windy illisible (JSON invalide ou forme inattendue)."""


# ============================================================
# Clé API — lecture env / .env, jamais exposée
# ============================================================

def _default_env_path() -> str:
    """IHM/app/windy.py -> IHM/.env (le fichier .env du projet, gitignored)."""
    return os.path.normpath(
        os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".env")
    )


def _parse_env_file(path: str) -> dict:
    """
    Mini-parseur .env maison (KEY=VALUE par ligne). Volontairement minimal :
    pas de dépendance python-dotenv supplémentaire dans ce module. Ignore les
    lignes vides / commentaires (#), tolère un préfixe "export ", et retire
    les guillemets simples/doubles entourant la valeur.

    Ne lève jamais : un .env absent ou illisible renvoie simplement {}.
    """
    values: dict = {}
    try:
        with open(path, "r", encoding="utf-8") as f:
            for raw_line in f:
                line = raw_line.strip()
                if not line or line.startswith("#"):
                    continue
                if line.startswith("export "):
                    line = line[len("export "):].strip()
                if "=" not in line:
                    continue
                key, _, value = line.partition("=")
                key = key.strip()
                value = value.strip()
                if len(value) >= 2 and value[0] == value[-1] and value[0] in ("'", '"'):
                    value = value[1:-1]
                if key:
                    values[key] = value
    except OSError:
        return {}
    return values


def load_windy_api_key(env_path: Optional[str] = None) -> str:
    """
    Renvoie WINDY_API_KEY : d'abord os.environ, sinon IHM/.env (mini-parseur
    ci-dessus). `env_path` permet aux tests de pointer vers un faux fichier.

    Lève WindyConfigError si la clé est introuvable dans les deux sources.
    Le message d'erreur ne contient JAMAIS de valeur de clé — seulement le
    chemin de fichier recherché (utile pour le diagnostic opérateur).
    """
    key = os.environ.get("WINDY_API_KEY")
    if key:
        return key

    path = env_path or _default_env_path()
    key = _parse_env_file(path).get("WINDY_API_KEY")
    if key:
        return key

    raise WindyConfigError(
        "WINDY_API_KEY manquante : ni variable d'environnement, ni "
        f"{path}. Provisionner la clé Windy avant d'utiliser /api/wind-forecast."
    )


# ============================================================
# Validation lat/lon — cohérente en bornes avec validate_coordinates()
# de routes/messages.py, mais définie ici pour que ce module reste
# testable/importable seul (sans FastAPI/pydantic/pymongo). Ne reprend PAS
# la règle métier "(0,0) = sentinelle GPS rejetée" de validate_coordinates,
# qui concerne les positions du bateau, pas un point de prévision météo
# arbitraire ; l'endpoint web (routes/messages.py) applique lui-même
# validate_coordinates en plus de cette vérification interne.
# ============================================================

def validate_lat_lon(lat: Any, lon: Any) -> tuple:
    """Renvoie (lat, lon) en float. Lève WindyValidationError sinon."""
    try:
        lat_f = float(lat)
        lon_f = float(lon)
    except (TypeError, ValueError):
        raise WindyValidationError(f"lat/lon non numériques : lat={lat!r}, lon={lon!r}")

    if math.isnan(lat_f) or math.isnan(lon_f):
        raise WindyValidationError(f"lat/lon invalides (NaN) : lat={lat!r}, lon={lon!r}")

    if not (-90.0 <= lat_f <= 90.0):
        raise WindyValidationError(f"Latitude hors limites [-90, 90] : {lat_f}")

    if not (-180.0 <= lon_f <= 180.0):
        raise WindyValidationError(f"Longitude hors limites [-180, 180] : {lon_f}")

    return lat_f, lon_f


# ============================================================
# Maths u/v -> direction/vitesse — LE cœur du module
# ============================================================

def wind_from_uv(u: float, v: float) -> tuple:
    """
    u = composante EST du vent (m/s), v = composante NORD (m/s).

    Renvoie (direction_deg_int, speed_ms) :
      - direction_deg_int : cap météo D'OÙ VIENT le vent (convention
        "from"), entier dans [0, 360), = round(degrees(atan2(-u, -v))) % 360
      - speed_ms : norme du vecteur vent, hypot(u, v), en m/s (float)

    Vérifié sur les 4 cas cardinaux :
      u=0,v=-5  (air va au sud)  -> vent DU nord  -> 0°
      u=-5,v=0  (air va à l'ouest) -> vent DE l'est -> 90°
      u=0,v=5   (air va au nord) -> vent DU sud   -> 180°
      u=5,v=0   (air va à l'est) -> vent DE l'ouest -> 270°
    """
    speed = math.hypot(u, v)
    raw_deg = math.degrees(math.atan2(-u, -v)) % 360.0
    direction = int(round(raw_deg)) % 360
    return direction, speed


# ============================================================
# Appel réseau — SEULE partie impure, injectable pour les tests
# ============================================================

def _default_fetcher(url: str, data: bytes, headers: dict, timeout: float) -> bytes:
    req = urllib.request.Request(url, data=data, headers=headers, method="POST")
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        return resp.read()


def fetch_point_forecast(
    lat: float,
    lon: float,
    *,
    model: str = "gfs",
    api_key: Optional[str] = None,
    timeout: float = DEFAULT_TIMEOUT_S,
    fetcher: Optional[Callable[[str, bytes, dict, float], bytes]] = None,
) -> dict:
    """
    POST vers l'API Windy point-forecast v2, renvoie le JSON décodé (dict
    brut, tel quel). C'est la SEULE fonction du module qui touche le réseau.

    `fetcher(url, data_bytes, headers, timeout) -> bytes` permet aux tests
    d'injecter une fausse réponse (ou une fausse erreur) sans réseau. Par
    défaut : urllib.request (stdlib seule).

    Lève :
      - WindyValidationError si lat/lon est hors bornes/non numérique
      - WindyConfigError si la clé API est absente
      - WindyNetworkError sur URLError/HTTPError/timeout/erreur fetcher
      - WindyParseError si la réponse n'est pas un JSON dict

    Le message d'erreur ne contient JAMAIS la clé API (elle n'apparaît que
    dans le corps de la requête POST, jamais réinjectée dans un message).
    """
    lat_f, lon_f = validate_lat_lon(lat, lon)

    key = api_key if api_key is not None else load_windy_api_key()

    payload = {
        "lat": lat_f,
        "lon": lon_f,
        "model": model,
        "parameters": ["wind"],
        "levels": ["surface"],
        "key": key,
    }
    body = json.dumps(payload).encode("utf-8")
    headers = {"Content-Type": "application/json"}

    do_fetch = fetcher or _default_fetcher

    try:
        raw = do_fetch(WINDY_ENDPOINT, body, headers, timeout)
    except urllib.error.HTTPError as e:
        raise WindyNetworkError(f"Windy API a répondu une erreur HTTP {e.code}.") from None
    except urllib.error.URLError as e:
        raise WindyNetworkError(f"Windy API injoignable : {e.reason}.") from None
    except TimeoutError:
        raise WindyNetworkError(f"Windy API : délai dépassé ({timeout}s).") from None
    except Exception as e:
        # Filet de sécurité : quel que soit le fetcher injecté, on ne laisse
        # jamais fuiter son exception brute (qui pourrait contenir des
        # détails imprévus) — seulement le type d'erreur.
        raise WindyNetworkError(f"Erreur réseau Windy API ({type(e).__name__}).") from None

    try:
        data = json.loads(raw)
    except (json.JSONDecodeError, TypeError, UnicodeDecodeError):
        raise WindyParseError("Réponse Windy API illisible (JSON invalide).") from None

    if not isinstance(data, dict):
        raise WindyParseError("Réponse Windy API inattendue (pas un objet JSON).")

    return data


# ============================================================
# Parsing de la réponse — jamais d'exception, toujours un résultat clair
# ============================================================

def wind_at_nearest_time(forecast: dict, target_ts_ms: Optional[int] = None) -> dict:
    """
    Extrait le vent au timestamp le plus proche de target_ts_ms (epoch ms)
    depuis une réponse Windy brute (dict avec 'ts', 'wind_u-surface',
    'wind_v-surface'). Si target_ts_ms est None, utilise l'instant présent.

    Renvoie soit :
      {"direction_deg": int, "speed_ms": float, "timestamp_ms": int,
       "model": str}
    soit, sur réponse malformée (clés manquantes, tableaux vides ou de
    longueurs différentes, entrées None) :
      {"error": "<message clair>", "error_type": "malformed_response"}

    Ne lève JAMAIS d'exception — cette fonction est le filet de sécurité
    pour tout ce qui peut arriver depuis une réponse HTTP externe.
    """
    if not isinstance(forecast, dict):
        return {"error": "Réponse de prévision invalide (pas un objet).",
                "error_type": "malformed_response"}

    ts = forecast.get("ts")
    u = forecast.get("wind_u-surface")
    v = forecast.get("wind_v-surface")

    if not isinstance(ts, list) or len(ts) == 0:
        return {"error": "Prévision sans horodatages (champ 'ts' manquant ou vide).",
                "error_type": "malformed_response"}

    if not isinstance(u, list) or not isinstance(v, list) or len(u) == 0 or len(v) == 0:
        return {"error": "Prévision sans composantes de vent "
                          "('wind_u-surface'/'wind_v-surface' manquant ou vide).",
                "error_type": "malformed_response"}

    if not (len(ts) == len(u) == len(v)):
        return {"error": "Tableaux de prévision de longueurs incohérentes "
                          f"(ts={len(ts)}, wind_u={len(u)}, wind_v={len(v)}).",
                "error_type": "malformed_response"}

    if target_ts_ms is None:
        target_ts_ms = int(time.time() * 1000)

    best_idx: Optional[int] = None
    best_delta: Optional[float] = None

    for i, t in enumerate(ts):
        if t is None or u[i] is None or v[i] is None:
            continue
        try:
            t_f = float(t)
            float(u[i])
            float(v[i])
        except (TypeError, ValueError):
            continue

        delta = abs(t_f - float(target_ts_ms))
        if best_delta is None or delta < best_delta:
            best_delta = delta
            best_idx = i

    if best_idx is None:
        return {"error": "Aucune entrée de prévision exploitable (valeurs nulles/invalides).",
                "error_type": "malformed_response"}

    direction_deg, speed_ms = wind_from_uv(float(u[best_idx]), float(v[best_idx]))

    return {
        "direction_deg": direction_deg,
        "speed_ms": speed_ms,
        "timestamp_ms": int(ts[best_idx]),
        "model": forecast.get("model", "gfs"),
    }


# ============================================================
# Point d'entrée pratique pour l'endpoint web — combine fetch + parsing,
# ne lève JAMAIS (toujours un dict "résultat" ou "erreur").
# ============================================================

def get_wind_forecast(
    lat: float,
    lon: float,
    *,
    model: str = "gfs",
    target_ts_ms: Optional[int] = None,
    api_key: Optional[str] = None,
    fetcher: Optional[Callable[[str, bytes, dict, float], bytes]] = None,
    timeout: float = DEFAULT_TIMEOUT_S,
) -> dict:
    """
    fetch_point_forecast() + wind_at_nearest_time() en un appel, avec gestion
    d'erreur uniforme. Utilisé par GET /api/wind-forecast (routes/messages.py).

    Renvoie toujours un dict : soit le résultat structuré (+ lat/lon repris
    en écho), soit {"error": "...", "error_type": "..."} — jamais
    d'exception, jamais la clé API dans le résultat.
    """
    try:
        forecast = fetch_point_forecast(
            lat, lon, model=model, api_key=api_key, fetcher=fetcher, timeout=timeout
        )
    except WindyError as e:
        return {"error": str(e), "error_type": type(e).__name__}

    result = wind_at_nearest_time(forecast, target_ts_ms=target_ts_ms)
    if "error" in result:
        return result

    result["lat"] = lat
    result["lon"] = lon
    return result
