"""
Tests hors-ligne pour app/windy.py — la source de prévision de vent
Windy.com qui alimente le nouvel endpoint GET /api/wind-forecast.

Aucun de ces tests ne touche le réseau : fetch_point_forecast() est toujours
appelée avec un `fetcher` injecté (faux appel) ou contournée en testant
wind_at_nearest_time() directement sur un dict fabriqué à la main. L'appel
réel à api.windy.com reste NON VÉRIFIÉ par cette suite (pas de réseau sortant
dans ce sandbox) — voir le rapport de livraison.

`windy` est importé directement (pas besoin de la fixture `messages` de
conftest.py) car le module ne dépend que de la stdlib : aucune des fakes
pydantic/FastAPI/pymongo n'est nécessaire ici, et donc aucun des tests de ce
fichier n'est "vacueux" à cause d'elles — ce sont de vrais tests du code
Python réel de windy.py (contrairement à un test qui dépendrait d'une
contrainte pydantic Field, invisible sous le stub sans validation de
conftest.py).
"""
import json
import math
import os
import sys
import urllib.error

import pytest

APP_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "app"))
if APP_DIR not in sys.path:
    sys.path.insert(0, APP_DIR)

import windy  # noqa: E402  (sys.path setup must happen first)


# ── wind_from_uv — cas cardinaux (REAL tests, pure math, no fakes involved) ──

def test_wind_from_uv_air_moving_south_is_from_north():
    # u=0, v=-5 : l'air va vers le sud -> le vent vient DU nord -> 0°
    direction, speed = windy.wind_from_uv(0, -5)
    assert direction == 0
    assert speed == pytest.approx(5.0)


def test_wind_from_uv_air_moving_west_is_from_east():
    # u=-5, v=0 : l'air va vers l'ouest -> le vent vient DE l'est -> 90°
    direction, speed = windy.wind_from_uv(-5, 0)
    assert direction == 90
    assert speed == pytest.approx(5.0)


def test_wind_from_uv_air_moving_north_is_from_south():
    # u=0, v=5 : l'air va vers le nord -> le vent vient DU sud -> 180°
    direction, speed = windy.wind_from_uv(0, 5)
    assert direction == 180
    assert speed == pytest.approx(5.0)


def test_wind_from_uv_air_moving_east_is_from_west():
    # u=5, v=0 : l'air va vers l'est -> le vent vient DE l'ouest -> 270°
    direction, speed = windy.wind_from_uv(5, 0)
    assert direction == 270
    assert speed == pytest.approx(5.0)


def test_wind_from_uv_diagonal_case():
    # u=-5, v=-5 : l'air va vers le sud-ouest -> le vent vient DU nord-est -> 45°
    direction, speed = windy.wind_from_uv(-5, -5)
    assert direction == 45
    assert speed == pytest.approx(math.hypot(5, 5))


def test_wind_from_uv_direction_always_in_0_360():
    # Balayage grossier : jamais de valeur hors [0, 360), jamais 360 pile
    # (round(raw_deg) pourrait déborder à 360 sans le second % 360).
    for u in (-9.9, -0.001, 0, 0.001, 9.9):
        for v in (-9.9, -0.001, 0, 0.001, 9.9):
            direction, _ = windy.wind_from_uv(u, v)
            assert 0 <= direction < 360
            assert isinstance(direction, int)


# ── wind_at_nearest_time — sélection du timestamp le plus proche ────────────

def _fixture_multi_entry():
    # 5 échéances toutes les 3h, vent qui tourne : facile à distinguer.
    base = 1_700_000_000_000  # epoch ms arbitraire
    hour = 3 * 3600 * 1000
    return {
        "ts": [base + i * hour for i in range(5)],
        "wind_u-surface": [0, -5, 5, 0, -5],
        "wind_v-surface": [-5, 0, 0, 5, -5],
        "model": "gfs",
    }, base, hour


def test_nearest_time_picks_closest_entry():
    forecast, base, hour = _fixture_multi_entry()
    # Cible tout près de l'échéance d'index 2 (base + 2h) -> plus proche de
    # l'entrée index 2 (base + 2*hour) que de l'index 1 ou 3.
    target = base + 2 * hour + 100  # 100 ms après l'échéance 2
    result = windy.wind_at_nearest_time(forecast, target_ts_ms=target)
    assert "error" not in result
    assert result["timestamp_ms"] == base + 2 * hour
    # index 2 -> u=5, v=0 -> vent DE l'ouest -> 270°
    assert result["direction_deg"] == 270
    assert result["model"] == "gfs"


def test_nearest_time_picks_earlier_entry_when_closer():
    forecast, base, hour = _fixture_multi_entry()
    # Juste avant le milieu entre l'échéance 0 et 1 -> plus proche de 0.
    target = base + (hour // 2) - 1000
    result = windy.wind_at_nearest_time(forecast, target_ts_ms=target)
    assert result["timestamp_ms"] == base
    # index 0 -> u=0, v=-5 -> vent DU nord -> 0°
    assert result["direction_deg"] == 0


def test_nearest_time_skips_none_entries():
    forecast, base, hour = _fixture_multi_entry()
    # Corrompt l'entrée la plus proche de la cible avec des None : doit
    # sauter à la suivante exploitable plutôt que planter ou renvoyer NaN.
    forecast["wind_u-surface"][2] = None
    target = base + 2 * hour
    result = windy.wind_at_nearest_time(forecast, target_ts_ms=target)
    assert "error" not in result
    assert result["timestamp_ms"] != base + 2 * hour


# ── wind_at_nearest_time — réponses malformées : erreur propre, pas d'exception ──

def test_missing_ts_key_returns_clean_error():
    result = windy.wind_at_nearest_time({"wind_u-surface": [1], "wind_v-surface": [1]})
    assert result["error_type"] == "malformed_response"
    assert "error" in result


def test_empty_arrays_return_clean_error():
    result = windy.wind_at_nearest_time({"ts": [], "wind_u-surface": [], "wind_v-surface": []})
    assert result["error_type"] == "malformed_response"


def test_missing_wind_components_return_clean_error():
    result = windy.wind_at_nearest_time({"ts": [1700000000000]})
    assert result["error_type"] == "malformed_response"


def test_mismatched_array_lengths_return_clean_error():
    result = windy.wind_at_nearest_time({
        "ts": [1, 2, 3],
        "wind_u-surface": [1, 2],
        "wind_v-surface": [1, 2, 3],
    })
    assert result["error_type"] == "malformed_response"


def test_all_none_entries_return_clean_error():
    result = windy.wind_at_nearest_time({
        "ts": [1, 2],
        "wind_u-surface": [None, None],
        "wind_v-surface": [None, None],
    })
    assert result["error_type"] == "malformed_response"


def test_non_dict_forecast_returns_clean_error_not_exception():
    result = windy.wind_at_nearest_time(["not", "a", "dict"])
    assert result["error_type"] == "malformed_response"


# ── validate_lat_lon — bornes géographiques ─────────────────────────────────

def test_validate_lat_lon_accepts_valid_point():
    lat, lon = windy.validate_lat_lon(48.36, -4.57)
    assert lat == pytest.approx(48.36)
    assert lon == pytest.approx(-4.57)


@pytest.mark.parametrize("lat,lon", [
    (91, 0),        # latitude > 90
    (-91, 0),       # latitude < -90
    (0, 181),       # longitude > 180
    (0, -181),      # longitude < -180
])
def test_validate_lat_lon_rejects_out_of_range(lat, lon):
    with pytest.raises(windy.WindyValidationError):
        windy.validate_lat_lon(lat, lon)


def test_validate_lat_lon_rejects_non_numeric():
    with pytest.raises(windy.WindyValidationError):
        windy.validate_lat_lon("nope", 0)


def test_validate_lat_lon_rejects_nan():
    with pytest.raises(windy.WindyValidationError):
        windy.validate_lat_lon(float("nan"), 0)


def test_fetch_point_forecast_rejects_out_of_range_before_network():
    # Ne doit même pas essayer d'appeler le fetcher (réseau) si lat/lon est
    # invalide : le fetcher lève si jamais appelé, pour le prouver.
    def _boom(*a, **k):
        raise AssertionError("fetcher ne doit pas être appelé pour un lat invalide")

    with pytest.raises(windy.WindyValidationError):
        windy.fetch_point_forecast(999, 0, api_key="fake-key", fetcher=_boom)


# ── load_windy_api_key — jamais de fuite de la clé ──────────────────────────

def test_load_key_from_environ(monkeypatch):
    monkeypatch.setenv("WINDY_API_KEY", "test-key-from-environ")
    assert windy.load_windy_api_key() == "test-key-from-environ"


def test_load_key_falls_back_to_env_file(monkeypatch, tmp_path):
    monkeypatch.delenv("WINDY_API_KEY", raising=False)
    env_file = tmp_path / ".env"
    env_file.write_text("SOME_OTHER=1\nWINDY_API_KEY=test-key-from-file\n")
    assert windy.load_windy_api_key(env_path=str(env_file)) == "test-key-from-file"


def test_load_key_missing_raises_clear_error_without_leaking(monkeypatch, tmp_path):
    monkeypatch.delenv("WINDY_API_KEY", raising=False)
    empty_env = tmp_path / ".env"
    empty_env.write_text("SOME_OTHER=1\n")

    with pytest.raises(windy.WindyConfigError) as excinfo:
        windy.load_windy_api_key(env_path=str(empty_env))

    message = str(excinfo.value)
    assert "WINDY_API_KEY" in message  # nomme la variable manquante...
    assert "manquante" in message.lower()
    # ...mais ne peut structurellement pas contenir de valeur : elle est absente.


def test_load_key_missing_env_file_does_not_raise_ioerror(monkeypatch, tmp_path):
    monkeypatch.delenv("WINDY_API_KEY", raising=False)
    nonexistent = tmp_path / "does_not_exist" / ".env"
    with pytest.raises(windy.WindyConfigError):
        windy.load_windy_api_key(env_path=str(nonexistent))


def test_network_error_does_not_leak_api_key(monkeypatch):
    # Régression directe de la consigne "ne jamais mettre la clé dans un
    # message d'erreur" : on force une vraie clé secrète-factice, on fait
    # échouer le fetcher, et on vérifie qu'elle n'apparaît nulle part dans
    # l'exception levée.
    secret = "sk-super-secret-TEST-VALUE-12345"

    def _raise_urlerror(*a, **k):
        raise urllib.error.URLError("Temporary failure in name resolution")

    with pytest.raises(windy.WindyNetworkError) as excinfo:
        windy.fetch_point_forecast(48.36, -4.57, api_key=secret, fetcher=_raise_urlerror)

    assert secret not in str(excinfo.value)


def test_missing_key_path_does_not_raise_unhandled_and_does_not_leak(monkeypatch, tmp_path):
    monkeypatch.delenv("WINDY_API_KEY", raising=False)
    empty_env = tmp_path / ".env"
    empty_env.write_text("")

    def _boom(*a, **k):
        raise AssertionError("fetcher ne doit pas être appelé sans clé API")

    # api_key=None force load_windy_api_key() en interne (via fetch_point_forecast) ;
    # on redirige son repli .env vers le faux fichier vide du test (jamais le
    # vrai IHM/.env) en monkeypatchant la fonction de chemin par défaut.
    monkeypatch.setattr(windy, "_default_env_path", lambda: str(empty_env))

    # get_wind_forecast() ne doit jamais lever : elle doit renvoyer un dict
    # d'erreur propre, y compris quand la cause est une clé absente.
    result = windy.get_wind_forecast(48.36, -4.57, fetcher=_boom, api_key=None)

    assert result["error_type"] == "WindyConfigError"
    assert "error" in result


# ── fetch_point_forecast — happy path avec fetcher injecté (fixture JSON) ───

def _canned_response_bytes():
    payload = {
        "ts": [1700000000000, 1700010800000],
        "wind_u-surface": [0, -5],
        "wind_v-surface": [-5, 0],
        "model": "gfs",
    }
    return json.dumps(payload).encode("utf-8")


def test_fetch_point_forecast_happy_path_with_fake_fetcher():
    captured = {}

    def _fake_fetcher(url, data, headers, timeout):
        captured["url"] = url
        captured["payload"] = json.loads(data)
        return _canned_response_bytes()

    result = windy.fetch_point_forecast(48.36, -4.57, api_key="fake-key", fetcher=_fake_fetcher)

    assert captured["url"] == windy.WINDY_ENDPOINT
    assert captured["payload"]["lat"] == pytest.approx(48.36)
    assert captured["payload"]["lon"] == pytest.approx(-4.57)
    assert captured["payload"]["key"] == "fake-key"
    assert captured["payload"]["parameters"] == ["wind"]
    assert captured["payload"]["levels"] == ["surface"]
    assert isinstance(result, dict)
    assert result["ts"] == [1700000000000, 1700010800000]


def test_fetch_point_forecast_malformed_json_raises_parse_error():
    def _fake_fetcher(*a, **k):
        return b"not valid json {{{"

    with pytest.raises(windy.WindyParseError):
        windy.fetch_point_forecast(48.36, -4.57, api_key="fake-key", fetcher=_fake_fetcher)


def test_fetch_point_forecast_non_dict_json_raises_parse_error():
    def _fake_fetcher(*a, **k):
        return b"[1, 2, 3]"

    with pytest.raises(windy.WindyParseError):
        windy.fetch_point_forecast(48.36, -4.57, api_key="fake-key", fetcher=_fake_fetcher)


def test_fetch_point_forecast_http_error_wrapped_cleanly():
    def _fake_fetcher(*a, **k):
        raise urllib.error.HTTPError(windy.WINDY_ENDPOINT, 403, "Forbidden", hdrs=None, fp=None)

    with pytest.raises(windy.WindyNetworkError):
        windy.fetch_point_forecast(48.36, -4.57, api_key="fake-key", fetcher=_fake_fetcher)


# ── get_wind_forecast — orchestration complète, offline ─────────────────────

def test_get_wind_forecast_end_to_end_offline():
    def _fake_fetcher(url, data, headers, timeout):
        return _canned_response_bytes()

    result = windy.get_wind_forecast(
        48.36, -4.57, api_key="fake-key", fetcher=_fake_fetcher, target_ts_ms=1700000000000
    )

    assert "error" not in result
    assert result["direction_deg"] == 0   # première entrée : u=0,v=-5 -> 0°
    assert result["speed_ms"] == pytest.approx(5.0)
    assert result["lat"] == pytest.approx(48.36)
    assert result["lon"] == pytest.approx(-4.57)


def test_get_wind_forecast_never_raises_on_network_failure():
    def _fake_fetcher(*a, **k):
        raise urllib.error.URLError("no network in this sandbox")

    result = windy.get_wind_forecast(48.36, -4.57, api_key="fake-key", fetcher=_fake_fetcher)
    assert result["error_type"] == "WindyNetworkError"
    assert "error" in result


# ============================================================
# Endpoint GET /api/wind-forecast (routes/messages.py) — via la fixture
# `messages` de conftest.py (module réel chargé sous les fakes FastAPI/
# pydantic/pymongo). On monkeypatche `windy.get_wind_forecast` (le même
# objet module que `messages.windy`, puisque les deux fichiers l'importent
# depuis le même sys.path) pour rester 100% hors-ligne et isoler le
# comportement de ROUTAGE/CODES HTTP de messages.py de celui, déjà testé
# plus haut, de windy.py lui-même.
#
# Ces tests sont RÉELS (pas vacueux) : ils exercent le vrai corps de
# get_wind_forecast() dans routes/messages.py (mapping d'erreurs, appel à
# validate_coordinates, non-appel à push_command_to_boat), pas une
# contrainte pydantic — conforme à la mise en garde de conftest.py.
# ============================================================

def test_windy_module_is_shared_between_test_and_messages(messages):
    # Prérequis des tests suivants : monkeypatcher `windy.xxx` doit affecter
    # ce que `messages.get_wind_forecast` appelle.
    assert messages.windy is windy


def test_endpoint_rejects_invalid_coordinates_without_calling_windy(monkeypatch, messages):
    def _boom(*a, **k):
        raise AssertionError("windy.get_wind_forecast ne doit pas être appelée : lat invalide")

    monkeypatch.setattr(windy, "get_wind_forecast", _boom)

    resp = messages.get_wind_forecast(999.0, -4.57)
    assert resp.status_code == 400
    assert resp.content["status"] == "error"


def test_endpoint_success_returns_forecast_and_never_pushes_to_boat(monkeypatch, messages):
    calls = []
    monkeypatch.setattr(messages, "push_command_to_boat", lambda msg: calls.append(msg))
    monkeypatch.setattr(
        windy, "get_wind_forecast",
        lambda lat, lon, model="gfs", target_ts_ms=None, **kw: {
            "direction_deg": 270, "speed_ms": 5.0,
            "timestamp_ms": 1700000000000, "model": "gfs",
            "lat": lat, "lon": lon,
        },
    )

    resp = messages.get_wind_forecast(48.36, -4.57)

    assert resp.status_code == 200
    assert resp.content["status"] == "ok"
    assert resp.content["forecast"]["direction_deg"] == 270
    # LECTURE SEULE : ne doit jamais avoir poussé de commande au bateau.
    assert calls == []


@pytest.mark.parametrize("error_type,expected_status", [
    ("WindyConfigError", 500),
    ("WindyNetworkError", 502),
    ("WindyParseError", 502),
    ("WindyValidationError", 400),
    ("SomeUnknownFutureError", 502),  # repli raisonnable sur un type inconnu
])
def test_endpoint_maps_windy_error_types_to_http_status(monkeypatch, messages, error_type, expected_status):
    monkeypatch.setattr(
        windy, "get_wind_forecast",
        lambda *a, **k: {"error": "message d'erreur windy.py", "error_type": error_type},
    )

    resp = messages.get_wind_forecast(48.36, -4.57)

    assert resp.status_code == expected_status
    assert resp.content["status"] == "error"


def test_endpoint_never_leaks_key_on_unexpected_internal_exception(monkeypatch, messages):
    secret = "sk-super-secret-TEST-VALUE-endpoint"

    def _raise_with_secret(*a, **k):
        # Simule un bug interne qui embarquerait la clé dans l'exception —
        # l'endpoint doit quand même ne JAMAIS la renvoyer au client.
        raise RuntimeError(f"unexpected failure, key was {secret}")

    monkeypatch.setattr(windy, "get_wind_forecast", _raise_with_secret)

    resp = messages.get_wind_forecast(48.36, -4.57)

    assert resp.status_code == 500
    assert secret not in json.dumps(resp.content)
