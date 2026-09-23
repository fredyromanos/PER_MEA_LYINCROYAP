"""
IHM ground-station protocol logic — the command-BUILDING half of the LoRa
contract that the firmware's LoRaComm.cpp parses on the other side.

Covers pure functions from routes/messages.py and serial_link.py:
  • build_waypoints_message  — flattened "lat,lon,..." string (8 dp) + count
  • parse_legacy_waypoints   — list & dict forms, <2-coord error
  • decode_message_data      — dict / json-str / embedded-json / garbage
  • push_command_to_boat     — 255-byte LoRa packet cap enforcement
  • extract_json_payload     — serial line brace-scan (serial_link)
"""
from types import SimpleNamespace


# ── build_waypoints_message ──────────────────────────────────────────────────
def test_build_waypoints_message_format(messages):
    wps = [SimpleNamespace(lat=48.3901, lon=-4.4860),
           SimpleNamespace(lat=48.3904, lon=-4.4856)]
    msg = messages.build_waypoints_message(wps)
    assert msg["waypoints"]["number"] == 2
    # firmware LoRaComm::handleWaypoints splits the "points" string on commas
    assert msg["waypoints"]["points"] == "48.39010000,-4.48600000,48.39040000,-4.48560000"


def test_build_waypoints_message_single(messages):
    msg = messages.build_waypoints_message([SimpleNamespace(lat=1.0, lon=2.0)])
    assert msg["waypoints"]["number"] == 1
    assert msg["waypoints"]["points"] == "1.00000000,2.00000000"


# ── parse_legacy_waypoints ───────────────────────────────────────────────────
def test_parse_legacy_list_form(messages):
    wps = messages.parse_legacy_waypoints("[[48.3901,-4.4860,5],[48.3904,-4.4856]]")
    assert len(wps) == 2
    assert wps[0].lat == 48.3901 and wps[0].lon == -4.4860
    assert wps[0].radius_m == 5.0
    assert wps[1].radius_m == 5.0  # default when omitted


def test_parse_legacy_dict_form(messages):
    wps = messages.parse_legacy_waypoints('[{"lat":48.39,"lon":-4.48,"radius_m":7}]')
    assert len(wps) == 1 and wps[0].radius_m == 7


def test_parse_legacy_too_few_coords_raises(messages):
    import pytest
    with pytest.raises(ValueError):
        messages.parse_legacy_waypoints("[[48.39]]")


def test_parse_then_build_roundtrip(messages):
    wps = messages.parse_legacy_waypoints("[[1.0,2.0],[3.0,4.0]]")
    msg = messages.build_waypoints_message(wps)
    assert msg["waypoints"]["points"] == "1.00000000,2.00000000,3.00000000,4.00000000"


# ── decode_message_data ──────────────────────────────────────────────────────
def test_decode_dict_passthrough(messages):
    assert messages.decode_message_data({"a": 1}) == {"a": 1}


def test_decode_json_string(messages):
    assert messages.decode_message_data('{"mode":"navigate"}') == {"mode": "navigate"}


def test_decode_embedded_json(messages):
    assert messages.decode_message_data('noise {"x":2} tail') == {"x": 2}


def test_decode_garbage_returns_empty(messages):
    assert messages.decode_message_data("not json at all") == {}
    assert messages.decode_message_data(12345) == {}


# ── push_command_to_boat — 255-byte LoRa cap ─────────────────────────────────
def test_push_command_within_cap_ok(messages):
    resp = messages.push_command_to_boat({"navigate": True})
    assert resp.status_code == 200


def test_push_command_over_cap_rejected(messages):
    huge = {"points": "x" * 300}  # forces the compact payload well over 255 bytes
    resp = messages.push_command_to_boat(huge)
    assert resp.status_code == 400
    assert resp.content["max_bytes"] == 255
    assert resp.content["actual_bytes"] > 255


# ── extract_json_payload (serial_link) ───────────────────────────────────────
def test_extract_json_clean(serial_link):
    assert serial_link.extract_json_payload('{"origin":"boat"}') == {"origin": "boat"}


def test_extract_json_with_prefix(serial_link):
    assert serial_link.extract_json_payload('[LORA] rx: {"a":1}') == {"a": 1}


def test_extract_json_none_when_absent(serial_link):
    assert serial_link.extract_json_payload("no braces here") is None


def test_extract_json_none_when_malformed(serial_link):
    assert serial_link.extract_json_payload("{broken json}") is None


# ── heartbeat with GPS heading validity (firmware comm/Heartbeat.h) ─────────
def test_heartbeat_heading_source_and_fix_age(serial_link):
    # Exact firmware format + the "rssi" the transceiver injects before "}}".
    line = ('{"origin":"boat","type":"info","message":{"mode":"navigate",'
            '"location":[48.36041,-4.56661],"servos":{"sail":10,"rudder":0},'
            '"heading":0,"hv":0,"wind":270,"bat":7.80,'
            '"fix":1,"fa":0,"sat":7,"hdop":1.2,"rc":1,"wt":3,"wc":0,"rssi":-102}}')
    msg = serial_link.extract_json_payload(line)["message"]
    assert msg["hv"] == 0          # no trustworthy heading: IHM must not show "0" as north
    assert msg["fa"] == 0
    assert msg["fix"] == 1 and msg["wt"] == 3 and msg["wc"] == 0   # existing fields unchanged
