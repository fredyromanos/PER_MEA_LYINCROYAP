#!/usr/bin/env python3
"""Documents (#10-#13) and cross-component consistency checks (R2, R4, R5).
R2 runs the REAL code of each link: IHM JavaScript (node), IHM Python route builder, firmware parser (g++)."""
import ast
import json
import re
import shutil
import subprocess
import tempfile
import textwrap
import unittest
import zipfile
from pathlib import Path
from typing import List

from paths import ARCHIVE, DEV, MEETINGS, PER, TRIALS, VN1

GPS_WORDS = re.compile(r"gps|satellit|antenne|nmea|positionnement", re.I)


def docx_text(path):
    with zipfile.ZipFile(path) as z:
        xml = z.read("word/document.xml").decode("utf-8")
    return re.sub(r"<[^>]+>", "", xml.replace("</w:p>", "\n"))


def xlsx_text(path):
    with zipfile.ZipFile(path) as z:
        xml = z.read("xl/sharedStrings.xml").decode("utf-8")
    return re.sub(r"<[^>]+>", "", xml.replace("</si>", "\n"))


class Documents(unittest.TestCase):
    def test_T10_broken_gps_receiver_2026(self):
        """T10 meeting minutes 25/03-29/04/2026 record a broken GPS receiver, replacement arrived 29/04"""
        for name in ("CR_25-03-2026.docx", "CR_01-04-2026.docx", "08-04-2026.docx"):
            self.assertIn("Récepteur GPS cassé", docx_text(MEETINGS / name), name)
        self.assertRegex(docx_text(MEETINGS / "CR_29-04-2026.docx"), r"Récepteur GPS cassé\s*:\s*commande arrivée")

    def test_T10b_spec_gps_reliability_unknown(self):
        """T10b specification: GPS test status 'Test en cours / Inconnu' and antenna placement warning"""
        t = xlsx_text(DEV.parent / "05_Documentation_et_livrables" / "Cahier des charges" / "Cahier des charges.xlsx")
        self.assertIn("Système GPS pour se repérer", t)
        self.assertIn("Inconnu", t)
        self.assertIn("l'antenne GPS doit être positionnée sur le boitier", t)

    def test_T11_trial_retex_never_mention_gps(self):
        """T11 RETEX of the 2026 trials (27/05 car park, 10/06 lake) contain no GPS-related word"""
        for sub in ("Parking", "Lac de Saint-Renan"):
            text = docx_text(TRIALS / sub / "RETEX.docx")
            self.assertTrue(text.strip(), sub)
            self.assertIsNone(GPS_WORDS.search(text), f"{sub} RETEX mentions GPS")

    def test_T12_gps_doc_claims_contradicted(self):
        """T12 gps_explication.tex presents isValid() as 'fix active' and a FIX LOST branch (T05 proves both wrong)"""
        tex = (PER / "docs" / "gps_explication.tex").read_text(encoding="utf-8")
        self.assertIn("gps_.location.isValid(); // (4) fix GPS actif ?", tex)
        self.assertIn('DBG("GPS", "FIX LOST")', tex)
        self.assertRegex(tex, r"uniquement si une phrase NMEA valide avec fix actif")

    def test_T13_adversarial_same_conclusion(self):
        """T13 ADVERSARIAL_FINDINGS.md independently names GPS course as heading the most likely on-water failure"""
        md = (DEV / "docs" / "ADVERSARIAL_FINDINGS.md").read_text(encoding="utf-8")
        self.assertIn("GPS course-over-ground as heading", md)
        self.assertIn("single most likely on-water failure", md)


def extract_js_function(src, name):
    start = src.index(f"function {name}(")
    depth, i = 0, src.index("{", start)
    for j in range(i, len(src)):
        depth += {"{": 1, "}": -1}.get(src[j], 0)
        if depth == 0:
            return src[start:j + 1]
    raise ValueError(name)


class WaypointOrderEndToEnd(unittest.TestCase):
    """R2: a GeoJSON route goes IHM JS -> IHM Python -> LoRa string -> firmware parser without lat/lon swap."""

    GEOJSON_COORDS = [[-4.4861, 48.3904], [-4.5000, 48.3600]]     # GeoJSON order: [lon, lat]

    @unittest.skipUnless(shutil.which("node"), "node not installed")
    def test_R2a_js_geojson_to_waypoints(self):
        """R2a IHM script.js addRouteToMap(): GeoJSON [lon,lat] -> {lat, lon} (real function, run in node)"""
        js = (PER / "IHM" / "app" / "static" / "script.js").read_text(encoding="utf-8")
        fn = extract_js_function(js, "addRouteToMap")
        geo = {"features": [{"geometry": {"coordinates": self.GEOJSON_COORDS}}]}
        prog = fn + f"\nconsole.log(JSON.stringify(addRouteToMap({json.dumps(geo)})));"
        out = subprocess.run(["node", "-e", prog], capture_output=True, text=True, check=True).stdout
        wps = json.loads(out)
        self.assertEqual([(w["lat"], w["lon"]) for w in wps], [(48.3904, -4.4861), (48.36, -4.5)])

    def test_R2b_python_route_to_lora_string(self):
        """R2b IHM messages.py build_waypoints_message(): points string is 'lat,lon,...' (real function via ast)"""
        src = (PER / "IHM" / "app" / "routes" / "messages.py").read_text(encoding="utf-8")
        node = next(n for n in ast.parse(src).body
                    if isinstance(n, ast.FunctionDef) and n.name == "build_waypoints_message")

        class Waypoint:
            def __init__(self, lat, lon, radius_m=5.0):
                self.lat, self.lon, self.radius_m = lat, lon, radius_m

        ns = {"List": List, "Waypoint": Waypoint}
        exec(compile(ast.Module([node], []), "messages.py", "exec"), ns)
        msg = ns["build_waypoints_message"]([Waypoint(48.3904, -4.4861), Waypoint(48.36, -4.5)])
        self.assertEqual(msg["waypoints"]["points"], "48.39040000,-4.48610000,48.36000000,-4.50000000")

    @unittest.skipUnless(shutil.which("g++"), "g++ not installed")
    def test_R2c_firmware_parser(self):
        """R2c firmware LoRaComm.cpp waypoint loop (extracted verbatim, compiled) reads lat then lon"""
        src = (PER / "main" / "src" / "comm" / "LoRaComm.cpp").read_text(encoding="utf-8")
        loop = src[src.index("    for (int i = 0; i < count && plan.count"):src.index("    if (plan.count > 0)")]
        cpp = textwrap.dedent("""
            #include <cstdio>
            #include <cstdlib>
            #include <cstring>
            struct Waypoint { double lat, lon; float radiusM; };
            struct MissionPlan { static constexpr int MAX_WAYPOINTS = 8; Waypoint waypoints[8]; unsigned count = 0; };
            int main() {
                char pts[] = "48.39040000,-4.48610000,48.36000000,-4.50000000";
                int count = 2;
                MissionPlan plan{};
                char* p = pts;
            """) + loop + textwrap.dedent("""
                for (unsigned i = 0; i < plan.count; ++i)
                    std::printf("%.4f %.4f\\n", plan.waypoints[i].lat, plan.waypoints[i].lon);
            }
            """)
        with tempfile.TemporaryDirectory() as d:
            (Path(d) / "p.cpp").write_text(cpp)
            subprocess.run(["g++", "-std=c++17", "-o", f"{d}/p", f"{d}/p.cpp"], check=True, capture_output=True)
            out = subprocess.run([f"{d}/p"], capture_output=True, text=True, check=True).stdout.split()
        self.assertEqual(out, ["48.3904", "-4.4861", "48.3600", "-4.5000"])


class RepoConsistency(unittest.TestCase):
    def test_R4_setup_gps_loop_terminates(self):
        """R4 ruled out: legacy setupGPS() while-loop decrements failCount (repo and archive boat.ino)"""
        with zipfile.ZipFile(ARCHIVE) as z:
            archived = z.read("Arduino/boat/boat.ino").decode("utf-8", "replace")
        for label, src in (("repo", (VN1 / "boat" / "boat.ino").read_text(encoding="utf-8", errors="replace")),
                           ("archive", archived)):
            m = re.search(r"while\s*\(\s*gpsBoat\.getStatus\(\)\s*!=\s*2\s*&&\s*failCount\s*>\s*0\s*\)\s*\{(.*?)\n\s*\}",
                          src, re.S)
            self.assertIsNotNone(m, label)
            self.assertIn("failCount--", m.group(1), label)

    def test_R5_archive_gps_code_identical(self):
        """R5 handover Arduino.zip GPS sources are identical to the analysed repo copies (ignoring CRLF)"""
        with zipfile.ZipFile(ARCHIVE) as z:
            for rel in ("GPS/Gps.cpp", "GPS/Gps.hpp", "heading/Gps.cpp", "heading/Gps.hpp",
                        "boat/Gps.cpp", "boat/Gps.hpp"):
                a = z.read(f"Arduino/{rel}").replace(b"\r\n", b"\n")
                b = (VN1 / rel).read_bytes().replace(b"\r\n", b"\n")
                self.assertEqual(a, b, rel)


if __name__ == "__main__":
    unittest.main(verbosity=2)
