"""Locations of the project files under test. Override the root with S9P_ROOT."""
import os
from pathlib import Path

# host_test -> gps_diag -> PER_MEA_LYINCROYAP -> 03_Developpement_logiciel -> S9P-2026
ROOT = Path(os.environ.get("S9P_ROOT", Path(__file__).resolve().parents[4]))
DEV = ROOT / "03_Developpement_logiciel"
PER = DEV / "PER_MEA_LYINCROYAP"
INFO = DEV / "Informatique"
VN1 = INFO / "AutoBoat_VN-1" / "Arduino"
FIELD_LOG = INFO / "AutoBoat_VN-1" / "Python" / "all_messages.log"
ARCHIVE = ROOT / "07_Passation_et_archives" / "PASSATION" / "RESSOURCES_INFORMATIQUE" / "Carte-Arduino" / "Arduino.zip"
MEETINGS = ROOT / "01_Pilotage_projet" / "Réunions"
TRIALS = ROOT / "04_Integration_et_essais" / "Essais"
