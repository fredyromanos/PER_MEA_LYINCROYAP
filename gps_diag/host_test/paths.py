"""Locations of the project files under test. Override the root with S9P_ROOT."""
import os
from pathlib import Path

# Top-level folders that only exist at the root of a full S9P-2026-style project
# tree (meeting minutes, deliverables, trial reports, archives...).
_ROOT_MARKERS = ("01_Pilotage_projet", "03_Developpement_logiciel",
                 "04_Integration_et_essais", "05_Documentation_et_livrables")


def _looks_like_root(p: Path) -> bool:
    return p.is_dir() and all((p / m).is_dir() for m in _ROOT_MARKERS)


def _find_root() -> Path:
    override = os.environ.get("S9P_ROOT")
    if override:
        return Path(override)

    here = Path(__file__).resolve()
    # host_test -> gps_diag -> PER_MEA_LYINCROYAP -> 03_Developpement_logiciel -> S9P-2026
    # True when this checkout is nested inside a full S9P-2026 tree.
    legacy_default = here.parents[4]
    if _looks_like_root(legacy_default):
        return legacy_default

    # Otherwise this is a standalone checkout (e.g. a benchmark copy) that isn't
    # nested inside the full tree: the meeting minutes / archives / legacy
    # AutoBoat_VN-1 sibling tree instead lives next to it, under "2026/S9P-2026".
    # Search ancestors for that sibling before giving up.
    for ancestor in here.parents:
        candidate = ancestor / "2026" / "S9P-2026"
        if _looks_like_root(candidate):
            return candidate

    # Nothing found: fall back to the historical default so any error/skip
    # message at least points at a predictable, meaningful path.
    return legacy_default


ROOT = _find_root()
DEV = ROOT / "03_Developpement_logiciel"
PER = DEV / "PER_MEA_LYINCROYAP"
INFO = DEV / "Informatique"
VN1 = INFO / "AutoBoat_VN-1" / "Arduino"
FIELD_LOG = INFO / "AutoBoat_VN-1" / "Python" / "all_messages.log"
ARCHIVE = ROOT / "07_Passation_et_archives" / "PASSATION" / "RESSOURCES_INFORMATIQUE" / "Carte-Arduino" / "Arduino.zip"
MEETINGS = ROOT / "01_Pilotage_projet" / "Réunions"
TRIALS = ROOT / "04_Integration_et_essais" / "Essais"
