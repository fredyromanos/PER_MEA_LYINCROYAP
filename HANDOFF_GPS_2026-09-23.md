# Handoff — Fiabilisation GPS / navigation (PER MEA S9P-2026)

Date : 2026-09-23 · Drone voilier autonome, LilyGO T-Beam V1.1 (ESP32 + NEO-6M, AXP192)
Racine projet : `/home/ArchH/Downloads/Project_Ecoresponsable/2026/S9P-2026/`

---

## 1. Point de départ

Constat terrain : « le projet dans son ensemble va bien, c'est à cause du GPS que ça ne
marche pas sur site ». Le diagnostic a confirmé que le problème est ancien et présent dans
**toutes** les versions du firmware (legacy comprises), jamais corrigé.

### Défauts prouvés (banc de test + log terrain 2024)

| # | Défaut | Preuve |
|---|--------|--------|
| D1 | Perte de fix jamais détectée : `location.isValid()` de TinyGPSPlus est *collant* | test hôte + 952 lignes `[0,0]` dans le log 2024 |
| D2 | Champs NMEA vides → les valeurs précédentes sont re-validées (cap et vitesse gelés) | test hôte sur trames RMC réelles |
| D3 | Cap = 0° (« faux nord ») au démarrage : `TinyGPSDecimal` laisse `newval` non initialisé (stockage statique ⇒ 0.0) | lecture du code de la lib + test |
| D4 | Récepteur laissé en modèle dynamique « portable » au lieu de « sea » | absence de CFG-NAV5 dans tout le dépôt |
| D5 | La navigation pilote sur ce cap faux ; l'estimation du vent amorce son EMA dessus et réinjecte 50×/s le même échantillon 1 Hz | `AutoController.cpp` avant correction |
| D6 | Rampe de position à moyenne nulle (79/79 fractions exactes), 183 lignes lat/lon inversées | analyse du log 2024 |

**Cause racine** : le bateau n'a **aucune source de cap** à basse vitesse (pas de compas,
cap = route-sur-le-fond GPS uniquement), et le driver sert des données périmées comme valides.

### Bug supplémentaire trouvé par l'utilisateur (offset du safran)

Trajectoire non cohérente : avec un **offset mécanique du safran**, le bateau part
indéfiniment tout droit. Reproduit et diagnostiqué :

- offset 5–19° → seuls les waypoints **plein vent arrière** échouent (route droite de 5–6 km,
  ratio de chemin 13–15, 0,3 virement)
- offset ≥ 20° → le bateau tourne en rond (85–93 virements)

Mécanisme : la phase 1 de l'anti-empannage exige une erreur de cap < 10°, mais le clamp de
correction à ±20° laisse une erreur permanente d'environ 24° en présence d'un biais ⇒ la phase
n'avance jamais, et rien ne surveille ce blocage. La largeur de couloir n'y est pour rien.

---

## 2. Ce qui a été fait

Décisions prises avec l'utilisateur : **pas de capteur supplémentaire** (option B) ; perte de
fix → neutre + reprise auto ; cap inconnu → poussée hélice en ligne droite ; 5 Hz reporté ;
couloir 30 m ; correction de l'offset par **trim auto + chien de garde**.

### Firmware — `main/src/`

| Fichier | Changement |
|---|---|
| `core/Types.h` | `GpsPosition` étendu : `speedValid`, `courseValid`, `courseSeq`, `satellites`, `hdop`, `ageMs` |
| `config/Calibration.h` | `GPS_FIX_MAX_AGE_MS=2000`, `GPS_MIN_SATS=5`, `GPS_MAX_HDOP=3.0`, `GPS_COURSE_MIN_SPEED_KMPH=1.0`, `HEADING_SPEED_ON_KMPH=1.5`, `HEADING_CONFIRM_SAMPLES=3`, `HEADING_SAMPLE_GAP_MS=1500`, `HEADING_HOLD_MS=2000`, `HEADING_ACQUIRE_TIMEOUT_MS=60000`, `WIND_OBS_MIN_SAMPLES=10`, `WIND_OBS_TIMEOUT_MS=180000` |
| `drivers/GpsUart.{h,cpp}` | Lecture des champs **bruts** RMC via `TinyGPSCustom` (vitesse champ 7, cap champ 8) ; validité = `isValid() && age<2 s && sats>=5 && hdop<=3` ; log `FIX LOST` ; `courseSeq++` uniquement sur un cap brut valide ; jamais de position `[0,0]` ; envoi UBX **CFG-NAV5** (`dynModel=5`, mer) avec checksum Fletcher calculé à l'exécution (`ubxChecksum()`) |
| `control/HeadingGate.{h,cpp}` | **NOUVEAU**, unité pure testable sur hôte : confirmation sur 3 échantillons, hystérésis de vitesse (1,5 / 1,0 km/h), maintien 2 s, invalidation immédiate si le fix tombe. `source` = none/gps/held |
| `control/AutoController.{h,cpp}` | `compute(windDeg, pos, target, nowMs)` ; `dtS` réel borné ; fix perdu → neutre ; cap absent → acquisition (safran centré, amure conservée, ESC croisière) + timeout ; propulsion ignore une vitesse périmée ; observation du vent sur nouveaux échantillons uniquement, amorcée sur le premier échantillon réel |
| `comm/Heartbeat.h` | **NOUVEAU** formateur pur ; champs `hv` (source de cap) et `fa` (âge du fix, s) ; pire cas 249 B ≤ 255 B LoRa |
| `comm/LoRaComm.{h,cpp}` | `sendHeartbeat(..., headingSrc, fixAgeS)`, refuse > 255 B |
| `app/DroneApp.cpp` | passe `nowMs`, gère `windObsFailed()`, ligne de debug enrichie |
| `navigation/navigation.h` | couloir par défaut 100 → **30 m** ; **trim auto** (`NAV_TRIM_GAIN_PER_S=0.02`, `NAV_TRIM_MAX_DEG=40`, fenêtre d'erreur 30°) appliqué **en dehors** du clamp ±20° ; **chien de garde** (`NAV_WATCHDOG_PHASE_TIMEOUT_S=120`, `NAV_WATCHDOG_NO_PROGRESS_S=300`, `NAV_WATCHDOG_PROGRESS_M=5`) ; `NavResult.headingTracked/headingErrorDeg` ; `NavState.rudderTrimDeg/watchedPhase/phaseTimeS/bestDistanceM/noProgressS` ; `nav_resetManoeuvre()` (conserve le trim) / `nav_resetTrim()` |

Compilation : `arduino-cli compile --fqbn esp32:esp32:t-beam main` → **OK, 354 280 octets (27 %)**.

### Simulation — `Simulation/simulation/`

- `sim_environment` : `distanceAndBearing()` statique **corrigée du cos(lat)**, identique au
  `Navigator` du firmware (avant : géométrie fausse) ; `setRudderBias()`, `setPropellerSpeed()`, `setSpeed()`
- `sim_boat` : modèle GPS réaliste (1 Hz, ±3 m, ±5°, perte de fix), passage par le vrai
  `HeadingGate`, acquisition, conversion de braquage `steerDeg = rudderAngle − nav_rudderCompensation(relWind)`
- `Makefile` : chemins `FW_SRC`/`-Imocks`, dépendances sur les en-têtes (corrige les objets
  périmés qui donnaient 0/6 scénarios)
- Mode `SIM_IDEAL_GPS=1` pour isoler la navigation du bruit GPS

### IHM — `IHM/`

- `AutoBoat_Simulation/simu_gps.cpp` (**jumeau numérique**), `predict_path.cpp` (prédiction de
  trajectoire côté serveur), `Makefile` — **NOUVEAUX**
- `app/routes/messages.py` : scénarios `classic`, `gps`, `gps-dropout`, `mission-square`,
  `mission-square-dropout` ; `/api/predict`, `/api/sim-info` ; `/start` idempotent (tue les
  anciens socat/sims) ; correction d'un **bug préexistant** : `terminate_external_pid` faisait
  un `killpg` qui tuait le serveur web lui-même
- `app/static/` : affichage `hv`/`fa`, bateau en `divIcon` SVG + centrage carte, trajectoire
  servie par le serveur, sélecteur de scénario, division par `simSpeedup` (la vitesse affichait
  50 km/h à ×10, maintenant 5,39 km/h), versions de cache (`script.js?v=18`, `map.js?v=8`)
- Départ simulé déplacé **sur l'eau** (48.340, −4.520), vérifié par couleur de tuile OSM
- `IHM/AutoBoat_Simulation/nav.js` : réplique dérivée de la navigation, **obsolète** (elle
  prédisait 878 m contre 232 m réels) — remplacée par `/api/predict`

---

## 3. Banc de test (indépendant du projet)

```bash
cd 03_Developpement_logiciel && ./run_all_tests.sh     # 6 couches
gps_diag/host_test/run.sh                              # banc GPS seul, exit 0 attendu
```

- `gps_diag/host_test/` : `test_tinygps.cpp`, `test_main_driver.cpp` (23 vérifications),
  `test_diag_util.cpp`, `test_field_log.py`, `test_analyze_log.py`, `test_docs_and_repo.py`,
  stubs Arduino ⇒ compile les **vraies** sources firmware sur hôte, ASan/UBSan
- `gps_diag/gps_diag_board/` : firmware de diagnostic terrain, `capture.sh`, `analyze_log.py`, `README.md`
- `PER_MEA_LYINCROYAP/test/test_gps_logic.cpp` : 39 vérifications (ctest, cible `test_gps_logic`)
- Tests de mutation utilisés pour prouver que le banc **détecte** les défauts réintroduits

---

## 4. KPI mesurés

**A — Couloir 100 m → 30 m** (mission carrée, 8 directions de vent)
| | avant | après |
|---|---|---|
| écart maximal | 103 m | **57 m** |
| écart moyen | 18,4 m | **8,3 m** |
| temps > 20 m hors route | 28,1 % | **15,1 %** |
| durée | — | +4 % |

**B — Trim auto + chien de garde** (balayage offset safran −30…+30 × 7 relèvements × 2 distances, GPS idéal ; la version « avant » est **le même fichier** avec gain de trim à 0 et chiens de garde désactivés, ce qui isole exactement la modification)

| offset | avant | après |
|---|---|---|
| \|offset\| ≤ 4° | 126/126 (100 %) | 126/126 (100 %) |
| 5–19° | 367/420 (87,4 %) | **420/420 (100 %)** |
| ≥ 20° | 30/308 (9,7 %) | **306/308 (99,4 %)** |
| **total** | **523/854** | **852/854** |
| ratio de chemin moyen ≥ 20° | 4,32 | **1,05** |
| virements moyens ≥ 20° | 1,5 | **0,1** |

Aucune régression à offset nul. Les 2 échecs résiduels restent à examiner.

---

## 5. Reste à faire

1. **Rebuild des binaires IHM** : `make -B` dans `IHM/AutoBoat_Simulation/` pour que
   `simu_gps`/`predict_path` embarquent la nouvelle `navigation.h`.
2. **Re-passer `run_all_tests.sh`** (les 6 couches) après la modification de `navigation.h`.
3. **Tests unitaires trim + chien de garde** à ajouter (`test_gps_logic.cpp` ou `test_nav.cpp`).
4. **Re-jouer** la campagne adverse et les KPI mission carrée avec trim/chien de garde pour un
   tableau KPI final.
5. Identifier les 2 cas encore en échec du balayage d'offset.
6. **Reflasher le transceiver** : il contient encore `gps_test`, il lui faut `transceiver.ino`.
   (Carte `01C00B54` = transceiver, cf. `CLAUDE.md` ; l'autre carte est celle du drone.)
7. **Reflasher le firmware du bateau** (`main`) — pas encore fait.
8. **Rien n'est commité** : le dépôt n'est pas sous git à la racine de travail.
9. **Essais extérieurs** avec `gps_diag_board` pour régler `GPS_COURSE_MIN_SPEED_KMPH`,
   `HEADING_*` et confirmer `dynModel=5`.
10. `IHM_desktop/nav.py` est **une autre réplique dérivée** de la navigation — à aligner ou supprimer.
11. `SIM_PROP_CRUISE_MS = 0.8` est une **hypothèse non mesurée** (vitesse hélice à 1700 µs).
12. Les copies **cassées** de `AXP202X_Library` (header-only ⇒ erreurs de link) sont toujours
    dans le dépôt ; utiliser l'upstream v1.1.3.

---

## 6. Pièges rencontrés (à ne pas refaire)

- `arduino-cli monitor` reste muet (DTR/RTS maintient l'ESP32 en reset) → utiliser
  `stty … -hupcl clocal; cat /dev/ttyACM0`.
- `stop_ihm.sh` fait un `pkill` sur motif : il peut tuer le shell appelant si la ligne de
  commande contient le motif.
- IHM : `.venv` à recréer (Python 3.12), MongoDB via `sudo docker compose` (l'utilisateur n'est
  pas dans le groupe docker).
- Toute modification d'en-tête de la simulation nécessite `make -B` si les dépendances ne sont
  pas à jour.

---

## 7. Livrables documentaires

- `PER_MEA_LYINCROYAP/RAPPORT_AVANCEMENT_2026-09-16.{md,tex,pdf}` — rapport d'avancement
- `gps_diag/README.md` — protocole de diagnostic terrain
- `03_Developpement_logiciel/TESTING.md` — description des 6 couches de test
