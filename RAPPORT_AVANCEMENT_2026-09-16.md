# Rapport d'avancement — Estimation du vent et validation de la chaîne GPS

**Projet :** PER MEA S9P-2026, drone voilier autonome
**Période couverte :** du 9 au 16 septembre 2026
**Cartes concernées :** deux LilyGO T-Beam V1.1 (ESP32 + u-blox NEO-6M + AXP192),
l'une embarquée sur le drone, l'autre servant de station sol

---

## 1. Objet

Deux sujets ont occupé cette période. D'abord une mise au clair de la logique
d'estimation du vent et du rôle respectif des trois actionneurs, qui restait
source de confusion dans nos échanges. Ensuite la mise en service effective du
banc de test GPS autonome, spécifié la semaine dernière et compilé pour la
première fois sur la cible cette semaine.

L'objectif de fond est le même dans les deux cas : réduire le nombre de
variables inconnues avant les essais en mer. Tant que l'on ne sait pas si un
défaut de navigation vient du logiciel, du câblage ou de la réception
satellite, chaque essai coûte une journée pour un diagnostic incertain.

---

## 2. Travaux du 9 septembre

### 2.1 Relations angulaires et estimation du vent

La question de départ portait sur un cas concret : un vent relatif à -90°
(venant de bâbord) avec une aile réglée à -10°. L'écart de 80° entre les deux
avait été interprété comme un « angle côté moteur », ce qui laissait supposer
une relation fixe entre la voile et le safran.

La vérification a montré que le calcul était juste mais que l'interprétation
ne l'était pas. Il n'existe pas d'angle « côté moteur » : le système comporte
trois actionneurs indépendants, sans relation angulaire imposée entre eux.

| Actionneur | Fonction | Débattement | Piloté par |
|---|---|---|---|
| Aileron d'aile | Sélection binaire du bord | ±10° | Algorithme de navigation, référence vent |
| Treuil de safran | Contrôle de cap | ±90° manuel, ±110° auto | Algorithme de navigation, référence waypoint |
| Hélice (ESC) | Poussée avant/arrière | Bidirectionnel | Mode manuel uniquement, voie CH3 |

L'écart de 80° relevé correspond donc au comportement physique attendu :
l'aile s'auto-aligne à environ 45° de la coque et l'aileron ne fait qu'ajouter
±10° pour choisir le bord. Le constat a été consigné dans
`WIND_OBSERVATION_ANALYSIS.md`, avec les références de code correspondantes
(`navigation.h`, lignes 280-281 et 430-431).

### 2.2 Lissage de la mesure de cap

Le second point concernait le filtrage exponentiel appliqué pendant la phase
d'observation du vent, implémenté dans `AutoController.cpp` (lignes 99-144)
avec un coefficient de 0,1. Deux réserves avaient été formulées, toutes deux
levées après relecture du code :

- **Perte de précision sur la position.** La position n'est jamais lissée. Le
  filtre ne s'applique qu'au cap (`courseDeg`) ; le calcul de distance
  parcourue utilise les coordonnées brutes issues du GPS.
- **Dérive de l'étalonnage des actionneurs.** L'observation du vent ne touche
  à aucun paramètre d'étalonnage. Pendant la séquence, l'aile est figée à +10°,
  le safran centré à 1500 µs et l'ESC inactif. La seule sortie produite est
  l'angle de vent estimé.

La séquence retenue reste : commande LoRa, aile bloquée sur un bord, lissage du
cap à chaque point GPS, puis estimation du vent à 90° de la route suivie une
fois 30 m parcourus.

### 2.3 Spécification du banc de test GPS

Le banc `gps_test/` a été spécifié puis écrit dans la foulée. Le principe est
de reprendre exactement le même code d'alimentation (`AxpPower`) et le même
pilote GPS (`GpsUart`) que le firmware, sans aucune autre dépendance. La
conséquence est utile : si le banc obtient un point, la chaîne GPS du firmware
est saine ; s'il n'en obtient pas, le défaut est nécessairement matériel,
côté antenne ou côté conditions de réception.

Le banc regroupe sept fichiers (`gps_test.ino`, `GpsUart.h/.cpp`,
`AxpPower.h/.cpp`, `BoardConfig.h`, `DebugConfig.h`, `Types.h`). La procédure
de dépannage rédigée à cette occasion récapitule les défauts déjà rencontrés
sur cette carte : configuration sauvegardée désactivant les trames NMEA, qui
impose une remise aux valeurs d'usine par UBX CFG-CFG, et superviseur
d'antenne active inactif, corrigé par CFG-ANT avec `flags=0x001B`.

---

## 3. Travaux du 16 septembre

### 3.1 Remise en état de la chaîne de compilation

La première compilation sur cible a échoué faute d'environnement. Le poste ne
disposait que du paquet `arduino:avr` ; le support ESP32 et les bibliothèques
nécessaires étaient absents. Ont été installés le cœur `esp32:esp32` en
version 3.3.11 et la bibliothèque `TinyGPSPlus` 1.0.3.

Un défaut plus gênant est apparu ensuite, et mérite d'être signalé à l'équipe.
Les deux copies de la bibliothèque `AXP202X_Library` présentes dans le dépôt,
sous `AutoBoat/` et sous `AutoBoat_VN-1/`, sont **incomplètes** : elles ne
contiennent que l'en-tête `axp20x.h`, le fichier source `axp20x.cpp` est
absent. La compilation passe sans erreur et l'échec ne survient qu'à l'édition
de liens, sous la forme d'une dizaine de références non définies
(`AXP20X_Class::begin`, `setLDO3Voltage`, `setPowerOutPut`, `adc1Enable`,
entre autres). Le diagnostic est peu évident si l'on n'y est pas préparé. La
bibliothèque complète a été récupérée en version 1.1.3 pour débloquer la
compilation, mais **les copies défectueuses sont toujours dans le dépôt** et
feront perdre du temps au prochain qui compilera.

### 3.2 Compilation et téléversement

Une fois l'environnement complet, la compilation pour la cible
`esp32:esp32:t-beam` aboutit sans avertissement :

```
Sketch uses 299860 bytes (22%) of program storage space
Global variables use 24052 bytes (7%) of dynamic memory
```

Les marges sont confortables et ne poseront pas de contrainte lors de
l'intégration au firmware complet.

Le téléversement sur `/dev/ttyUSB0` s'est déroulé normalement, empreinte
vérifiée. À noter pour la suite : le pont USB-série de la carte est un CP2104
et le port n'apparaît que si le module noyau `cp210x` est chargé. L'accès
suppose par ailleurs l'appartenance au groupe `uucp`.

### 3.3 Lecture de la liaison série

La lecture du port par `arduino-cli monitor` ne renvoie rien sur cette carte :
l'outil manipule les lignes DTR et RTS, ce qui maintient l'ESP32 en réinitialisation.
Le contournement retenu consiste à configurer le port sans contrôle de modem
avant de le lire :

```bash
stty -F /dev/ttyUSB0 115200 raw -echo -hupcl clocal
cat /dev/ttyUSB0
```

Cette remarque vaut d'être retenue, le symptôme (aucune sortie, carte
apparemment muette) ressemblant à une panne alors qu'il n'en est rien.

### 3.4 Identification de la carte sous test

Un point d'organisation doit être consigné ici, car il conditionne la portée
des résultats. Le projet met en oeuvre deux T-Beam V1.1 : celle du drone et
celle qui sert de station sol, chargée du pont USB vers LoRa (`transceiver/`).
Les deux cartes étant strictement identiques du point de vue de la cible de
compilation, rien ne distingue l'une de l'autre à la connexion.

L'essai décrit ci-dessous a été mené sur **la carte de station sol**, et non
sur celle du drone. La vérification du programme présent sur la carte n'ayant
pas été faite avant le téléversement, le firmware `transceiver` qu'elle
portait a été écrasé par le banc `gps_test`. La perte est sans gravité, le
source étant sous gestion de version, mais **cette carte devra être reflashée
avec `transceiver.ino` avant le prochain essai de liaison LoRa**. Elle est
laissée en l'état pour le moment.

La conséquence sur l'interprétation est détaillée au 3.6.

### 3.5 Résultats de l'essai

L'essai a été conduit en intérieur, sur une durée cumulée d'environ deux
minutes, dont un relevé continu de 60 secondes.

Les éléments suivants sont validés sur la carte testée :

- **Alimentation.** Le rail LDO3 alimente le récepteur, qui répond, et aucune
  erreur d'initialisation de l'AXP192 n'a été remontée. Cette conclusion
  appelle toutefois une réserve, détaillée plus bas.
- **Liaison série.** Les trames NMEA arrivent en continu, le compteur de
  caractères progresse régulièrement et le compteur de sommes de contrôle
  erronées reste à zéro. Le câblage et la vitesse de transmission sont corrects.
- **Superviseur d'antenne.** La séquence attendue est observée :
  `ANTSTATUS=DONTKNOW`, puis `INIT`, puis `OK`.

En revanche, aucun satellite n'a été capté. Le nombre de satellites visibles
est resté à zéro pendant toute la durée du relevé, toutes les trames GSV sont
vides et les trames GGA restent sans solution. Aucun point n'a été obtenu.

Une réserve sur le volet alimentation : le relevé série a été lancé après le
début du démarrage et les premières lignes de la trace ont été perdues.
L'absence d'erreur AXP192 n'a donc pas été constatée directement, elle est
déduite du fait que le récepteur était alimenté. Comme le LDO3 peut être actif
par défaut à la mise sous tension, ce raisonnement n'est pas une preuve. La
trace complète du démarrage reste à capturer.

### 3.6 Interprétation

Ce qui est acquis : le code d'alimentation et le pilote GPS du firmware
compilent, se téléversent et s'exécutent correctement sur une T-Beam V1.1. Le
pilote `GpsUart` lit les trames sans erreur de somme de contrôle et la
configuration UBX envoyée au démarrage est bien prise en compte par le
récepteur. La chaîne logicielle est donc saine.

Ce qui ne l'est pas : l'essai ayant été mené sur la station sol, **il ne dit
rien de l'installation GPS du drone**, en particulier de son antenne active
déportée et de son câblage. Le raisonnement d'isolation prévu à la conception
du banc, où une chaîne NMEA propre écarte simultanément logiciel, câblage et
antenne, ne s'applique qu'à la carte effectivement testée.

Quant à l'absence de satellites, elle reste cohérente avec un essai en
intérieur et n'appelle pas d'autre explication à ce stade.

L'essai est donc à refaire sur la carte du drone, à l'extérieur et en vue
dégagée. La batterie LiPo devra être connectée : sans elle, le récepteur perd
ses éphémérides à chaque coupure et repart en démarrage à froid, ce qui ajoute
environ deux minutes à l'acquisition.

---

## 4. Suites à donner

| Action | Motif | Priorité |
|---|---|---|
| Refaire l'essai GPS sur la carte du drone, en extérieur, LiPo connectée | L'installation GPS du drone reste non validée | Haute |
| Reflasher `transceiver.ino` sur la station sol | Son firmware a été écrasé par le banc de test | Haute |
| Remplacer les copies incomplètes d'`AXP202X_Library` | Échec d'édition de liens au diagnostic non évident | Haute |
| Verser les travaux de septembre dans le dépôt | Dernier commit au 17 juin, trois mois hors gestion de version | Haute |
| Repérer physiquement les deux T-Beam | Rien ne les distingue une fois branchées | Moyenne |
| Exclure `IHM/.venv/` du suivi Git | L'état courant est noyé sous les fichiers d'environnement virtuel | Moyenne |
| Capturer la trace complète du démarrage | Confirmer l'initialisation de l'AXP192, aujourd'hui seulement déduite | Basse |

Le versement dans le dépôt est le point le plus exposé. Les développements de
septembre n'existent aujourd'hui que dans la copie de travail d'un seul poste,
sans sauvegarde ni historique. Le nettoyage préalable de `IHM/.venv/`
conditionne d'ailleurs la lisibilité du commit.

Le repérage des cartes n'est pas un point de confort. C'est la confusion entre
les deux qui a conduit à écraser le firmware de la station sol, et le même
incident se reproduira tant qu'un marquage ne permettra pas de les distinguer.

---

## 5. Conclusion

La logique d'estimation du vent est clarifiée et documentée, et les deux
réserves émises sur le lissage se sont révélées infondées après vérification
du code.

Le banc de test GPS est opérationnel au sens où il compile, se téléverse et
s'exécute sur cible, et il a permis de valider la chaîne logicielle
d'acquisition GPS. Son objectif premier n'est en revanche pas atteint :
l'essai ayant été mené sur la station sol et non sur le drone, l'installation
GPS de ce dernier reste à qualifier. Un essai en extérieur sur la bonne carte
suffira à clore le point, la procédure et l'environnement de compilation étant
désormais en place.
