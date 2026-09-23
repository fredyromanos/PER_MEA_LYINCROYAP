# AutoBoat - Documentation de la navigation

Ce document explique la logique de navigation du bateau et la simulation. Il est ecrit pour etre lu par des etudiants qui decouvrent le projet.

## Idee generale

AutoBoat est un voilier autonome. Il avance grace au vent, sans moteur de propulsion.

Le code doit donc repondre a trois questions en boucle :

1. Ou est le bateau ?
2. Ou est le prochain waypoint ?
3. Comment regler la voile et le safran pour s'en rapprocher ?

La logique principale est dans `boat/navigation.h`. Ce fichier est utilise par :

- `boat/boat.ino` pour le bateau reel sur ESP32 ;
- `simulation/sim_boat.cpp` pour la simulation sur ordinateur.

Modifier `boat/navigation.h` modifie donc le comportement du bateau reel et de la simulation.

## Les grandeurs importantes

Les angles sont en degres.

| Grandeur | Signification |
|---|---|
| `boatHeading` | Cap actuel du bateau. `0` = nord, `90` = est, `180` = sud, `270` = ouest. |
| `wptHeading` | Direction a suivre pour aller vers le waypoint courant. |
| `wptDistance` | Distance entre le bateau et le waypoint courant. |
| `windDir` | Direction d'ou vient le vent. |
| `sailAngle` | Angle de l'aileron de voile : `-10` ou `+10`. |
| `rudderAngle` | Decalage du safran. Valeur limitee par la navigation a `-20` / `+20`. |

La fonction utilitaire la plus importante est :

```cpp
nav_relativeAngle(reference, target)
```

Elle renvoie l'angle de `target` par rapport a `reference`, entre `-180` et `+180`.

- valeur positive : la cible est a droite / tribord ;
- valeur negative : la cible est a gauche / babord.

Exemple : si le bateau pointe vers le nord (`0`) et que le waypoint est a l'est (`90`), `nav_relativeAngle(0, 90)` vaut `+90`.

## Cycle de vie du bateau

Le firmware reel utilise la variable `boatMode`.

```text
setup
  -> setup-ready
  -> route-ready
  -> wind-observation
  -> wind-ready
  -> navigate
  -> standby
```

En pratique :

| Mode | Role |
|---|---|
| `setup` | Demarrage du programme. |
| `setup-ready` | GPS, LoRa, servos et batterie initialises. |
| `route-ready` | Les waypoints ont ete recus. |
| `wind-observation` | Le bateau avance pour estimer la direction du vent. |
| `wind-ready` | La direction du vent est connue. |
| `navigate` | Le bateau suit les waypoints. |
| `standby` | Navigation terminee ou arretee. |

Dans la simulation, les modes sont equivalents, mais les waypoints sont crees directement dans `sim_main.cpp` au lieu d'etre recus par LoRa.

## Observation du vent

Le bateau n'a pas de capteur de vent. Il estime donc le vent par son mouvement.

Dans `nav_handleWindObservation()` :

1. Le bateau memorise sa position de depart.
2. Il avance avec ses angles de servo actuels.
3. Quand il a parcouru `WIND_DISTANCE` metres, le vent est considere acquis.
4. La direction du vent est estimee par :

```cpp
windDirection = smoothHeading + 90 deg
```

Le code normalise ensuite l'angle entre `0` et `360`.

Dans le projet actuel :

- `WIND_DISTANCE` vaut `30 m` dans `boat/config_pins.h` ;
- `WIND_DISTANCE_SIM` vaut aussi `30 m` dans `simulation/sim_boat.hpp`.

En simulation, plusieurs scenarios appellent ensuite `setWindDirection(...)`. Cela force le vent connu par la navigation a la valeur exacte du scenario. C'est utile car l'estimation `cap + 90` est simplifiee et peut etre imprecise.

## Navigation autonome

La fonction centrale est :

```cpp
nav_handleNavigation(...)
```

Elle ne deplace pas directement le bateau. Elle calcule seulement une decision :

- angle d'aileron ;
- angle de safran ;
- mode de navigation ;
- waypoint atteint ou non.

Le deplacement physique est ensuite fait par le firmware reel ou par `SimulationEnvironment` en simulation.

## Choix du cote de voile

Le bateau regarde d'abord le vent par rapport a son cap :

```cpp
windFromBoat = nav_relativeAngle(boatHeading, windDir)
```

Puis il place l'aileron du cote coherent :

```cpp
r.sailAngle = (windFromBoat >= 0) ? +10 : -10;
```

Interpretation :

- vent a tribord : `sailAngle = +10` ;
- vent a babord : `sailAngle = -10`.

Dans la simulation, si le signe est mauvais, la voile est consideree peu efficace et le bateau avance beaucoup moins.

## La logique oldNavigation

La version actuelle de `navigation.h` est volontairement identique a `oldNavigation.h`.

Elle utilise trois angles principaux :

```cpp
oppositeWind = nav_oppositeAngle(windDir);
relativeWind = nav_relativeAngle(boatHeading, windDir);
relativeWpt  = nav_relativeAngle(boatHeading, wptHeading);
```

Puis elle decide avec deux fonctions simples :

- `nav_isBetween(angle, start, end)` : verifie si une direction est entre le cap du bateau et le cap du waypoint ;
- `nav_sameSign(a, b)` : verifie si deux directions relatives sont du meme cote du bateau.

## Les modes de navigation

### 1. VDB

Le mode VDB est choisi si le vent ou l'oppose du vent se trouve entre le cap actuel du bateau et le cap vers le waypoint :

```cpp
if (nav_isBetween(oppositeWind, boatHeading, wptHeading) ||
    nav_isBetween(windDir, boatHeading, wptHeading)) {
    ...
}
```

Dans ce cas, le code met directement le safran en butee et place la voile selon le cote du vent :

```cpp
if (relativeWind < 0) {
    rudderAngle = 20;
    sailAngle = -10;
} else {
    rudderAngle = -20;
    sailAngle = 10;
}
```

Ce mode sert a changer de bord quand la route directe est problematique par rapport au vent.

### 2. Lofer

Si le bateau n'est pas en VDB, le code regarde si le vent et le waypoint sont du meme cote du bateau.

Condition :

```cpp
nav_sameSign(relativeWind, relativeWpt)
```

Si oui, le bateau lofe. Le safran n'est pas recalcule depuis zero : il est modifie progressivement par pas de `5 deg`.

```cpp
if (relativeWind < 0) {
    sailAngle = -10;
    rudderAngle = currentRudderAngle + 5;
} else {
    sailAngle = 10;
    rudderAngle = currentRudderAngle - 5;
}
```

### 3. Abattre

Si le bateau n'est pas en VDB et que le vent et le waypoint sont de cotes opposes, le bateau abat.

Comme pour lofer, le safran est modifie progressivement par pas de `5 deg`, mais dans le sens oppose :

```cpp
if (relativeWind < 0) {
    sailAngle = -10;
    rudderAngle = currentRudderAngle - 5;
} else {
    sailAngle = 10;
    rudderAngle = currentRudderAngle + 5;
}
```

### 4. Direct

Le resultat est initialise avec le mode `"direct"`, mais dans la pratique cette version du code remplace ensuite ce mode par `"vdb"`, `"lofer"` ou `"abattre"`.

Il n'y a pas de zone morte ni de correction proportionnelle dans cette version. C'est justement une difference importante avec l'autre algorithme.

## Atteinte d'un waypoint

La navigation signale qu'un waypoint est atteint si :

```cpp
wptDistance <= waypointDistance
```

Valeurs actuelles :

- reel : `WAYPOINT_DISTANCE = 10 m` ;
- simulation : `WAYPOINT_DISTANCE_SIM = 10 m`.

Quand un waypoint est atteint :

- s'il reste un waypoint, la simulation passe au suivant ;
- sinon elle passe en `standby`.

## Comment lire une decision de navigation

A chaque appel de `nav_handleNavigation()`, on peut resumer la decision ainsi :

```text
cap bateau + cap waypoint + vent
  -> calcul de oppositeWind, relativeWind, relativeWpt
  -> si windDir ou oppositeWind est entre cap bateau et cap waypoint : VDB
  -> sinon, si relativeWind et relativeWpt ont le meme signe : lofer
  -> sinon : abattre
  -> appliquer voile et safran
  -> tester si le waypoint est atteint
```

## Parametres a modifier

Dans cette version, il y a peu de constantes explicites dans `navigation.h`. Les valeurs importantes sont directement dans le code.

| Valeur | Ou | Effet |
|---|---|
| `20` / `-20` | branche VDB | Butee de safran pendant un changement de bord. |
| `10` / `-10` | toutes les branches | Angle de voile selon le cote du vent. |
| `+5` / `-5` | branches lofer et abattre | Increment progressif du safran a chaque appel. |
| `sendInterval = 300` | VDB | Frequence plus rapide pendant VDB. |
| `sendInterval = 2000` | lofer / abattre | Frequence normale de navigation. |

Les distances sont dans :

- `boat/config_pins.h` pour le bateau reel ;
- `simulation/sim_boat.hpp` pour la simulation.

## Conseils pour les etudiants

Pour changer le comportement de navigation, commencer par modifier un seul parametre a la fois.

Exemples :

- si le bateau tourne trop fort en VDB, diminuer les valeurs `20` / `-20` ;
- si le bateau corrige trop lentement en lofer ou abattre, augmenter le pas `5` ;
- si le bateau corrige trop brutalement en lofer ou abattre, diminuer le pas `5` ;
- si les waypoints sont valides trop tot ou trop tard, modifier `WAYPOINT_DISTANCE` et `WAYPOINT_DISTANCE_SIM`.
