# AutoBoat - Architecture de la simulation

Ce document explique comment la simulation fonctionne, ou modifier les scenarios, et comment relier la simulation a la navigation reelle.

## Vue d'ensemble

```text
simulation/sim_main.cpp
  -> cree les scenarios
  -> lance l'observation du vent
  -> lance la navigation
  -> exporte les resultats

simulation/sim_boat.cpp / .hpp
  -> represente le bateau simule
  -> appelle la logique de navigation de boat/navigation.h
  -> applique les angles de voile et de safran a l'environnement

boat/navigation.h
  -> contient la decision de navigation
  -> fichier partage avec le firmware reel

simulation/sim_environment.cpp / .hpp
  -> simule la physique
  -> calcule vitesse, cap et nouvelle position GPS

simulation/export/
  -> ecrit les fichiers CSV
  -> genere la page HTML interactive

simulation/mocks/
  -> remplace Arduino, LoRa et SPI pour compiler sur ordinateur
```

## Principe important

La simulation n'a pas sa propre logique de navigation. Elle inclut :

```cpp
#include "navigation.h"
```

dans `simulation/sim_boat.cpp`.

Comme le `Makefile` ajoute `../boat` aux dossiers d'inclusion, ce fichier est `boat/navigation.h`.

Cela permet de tester sur ordinateur le meme algorithme que celui utilise par `boat/boat.ino`.

## Execution d'un pas de simulation

Un pas de simulation correspond a :

```text
SimulatedBoat::stepSimulation(dt_ms)
  -> updateNavigationLogic()
  -> environment.update(dt_ms)
```

Les deux parties ont des roles differents.

### 1. `updateNavigationLogic()`

Cette methode lit l'etat actuel :

- position GPS ;
- cap ;
- waypoint courant ;
- vent connu ;
- angles actuels.

Puis elle appelle la navigation partagee :

- `nav_handleWindObservation()` si le mode est `wind-observation` ;
- `nav_handleNavigation()` si le mode est `navigate`.

La navigation renvoie un `NavResult`.

```cpp
struct NavResult {
    float sailAngle;
    float rudderAngle;
    int sendInterval;
    const char* mode;
    const char* logMessage;
    bool windAcquired;
    double acquiredWindDir;
    bool waypointReached;
};
```

La simulation applique ensuite les angles a l'environnement :

```cpp
environment.setServoAngles(clampedSail, clampedRudder);
```

### 2. `environment.update(dt_ms)`

Cette methode avance le monde physique.

Elle ne decide pas ou aller. Elle repond seulement a la question :

> Si la voile et le safran ont ces angles pendant `dt_ms`, ou sera le bateau apres ?

Le resultat est ajoute dans l'historique `history`, qui sert ensuite aux exports CSV et HTML.

## La physique simplifiee

Toute la physique est dans :

```cpp
SimulationEnvironment::updateBoatDynamics(...)
```

Elle suit cinq etapes.

### 1. Vent relatif

Le simulateur calcule d'abord l'angle du vent par rapport au bateau :

```cpp
relativeWind = windDirection - heading
```

Puis il normalise entre `-180` et `+180`.

### 2. Efficacite de la voile

La voile est modelisee comme une aile rigide libre en rotation.

L'aileron donne l'angle d'attaque :

```cpp
angleOfAttack = abs(aileronAngle)
```

Si le signe de l'aileron est coherent avec le vent relatif, la voile produit de la portance. Sinon, elle produit tres peu de propulsion.

### 3. Polaire du bateau

La polaire donne une vitesse theorique selon l'angle au vent.

| Angle au vent | Coefficient |
|---|---|
| 0 a 30 deg | presque nul |
| 30 a 60 deg | augmente progressivement |
| 60 a 110 deg | maximum |
| 110 a 150 deg | bon mais decroissant |
| 150 a 180 deg | plus faible |

La vitesse cible vaut :

```cpp
targetSpeed = maxSpeed * polarCoeff * sailEff * (windSpeed / 5.0)
```

Avec `maxSpeed = 2.5 m/s`.

### 4. Inertie

Le bateau ne passe pas instantanement a la vitesse cible.

```cpp
speed += (targetSpeed - speed) * 0.05
```

Cette formule donne une acceleration progressive.

### 5. Rotation et deplacement GPS

Le safran ne fonctionne bien que si le bateau avance :

```cpp
steeringEff = min(1.0, speed / 0.5)
turnRate = -rudderOffset * 0.5 * steeringEff
heading += turnRate * dt_s
```

Ensuite, la distance parcourue est :

```cpp
moveDistanceM = speed * dt_s
```

Le simulateur convertit cette distance en variation de latitude et longitude.

## Les scenarios

Les scenarios sont dans `simulation/sim_main.cpp`.

Chaque scenario suit le meme modele :

```cpp
SimTime::init();

SimulatedBoat boat;
boat.init(startLat, startLng, windDir, windSpeed, initialHeading);
boat.addWaypoint(wptLat, wptLng);
boat.startWindObservation();
boat.runSimulation(60000, 100);
boat.setWindDirection(windDir);
boat.startNavigation();
boat.runSimulation(duration, 100);
```

Signification des arguments de `boat.init(...)` :

| Argument | Signification |
|---|---|
| `startLat` | Latitude de depart. |
| `startLng` | Longitude de depart. |
| `windDir` | Direction d'ou vient le vent. |
| `windSpeed` | Vitesse du vent en m/s. |
| `initialHeading` | Cap initial du bateau. |

Scenarios disponibles :

| Numero | Nom | Objectif |
|---|---|---|
| 1 | Wind Obs | Tester seulement l'observation du vent. |
| 2 | VDB | Waypoint face au vent. |
| 3 | Lofer | Waypoint atteignable en remontant vers le vent. |
| 4 | Abattre | Waypoint atteignable en descendant du vent. |
| 5 | Complex | Parcours avec deux waypoints. |
| 6 | Simple | Cas simple de reaching. |

## Comment modifier un scenario

### Changer le vent

Dans `sim_main.cpp`, modifier :

```cpp
boat.init(48.340, -4.520, 180, 4.0, 90);
```

Ici :

- `180` est la direction du vent ;
- `4.0` est la vitesse du vent ;
- `90` est le cap initial.

Si le scenario force ensuite le vent avec `setWindDirection(...)`, modifier aussi cette ligne.

### Changer un waypoint

Modifier ou ajouter :

```cpp
boat.addWaypoint(48.3368, -4.520);
```

Pour plusieurs waypoints, appeler `addWaypoint(...)` plusieurs fois dans l'ordre voulu.

### Changer la duree

Modifier :

```cpp
boat.runSimulation(6400000, 100);
```

- premier argument : duree totale en millisecondes ;
- second argument : pas de temps en millisecondes.

`100` signifie que la navigation et la physique sont mises a jour toutes les `100 ms`.

## Comment ajouter un nouveau scenario

1. Copier une fonction existante, par exemple `runScenario6()`.
2. La renommer, par exemple `runScenario7()`.
3. Modifier depart, vent, cap, waypoints et duree.
4. Ajouter l'appel dans `main()` :

```cpp
if (scenario == 0 || scenario == 7) allScenarios.push_back(runScenario7());
```

5. Mettre a jour le message d'erreur qui liste les scenarios disponibles.
6. Mettre a jour `simulation/Makefile` si `run-all` doit lancer ce nouveau scenario.

## Compiler et lancer

Depuis le dossier `simulation` :

```bash
make
./boat_simulator 3
```

Pour lancer tous les scenarios :

```bash
make run-all
```

Les fichiers generes sont dans `simulation/output/` :

- CSV de trajectoire ;
- CSV des waypoints ;
- CSV des conditions initiales ;
- `simulation.html` pour visualiser les scenarios sur une carte.

## Export HTML

Le fichier `simulation/export/sim_html_export.cpp` genere une page HTML avec :

- carte Leaflet ;
- onglets de scenarios ;
- animation du bateau ;
- slider temporel ;
- affichage du mode de navigation ;
- affichage du vent ;
- affichage des waypoints.

Les donnees viennent de `history`, c'est-a-dire de tous les etats sauvegardes par `SimulationEnvironment`.

## Ou modifier quoi

| Objectif | Fichier | Zone |
|---|---|---|
| Changer la logique de navigation | `boat/navigation.h` | `nav_handleNavigation()` |
| Changer l'observation du vent | `boat/navigation.h` | `nav_handleWindObservation()` |
| Changer les seuils reels | `boat/config_pins.h` | `WAYPOINT_DISTANCE`, `WIND_DISTANCE` |
| Changer les seuils simulation | `simulation/sim_boat.hpp` | `WAYPOINT_DISTANCE_SIM`, `WIND_DISTANCE_SIM` |
| Changer les scenarios | `simulation/sim_main.cpp` | `runScenarioX()` |
| Changer la physique | `simulation/sim_environment.cpp` | `updateBoatDynamics()` |
| Changer la vitesse max | `simulation/sim_environment.cpp` | `maxSpeed` |
| Changer l'inertie | `simulation/sim_environment.cpp` | coefficient `0.05` |
| Changer l'export HTML | `simulation/export/sim_html_export.cpp` | fonctions `write...()` |

## Limites de la simulation

La simulation est volontairement simplifiee.

Elle ne modelise pas parfaitement :

- les vagues ;
- le courant ;
- les rafales ;
- le bruit GPS ;
- les delais reels des servos ;
- les erreurs de communication LoRa.

Elle sert surtout a comprendre et tester la logique de navigation avant un essai reel.
