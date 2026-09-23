> # ⚠️ CECI NE REMPLACE PAS L'IHM WEB
>
> ## 🏭 Production / déploiement à grande échelle → **IHM web** (`../IHM/`)
> C'est la solution finale : plus robuste, persiste les données dans **MongoDB**
> et sépare bien les composants (serial_link, serveur, base de données, client
> web) en modules découplés qui peuvent tourner sur des machines différentes et
> monter en charge.
>
> ## 🧪 Cette app de bureau → **test et débogage rapides**
> Pratique quand on ne dispose que d'**un seul PC** et qu'il faut que ça
> **démarre tout de suite**, sans monter Docker, MongoDB ni serveur : un
> `./run.sh` et c'est parti. Ne conserve pas d'historique et ne gère pas
> plusieurs clients.
>
> # 👉 C'est UN OUTIL DE PLUS, pas la solution finale.

---

# IHM Desktop — AutoBoat

IHM de bureau en Python qui réplique l'IHM web (`../IHM/`) **sans navigateur,
sans serveur FastAPI, sans MongoDB et sans Docker**. Un seul processus parle
directement au transceiver via USB.

> L'IHM web d'origine reste intacte dans `../IHM/`. Ceci est une alternative
> indépendante.

## Lancer

```bash
./run.sh            # crée le venv la première fois et démarre l'app
```

ou manuellement :

```bash
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt
.venv/bin/python ihm_desktop.py
```

## Ce que ça fait (parité avec l'IHM web)

- **Carte OSM** (tkintermapview) centrée sur Brest, marqueur du bateau, trace
  parcourue, route planifiée, prédiction de trajectoire.
- **Télémétrie en direct** : mode, batterie (V), vitesse (km/h et nœuds), cap,
  servos (voile/safran), vent, coordonnées, waypoints total/actuel, heure du
  dernier message.
- **Panneau de diagnostic** (feux) : Liaison (fraîcheur + RSSI), GPS
  (fix/sat/hdop), Batterie (seuils 2S), Radio (RC), Contrôle (auto/radio).
  Alerte quand la liaison est perdue.
- **Boussole de vent** (la flèche pointe vers la source du vent).
- **Mesure du vent** : barre de progression visible tant que `wobs` arrive,
  avec un bouton d'annulation.
- **Waypoints** : clic sur la carte pour ajouter, liste latérale, suppression
  sélective, envoyer la route, effacer la route, importer un `.geojson`
  (LineString).
- **Commandes** : Naviguer, Stop, Mesure vent, Envoi vent, Redémarrer,
  Reconnecter/Reset transceiver (impulsion DTR/RTS).

## Port série

Le sélecteur de port remplace le bascule Sim/Réel du web :
- `auto` → auto-détection du transceiver par n° de série CP2104 (`01C00B54`).
- choisir un `/dev/ttyUSB*` précis.
- `/tmp/ttyV1` → pointer vers le simulateur (s'il tourne à part).

« Appliquer » reconnecte sur le port choisi.

## Notes

- La carte télécharge les tuiles d'OpenStreetMap ; **il faut internet**. Pour le
  terrain sans réseau, tkintermapview gère un cache SQLite de tuiles
  (pré-téléchargement de la zone) — non configuré par défaut.
- Les commandes sont envoyées en rafale 3× (anti-collision LoRa half-duplex),
  comme le `serial_link.py` d'origine.
- La logique de prédiction de trajectoire (`nav.py`) est un portage 1:1 de
  `navigation.h` / `nav.js`.
