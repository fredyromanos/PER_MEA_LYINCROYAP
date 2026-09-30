import json
from datetime import datetime
from unittest import result
from pymongo.mongo_client import MongoClient
from pymongo import MongoClient, ReturnDocument
from config import MONGO_URI, DB_NAME


def ensure_indexes(collection):
    """
    Crée (de façon idempotente — create_index ne duplique pas un index déjà
    présent avec les mêmes clés) les index nécessaires aux requêtes chaudes :
    dispatch de commandes (`status`) et polling de télémétrie (`origin` +
    `timestamp`). Sans ça, ces requêtes dégradent en scan complet sur une
    mission longue.

    Ne doit jamais faire planter l'appli : Mongo absent/indisponible est
    l'état normal de cet environnement de dev, donc on log et on continue.
    """
    try:
        collection.create_index("status")
        collection.create_index([("origin", 1), ("timestamp", 1)])
    except Exception as e:
        print(f"⚠️ Impossible de créer les index MongoDB (Mongo indisponible ?) : {e}")


def sync_client():

    print("DEBUG URI utilisée :", MONGO_URI)
    client = MongoClient(MONGO_URI)
    db = client[DB_NAME]
    # Ensure collections exist
    collection = db["messages"]
    ensure_indexes(collection)
    return collection

def async_client():
    client = MongoClient(MONGO_URI)
    db = client[DB_NAME]
    # Ensure collections exist
    collection = db["messages"]
    ensure_indexes(collection)
    return collection


def push(origin, destination, data, status, collection):
    timestamp = datetime.now()#.isoformat()

    type = "raw"
    if isinstance(data, (dict, list)):
        type = "json"
    else:
        try:
            data = json.loads(data)
            type = "json"
        except json.JSONDecodeError:
            # If not valid JSON, keep the original raw payload.
            pass

    message = {
        "timestamp": timestamp,
        "origin": origin,
        "destination": destination,
        "data": data,
        "type": type,
        "status": status
    }
    print(message)

    result = collection.insert_one(message)
    print("DB:", collection.database.name)
    print("Collection:", collection.name)
    print("Client:", collection.database.client)


def get_pending_message(collection):
    # Tri FIFO par heure d'insertion : sans lui, l'ordre de dispatch entre
    # deux commandes en attente simultanément (ex. Stop puis Navigate dans
    # la fenêtre de ~1,05 s d'une rafale d'envoi) est indéfini — pour un
    # bateau, faire perdre à "Stop" une course contre "Navigate" est le
    # mauvais résultat.
    message = collection.find_one_and_update(
        {"status": "pending"},
        {"$set": {"status": "sent"}},
        sort=[("timestamp", 1)],
        return_document=ReturnDocument.BEFORE
    )
    return message
