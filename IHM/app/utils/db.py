import json
from datetime import datetime
from unittest import result
from pymongo.mongo_client import MongoClient
from pymongo import MongoClient, ReturnDocument
from config import MONGO_URI, DB_NAME


def sync_client():

    print("DEBUG URI utilisée :", MONGO_URI)
    client = MongoClient(MONGO_URI)
    db = client[DB_NAME]
    # Ensure collections exist
    collection = db["messages"]
    return collection

def async_client():
    client = MongoClient(MONGO_URI)
    db = client[DB_NAME]
    # Ensure collections exist
    collection = db["messages"]
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
    # Might add an order condition here
    message = collection.find_one_and_update(
        {"status": "pending"},
        {"$set": {"status": "sent"}},
        return_document=ReturnDocument.BEFORE
    )
    return message
