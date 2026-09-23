from pymongo import MongoClient
from dotenv import load_dotenv
import os

load_dotenv()

MONGO_URI = os.getenv("MONGO_URI")
DB_NAME = os.getenv("DB_NAME")

print("MONGO_URI =", MONGO_URI)
print("DB_NAME =", DB_NAME)

try:
    client = MongoClient(
        MONGO_URI,
        serverSelectionTimeoutMS=5000
    )

    client.admin.command("ping")

    print("✅ Connexion MongoDB OK")

    db = client[DB_NAME]
    print("Collections disponibles :", db.list_collection_names())

except Exception as e:
    print("❌ Erreur MongoDB")
    print(type(e).__name__)
    print(e)
