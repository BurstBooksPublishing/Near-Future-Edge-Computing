#!/usr/bin/env python3
"""
Edge privacy agent: pseudonymize fields, encrypt storage, publish aggregates,
and enforce retention. Integrate with TPM or HSM for key management in production.
"""
import json, sqlite3, time, os, logging
from datetime import datetime, timedelta
import hashlib, base64
import paho.mqtt.client as mqtt
from cryptography.fernet import Fernet

# Configuration (move to secure config store)
DB_PATH = "/var/lib/edge/telemetry.db"
MQTT_BROKER = "broker.example.com"
MQTT_TOPIC = "edge/aggregates"
RETENTION_DAYS = 30

logging.basicConfig(level=logging.INFO)
# Production: fetch key from TPM/HSM; here derive from a file-protected secret.
def load_encryption_key():
    # Placeholder: replace with TPM-backed key retrieval
    if os.path.exists("/etc/edge/enc_key"):
        return open("/etc/edge/enc_key","rb").read().strip()
    # generate and store securely for demo
    key = Fernet.generate_key()
    with open("/etc/edge/enc_key","wb") as f:
        os.chmod("/etc/edge/enc_key", 0o600)
        f.write(key)
    return key

FERNET = Fernet(load_encryption_key())

def pseudonymize(value: str) -> str:
    # stable pseudonym using HMAC-like hash; do not use raw identifiers in logs.
    h = hashlib.sha256()
    h.update(value.encode("utf-8"))
    return base64.urlsafe_b64encode(h.digest()[:16]).decode("ascii")

def publish_aggregate(aggregate: dict):
    client = mqtt.Client()
    client.tls_set()  # enforce TLS
    client.connect(MQTT_BROKER, 8883, 60)
    payload = json.dumps(aggregate).encode("utf-8")
    client.publish(MQTT_TOPIC, payload, qos=1)
    client.disconnect()

def store_record(record: dict):
    # pseudonymize PII fields
    if "device_id" in record:
        record["device_id"] = pseudonymize(record["device_id"])
    blob = FERNET.encrypt(json.dumps(record).encode("utf-8"))
    with sqlite3.connect(DB_PATH) as conn:
        conn.execute("CREATE TABLE IF NOT EXISTS telemetry (ts INTEGER, data BLOB)")
        conn.execute("INSERT INTO telemetry VALUES (?, ?)", (int(time.time()), blob))
        conn.commit()

def enforce_retention():
    cutoff = int((datetime.utcnow() - timedelta(days=RETENTION_DAYS)).timestamp())
    with sqlite3.connect(DB_PATH) as conn:
        conn.execute("DELETE FROM telemetry WHERE ts < ?", (cutoff,))
        conn.commit()

# Example runtime loop
def main_loop():
    while True:
        # In production, replace with real data ingestion
        sample = {"device_id":"sensor-42","vibration":0.012,"ts":time.time()}
        store_record(sample)
        # publish only non-identifying aggregate
        publish_aggregate({"vibration_mean":0.012, "window":"60s"})
        enforce_retention()
        time.sleep(60)

if __name__ == "__main__":
    main_loop()