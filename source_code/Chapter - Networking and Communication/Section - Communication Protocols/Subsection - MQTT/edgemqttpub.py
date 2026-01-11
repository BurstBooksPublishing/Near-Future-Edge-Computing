#!/usr/bin/env python3
# Production-ready MQTT publisher pattern for edge devices.
import ssl, time, json, sqlite3, logging, socket
import paho.mqtt.client as mqtt
from pathlib import Path
from random import random

BROKER = "gateway.local"
PORT = 8883
CLIENT_ID = "edge-sensor-001"
TOPIC = "factory/machine/telemetry"
CA_CERT = "/etc/ssl/certs/ca.pem"
CLIENT_CERT = "/etc/ssl/certs/client.crt"
CLIENT_KEY = "/etc/ssl/private/client.key"
DB_PATH = "/var/lib/edge_mqtt/outbox.db"

# local persistent queue
def init_db():
    conn = sqlite3.connect(DB_PATH, timeout=1.0)
    conn.execute("CREATE TABLE IF NOT EXISTS outbox(id INTEGER PRIMARY KEY, topic TEXT, payload BLOB, qos INTEGER)")
    conn.commit()
    return conn

conn = init_db()
log = logging.getLogger("edge_mqtt")
logging.basicConfig(level=logging.INFO)

def save_message(topic, payload, qos=1):
    conn.execute("INSERT INTO outbox(topic,payload,qos) VALUES (?,?,?)", (topic, payload, qos))
    conn.commit()

def flush_outbox(client):
    # send saved messages FIFO
    cur = conn.execute("SELECT id,topic,payload,qos FROM outbox ORDER BY id LIMIT 100")
    rows = cur.fetchall()
    for _id, topic, payload, qos in rows:
        rc = client.publish(topic, payload, qos=qos).rc
        if rc == mqtt.MQTT_ERR_SUCCESS:
            conn.execute("DELETE FROM outbox WHERE id=?", (_id,))
            conn.commit()
        else:
            break

def create_client():
    client = mqtt.Client(client_id=CLIENT_ID, clean_session=False)  # persistent session
    client.tls_set(ca_certs=CA_CERT, certfile=CLIENT_CERT, keyfile=CLIENT_KEY,
                   tls_version=ssl.PROTOCOL_TLSv1_2)
    client.tls_insecure_set(False)
    client.max_inflight_messages_set(20)
    client.on_connect = lambda c, u, f, rc: log.info("connected rc=%s", rc)
    client.on_disconnect = lambda c, u, rc: log.warning("disconnected rc=%s", rc)
    return client

client = create_client()

def connect_with_backoff(client, max_backoff=60):
    backoff = 1.0
    while True:
        try:
            client.connect(BROKER, PORT, keepalive=60)
            client.loop_start()
            # wait for socket to be writable, ensures TLS handshake complete
            time.sleep(0.2)
            flush_outbox(client)
            return
        except (socket.error, ssl.SSLError) as e:
            log.exception("connect failed")
            time.sleep(backoff + random()*0.2)
            backoff = min(max_backoff, backoff*2)

connect_with_backoff(client)

# publish loop (replace with real sensor read)
try:
    while True:
        payload = json.dumps({"ts": time.time(), "v": 42.0})
        info = client.publish(TOPIC, payload, qos=1)  # QoS 1 for assured delivery
        if info.rc != mqtt.MQTT_ERR_SUCCESS:
            save_message(TOPIC, payload, qos=1)  # persist for later
        time.sleep(0.5)
except KeyboardInterrupt:
    client.loop_stop()
    client.disconnect()