#!/usr/bin/env python3
import json, sqlite3, time, threading
import paho.mqtt.client as mqtt

MQTT_BROKER = "broker.example.local"
MQTT_PORT = 8883
TOPIC_DELTA = "edge/device/{device_id}/delta"
TOPIC_RESUME = "edge/device/{device_id}/resume"
DEVICE_ID = "unit42"

# SQLite: table changes(seq INTEGER PRIMARY KEY, key TEXT, op TEXT, payload BLOB)
conn = sqlite3.connect("local.db", check_same_thread=False)
conn.execute("CREATE TABLE IF NOT EXISTS changes(seq INTEGER PRIMARY KEY AUTOINCREMENT, key TEXT, op TEXT, payload BLOB)")
conn.commit()

client = mqtt.Client(client_id=DEVICE_ID)
client.tls_set()  # system CA; replace with certs for mTLS
client.reconnect_delay_set(min_delay=1, max_delay=30)

def publish_loop():
    last_seq = 0
    while True:
        rows = conn.execute("SELECT seq,key,op,payload FROM changes WHERE seq>?", (last_seq,)).fetchall()
        for seq,key,op,payload in rows:
            msg = {"seq": seq, "key": key, "op": op, "payload": json.loads(payload)}
            client.publish(TOPIC_DELTA.format(device_id=DEVICE_ID), json.dumps(msg), qos=1)
            last_seq = seq
            # persist resume point to a small retained control topic
            client.publish(TOPIC_RESUME.format(device_id=DEVICE_ID), json.dumps({"last_seq": last_seq}), qos=1, retain=True)
        time.sleep(1)  # backoff; tune for latency vs energy

def on_connect(c, userdata, flags, rc):
    c.subscribe(TOPIC_DELTA.format(device_id="+"), qos=1)  # subscribe to peer deltas
    # request resume tokens if necessary (handled by retained TOPIC_RESUME)

def on_message(c, userdata, msg):
    m = json.loads(msg.payload.decode())
    seq = m["seq"]
    key = m["key"]
    op = m["op"]
    payload = json.dumps(m["payload"])
    # idempotent apply using seq check
    existing = conn.execute("SELECT 1 FROM changes WHERE seq=?", (seq,)).fetchone()
    if existing: return
    conn.execute("INSERT INTO changes(seq,key,op,payload) VALUES(?,?,?,?)", (seq,key,op,payload))
    conn.commit()
    # apply to local state machine here (application-specific)

client.on_connect = on_connect
client.on_message = on_message

client.connect(MQTT_BROKER, MQTT_PORT)
threading.Thread(target=publish_loop, daemon=True).start()
client.loop_forever()