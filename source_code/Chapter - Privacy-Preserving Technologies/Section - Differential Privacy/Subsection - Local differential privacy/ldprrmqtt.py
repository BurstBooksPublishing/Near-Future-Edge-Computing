#!/usr/bin/env python3
# Minimal production-ready client: k-ary randomized response + MQTT publish.
import json, time, ssl, secrets, random
import paho.mqtt.client as mqtt

BROKER = "broker.example.com"
PORT = 8883
TOPIC = "factory/sensor/priv"
CLIENT_ID = "edge-node-01"
TLS_CERT = "/etc/certs/device.crt"
TLS_KEY = "/etc/certs/device.key"
CA_CERT = "/etc/certs/ca.pem"

k = 10                     # domain size
eps = 1.0                  # privacy budget per report
p = (2.718281828459045**eps) / (2.718281828459045**eps + k - 1)
q = 1.0 / (2.718281828459045**eps + k - 1)

def privatize(category):
    # category: integer in [0, k-1]; returns privatized integer
    r = secrets.randbelow(10**6) / 10**6           # secure uniform in [0,1)
    if r < p:
        return category
    # choose uniformly among other categories
    others = list(range(k))
    others.remove(category)
    return secrets.choice(others)

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        client.connected_flag = True

# MQTT client setup with TLS and robust reconnect
client = mqtt.Client(client_id=CLIENT_ID, clean_session=True)
client.tls_set(ca_certs=CA_CERT, certfile=TLS_CERT, keyfile=TLS_KEY,
               tls_version=ssl.PROTOCOL_TLSv1_2)
client.reconnect_delay_set(min_delay=1, max_delay=60)
client.on_connect = on_connect
client.connected_flag = False
client.connect_async(BROKER, PORT)
client.loop_start()

def publish_batch(samples):
    # samples: list of integer categories; batch privatize and publish JSON
    reports = [privatize(s) for s in samples]
    payload = json.dumps({"node": CLIENT_ID, "ts": int(time.time()), "reports": reports})
    # QoS=1 ensures broker acknowledgement; retain=False for streaming telemetry
    client.publish(TOPIC, payload, qos=1, retain=False)

# Example runtime loop (replace with sensor sampling code)
try:
    while True:
        if not getattr(client, "connected_flag", False):
            time.sleep(0.5); continue
        # collect or receive 20 categories locally, then publish
        samples = [random.randrange(k) for _ in range(20)]  # replace with sensor read
        publish_batch(samples)
        time.sleep(5)   # batching interval tuned for latency/efficiency trade-off
finally:
    client.loop_stop(); client.disconnect()