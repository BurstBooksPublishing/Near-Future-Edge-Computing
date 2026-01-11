#!/usr/bin/env python3
"""
Edge service: local feature noise injection (LDP) and MQTT publish.
Assumes features computed elsewhere and delivered via UNIX socket or file.
"""
import os, json, time, math, logging, ssl
import random, secrets
import numpy as np
import paho.mqtt.client as mqtt

# Configuration (load from secure config management)
BROKER = os.getenv("MQTT_BROKER", "broker.example.local")
PORT = int(os.getenv("MQTT_PORT", "8883"))
TOPIC = os.getenv("MQTT_TOPIC", "factory/line1/features")
CLIENT_CERT = "/etc/edge/certs/client.pem"
CLIENT_KEY = "/etc/edge/certs/client.key"
CA_CERT = "/etc/edge/certs/ca.pem"
EPSILON = float(os.getenv("LDP_EPSILON", "0.5"))  # privacy budget
SENSITIVITY = float(os.getenv("FEATURE_SENS", "1.0"))  # max L1 change

# Logging
logging.basicConfig(level=logging.INFO)
log = logging.getLogger("edge-ldp")

def laplace_noise(scale: float) -> float:
    # Use cryptographically-secure randomness for production
    u = random.random() - 0.5
    return -scale * math.copysign(math.log(1 - 2 * abs(u)), u)

def sanitize_feature(value: float, sensitivity: float, eps: float) -> float:
    b = sensitivity / eps
    noise = laplace_noise(b)
    return float(value + noise)

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        log.info("Connected to MQTT broker")
    else:
        log.error("MQTT connect failed rc=%s", rc)

def build_mqtt_client():
    client = mqtt.Client(client_id=f"edge-{secrets.token_hex(4)}")
    client.tls_set(ca_certs=CA_CERT, certfile=CLIENT_CERT,
                   keyfile=CLIENT_KEY, tls_version=ssl.PROTOCOL_TLS_CLIENT)
    client.tls_insecure_set(False)
    client.on_connect = on_connect
    client.connect(BROKER, PORT, keepalive=60)
    return client

def publish_feature(client, feature_name: str, value: float):
    sanitized = sanitize_feature(value, SENSITIVITY, EPSILON)
    payload = json.dumps({"feature": feature_name, "value": sanitized, "ts": int(time.time())})
    client.publish(TOPIC, payload, qos=1)
    log.info("Published sanitized feature %s", feature_name)

def main():
    client = build_mqtt_client()
    client.loop_start()
    try:
        while True:
            # Replace with actual data ingestion from sensors or local aggregator.
            raw_feature = np.random.normal(loc=0.0, scale=0.5)
            publish_feature(client, "vibration_rms", float(raw_feature))
            time.sleep(1.0)
    finally:
        client.loop_stop()
        client.disconnect()

if __name__ == "__main__":
    main()