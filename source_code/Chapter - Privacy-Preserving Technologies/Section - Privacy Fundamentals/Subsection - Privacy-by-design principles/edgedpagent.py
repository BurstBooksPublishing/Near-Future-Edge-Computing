#!/usr/bin/env python3
"""
Edge DP agent: sample sensor, add Laplace noise, sign and publish securely.
Assumes TLS certificates deployed and private key stored in a TEE/HSM.
"""
import os
import time
import json
import hmac
import hashlib
import secrets
import ssl
import paho.mqtt.client as mqtt
import numpy as np

# Configuration (move to secure config store in production)
MQTT_BROKER = "mqtt.example.local"
MQTT_PORT = 8883
MQTT_TOPIC = "factory/vibration/aggregates"
SAMPLE_INTERVAL_S = 1.0
EPSILON = 0.5                 # local DP budget per release
SENSITIVITY = 1.0            # max change in aggregate per single sample

# Use hardware RNG if available
def secure_rand():
    return secrets.token_bytes(32)

# Laplace mechanism
def laplace_noise(scale):
    u = np.random.uniform(-0.5, 0.5)
    return -scale * np.sign(u) * np.log(1 - 2 * abs(u))

def apply_local_dp(value, eps=EPSILON, delta_f=SENSITIVITY):
    b = delta_f / eps
    return float(value + laplace_noise(b))

# Sign payload with HMAC-SHA256 using key stored by TEE/HSM; fallback uses file.
def load_signing_key():
    # In production, retrieve from secure element via PKCS#11 or vendor API.
    key_path = "/etc/edge/secure_sign_key.bin"
    try:
        with open(key_path, "rb") as f:
            return f.read()
    except Exception:
        return secure_rand()

def sign_payload(payload_bytes, key):
    return hmac.new(key, payload_bytes, hashlib.sha256).hexdigest()

# MQTT setup with TLS
client = mqtt.Client()
client.tls_set(ca_certs="/etc/ssl/certs/ca.pem",
               certfile="/etc/ssl/certs/device_cert.pem",
               keyfile="/etc/ssl/private/device_key.pem",
               tls_version=ssl.PROTOCOL_TLS_CLIENT)
client.tls_insecure_set(False)
client.connect(MQTT_BROKER, MQTT_PORT, keepalive=60)

sign_key = load_signing_key()

def sample_sensor():
    # Replace with actual sensor read (SPI/I2C/ADC) for production
    return np.random.normal(0.0, 0.5)

while True:
    try:
        window = [sample_sensor() for _ in range(16)]            # local buffer
        aggregate = float(np.median(window))                     # irreversible feature
        dp_value = apply_local_dp(aggregate)
        payload = {
            "node_id": os.uname().nodename,
            "ts": int(time.time()),
            "agg": dp_value
        }
        payload_bytes = json.dumps(payload, separators=(",",":")).encode("utf-8")
        signature = sign_payload(payload_bytes, sign_key)
        message = {
            "payload": payload,
            "sig": signature,
            "meta": {"method":"laplace","eps":EPSILON}
        }
        client.publish(\lstinline|MQTT_TOPIC|, json.dumps(message), qos=1)
        time.sleep(SAMPLE_INTERVAL_S)
    except Exception as e:
        # Minimal crash-safe logging, integrate with local observability stack
        print("agent error:", e)
        time.sleep(1)