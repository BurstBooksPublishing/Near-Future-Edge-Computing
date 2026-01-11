#!/usr/bin/env python3
# Encrypt sensor payload, store audit log, publish over MQTT TLS.
import os, json, time
from cryptography.hazmat.primitives.ciphers.aead import AESGCM
import paho.mqtt.client as mqtt

# Load key from secure storage; in prod use TPM/PKCS11, here read from file for example.
with open("/etc/edge/keys/device_aes_key.bin","rb") as f:
    key = f.read()  # 32 bytes for AES-256-GCM

aesgcm = AESGCM(key)
MQTT_BROKER="iot.example.com"
MQTT_PORT=8883
MQTT_TOPIC="medical/ecg"

def encrypt_and_store(payload: bytes, audit_path="/var/log/edge_secure.log"):
    nonce = os.urandom(12)
    ct = aesgcm.encrypt(nonce, payload, None)
    record = {"ts": time.time(), "nonce": nonce.hex(), "ct_len": len(ct)}
    # append audit record; file owner and perms must enforce least privilege.
    with open(audit_path,"a") as a:
        a.write(json.dumps(record)+"\n")
    return nonce, ct

def on_connect(client, userdata, flags, rc):
    if rc != 0:
        raise SystemExit("MQTT connect failed: %s" % rc)

client = mqtt.Client()
client.tls_set(ca_certs="/etc/ssl/certs/ca.pem",
               certfile="/etc/ssl/certs/device.crt",
               keyfile="/etc/ssl/private/device.key")
client.on_connect = on_connect
client.connect(MQTT_BROKER, MQTT_PORT, keepalive=60)
client.loop_start()

# Example publish routine
raw_payload = b'{"patient_id":"hashed_id","ecg": [ ... ] }'
nonce, ciphertext = encrypt_and_store(raw_payload)
# Publish ciphertext base64 or binary; retain minimal metadata.
client.publish(MQTT_TOPIC, payload=ciphertext, qos=1)
client.loop_stop()
client.disconnect()