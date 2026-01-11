#!/usr/bin/env python3
"""Post reading to EdgeX Core Data and publish MQTT notification."""
import os, json, requests, ssl
import paho.mqtt.client as mqtt

# Configuration from env vars for deployment flexibility
EDGEX_URL = os.environ.get("EDGEX_URL", "http://127.0.0.1:48080")  # EdgeX Core Data
MQTT_BROKER = os.environ.get("MQTT_BROKER", "mqtt.example.com")
MQTT_PORT = int(os.environ.get("MQTT_PORT", "8883"))

def post_reading(device_name: str, resource_name: str, value: float) -> dict:
    payload = {
        "device": device_name,
        "readings": [{"resource": resource_name, "value": str(value)}]
    }
    url = f"{EDGEX_URL}/api/v1/event"
    resp = requests.post(url, json=payload, timeout=5)
    resp.raise_for_status()
    return resp.json()

def mqtt_publish(topic: str, message: dict):
    client = mqtt.Client()  # use TLS in production: client.tls_set(...)
    client.tls_set()        # default certs; replace with valid CA/client certs
    client.tls_insecure_set(False)
    client.connect(MQTT_BROKER, MQTT_PORT, keepalive=60)
    client.publish(topic, json.dumps(message), qos=1)
    client.disconnect()

if __name__ == "__main__":
    # Real deployments use device identity tied to EVE attestation and EdgeX auth
    try:
        event = post_reading("vibe-gw-01", "vibration", 12.34)
        mqtt_publish("factory/line1/vibe", {"event_id": event.get("id")})
    except Exception as e:
        # Log error to local syslog or observability agent; fail-safe to buffer on disk
        print("ERROR:", e)
        raise