#!/usr/bin/env python3
# Add Laplace noise using secure RNG and publish via paho-mqtt.
import math
import json
import time
import random
import paho.mqtt.client as mqtt

rng = random.SystemRandom()   # cryptographically secure RNG using os.urandom

def laplace_noise(scale: float) -> float:
    # U ~ Uniform(-0.5,0.5)
    u = rng.random() - 0.5
    return -scale * math.copysign(1.0, u) * math.log(1.0 - 2.0 * abs(u))

def publish_noisy_reading(client, topic: str, value: float, sensitivity: float, eps: float):
    b = sensitivity / eps
    noisy = value + laplace_noise(b)
    payload = json.dumps({"ts": int(time.time()), "value": noisy})
    client.publish(topic, payload, qos=1)  # persistent QoS for edge reliability

# MQTT client setup (use TLS in production with certificates)
client = mqtt.Client(client_id="edge-gateway-01")
client.tls_set()                     # require proper certs in production
client.username_pw_set("device","pw")
client.connect("broker.example.com", 8883)
client.loop_start()

# Example usage: publish aggregated reading every minute
SENSITIVITY = 1.0   # change according to aggregation granularity
EPSILON = 0.5       # per-report privacy budget
try:
    while True:
        reading = 12.3                      # obtain from ADC, sensor driver, or local DB
        publish_noisy_reading(client, "grid/meter/area42", reading, SENSITIVITY, EPSILON)
        time.sleep(60)
finally:
    client.loop_stop()
    client.disconnect()