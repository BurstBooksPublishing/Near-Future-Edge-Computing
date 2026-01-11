#!/usr/bin/env python3
# Minimal, robust asyncio service for edge pose smoothing and MQTT distribution.
import asyncio
import json
import time
from paho.mqtt import client as mqtt
import numpy as np

BROKER = "127.0.0.1"           # local KubeEdge/EMQX broker
POSE_TOPIC = "metaverse/pose"  # input from device gateways
PUB_TOPIC = "metaverse/state"  # output to headset clients

# Simple constant-velocity Kalman filter implementation
class Kalman1D:
    def __init__(self, q=1e-3, r=1e-2):
        self.x = np.zeros(2)      # [pos, vel]
        self.P = np.eye(2)
        self.dt = 0.05
        self.Q = q * np.array([[self.dt**4/4, self.dt**3/2],
                               [self.dt**3/2, self.dt**2]])
        self.R = r

    def predict(self):
        F = np.array([[1, self.dt],[0,1]])
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + self.Q

    def update(self, z):
        H = np.array([[1,0]])
        y = z - H @ self.x
        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T / S
        self.x += K.flatten() * y
        self.P = (np.eye(2) - K @ H) @ self.P

# MQTT client setup
client = mqtt.Client()
client.connect(BROKER, 1883, 60)

filters = {}  # per-device kalman filters

def on_pose(msg):
    payload = json.loads(msg.payload.decode())
    dev = payload["device_id"]
    ts = payload.get("ts", time.time())
    pos = float(payload["pos"])  # scalar along axis for brevity
    kf = filters.setdefault(dev, Kalman1D())
    kf.predict()
    kf.update(pos)
    # Predict 50 ms into future to hide network jitter
    kf.dt = 0.05
    kf.predict()
    out = {"device_id": dev, "ts": ts, "pred_pos": float(kf.x[0])}
    client.publish(PUB_TOPIC, json.dumps(out), qos=1)

# subscribe loop (blocking callback implemented for clarity)
def start_subscriber():
    sub = mqtt.Client()
    sub.connect(BROKER)
    def on_message(client, userdata, message):
        on_pose(message)
    sub.on_message = on_message
    sub.subscribe(POSE_TOPIC, qos=1)
    sub.loop_forever()

if __name__ == "__main__":
    start_subscriber()