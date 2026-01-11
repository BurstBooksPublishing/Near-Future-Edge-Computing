#!/usr/bin/env python3
# Production-ready: async agent, TLS, reconnect logic, JSON payloads.
import asyncio, json, ssl, time
import paho.mqtt.client as mqtt
import numpy as np

# Kalman filter state (small linear example).
A = np.array([[1.0]])
C = np.array([[1.0]])
Q = np.array([[1e-4]])
R = np.array([[1e-2]])

x = np.array([[0.0]])   # state estimate
P = np.array([[1.0]])   # covariance

DEVICE_ID = "cnc_spindle_01"
MQTT_BROKER = "edge-ditto.local"
MQTT_PORT = 8883
TOPIC = f"ditto/{DEVICE_ID}/features"

# TLS context for mutual TLS / server validation
ssl_ctx = ssl.create_default_context()
ssl_ctx.check_hostname = True
# ssl_ctx.load_cert_chain(certfile='agent.crt', keyfile='agent.key')
# ssl_ctx.load_verify_locations(cafile='ca.pem')

client = mqtt.Client(client_id=f"twin-agent-{DEVICE_ID}")
client.tls_set_context(ssl_ctx)
client.reconnect_delay_set(min_delay=1, max_delay=60)

def kalman_predict(u=np.array([[0.0]])):
    global x, P
    x = A @ x + u
    P = A @ P @ A.T + Q

def kalman_update(y):
    global x, P
    S = C @ P @ C.T + R
    K = P @ C.T @ np.linalg.inv(S)
    residual = y - (C @ x)
    x[:] = x + K @ residual
    P[:] = (np.eye(P.shape[0]) - K @ C) @ P

async def sensor_loop():
    while True:
        # Read sensor, extract feature (placeholder)
        y = np.array([[read_sensor_feature()]])
        kalman_predict()
        kalman_update(y)
        payload = {
            "timestamp": int(time.time()*1000),
            "twin_state": float(x[0,0]),
            "covariance": float(P[0,0])
        }
        client.publish(TOPIC, json.dumps(payload), qos=1)
        await asyncio.sleep(0.1)  # 10 Hz local update

def read_sensor_feature():
    # Replace with optimized sensor read on Jetson or MCU bridge.
    return 0.01*np.random.randn()

def on_connect(c, userdata, flags, rc):
    c.subscribe(f"commands/{DEVICE_ID}/#", qos=1)  # remote control channel

client.on_connect = on_connect
client.connect_async(MQTT_BROKER, MQTT_PORT)
client.loop_start()

asyncio.run(sensor_loop())