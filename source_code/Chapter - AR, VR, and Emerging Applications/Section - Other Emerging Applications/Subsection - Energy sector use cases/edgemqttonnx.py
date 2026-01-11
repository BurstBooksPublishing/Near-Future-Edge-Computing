#!/usr/bin/env python3
# Edge gateway: subscribe telemetry, run ONNX model, publish control decisions.
import json, time, numpy as np, threading
import paho.mqtt.client as mqtt
import onnxruntime as ort

# Configurable parameters
MQTT_BROKER = "mqtt.local:1883"         # on-prem broker or bridge
TELEM_TOPIC = "plant/site1/sensor/+"    # subscribe using MQTT wildcards
CTRL_TOPIC  = "plant/site1/control"     # control output
MODEL_PATH  = "/opt/models/anomaly.onnx" # quantized ONNX model

# Load ONNX model with GPU if available (TensorRT on Jetson)
sess_options = ort.SessionOptions()
# enable intra-op threads tuning for predictable latency
sess_options.intra_op_num_threads = 2
session = ort.InferenceSession(MODEL_PATH, sess_options)

client = mqtt.Client(client_id="edge-gateway-01")
# TLS, auth, and persistent sessions recommended in production
# client.tls_set(...); client.username_pw_set(...)

def on_connect(c, userdata, flags, rc):
    c.subscribe(TELEM_TOPIC)  # subscribe at QoS 1
def on_message(c, userdata, msg):
    # Minimal parsing; validate, buffer, and batch for inference
    try:
        payload = json.loads(msg.payload.decode())
        features = np.array(payload["features"], dtype=np.float32)  # normalized
        # ONNX expects batch dimension
        inp = features.reshape(1, -1)
        out = session.run(None, {session.get_inputs()[0].name: inp})
        score = float(out[0][0,0])
        if score > 0.9:
            # Local fast action: publish immediate isolation command
            cmd = {"action":"isolate","device":payload["device_id"],"ts":time.time()}
            c.publish(CTRL_TOPIC, json.dumps(cmd), qos=1)
    except Exception as e:
        # avoid crashing; log and continue
        print("msg err:", e)

client.on_connect = on_connect
client.on_message = on_message
client.connect(MQTT_BROKER)
client.loop_start()

# Background thread to run periodic MPC re-dispatch if latency low
def periodic_dispatch():
    while True:
        # fetch local state, solve small convex QP, publish setpoints
        # placeholder for real solver (OSQP, cvxpy) on edge
        setpoints = {"timestamp": time.time(), "setpoints": {"der1": 200.0}}
        client.publish("plant/site1/dispatch", json.dumps(setpoints), qos=1)
        time.sleep(1.0)  # 1 s control loop on gateway

threading.Thread(target=periodic_dispatch, daemon=True).start()

# keep process running
while True:
    time.sleep(60)