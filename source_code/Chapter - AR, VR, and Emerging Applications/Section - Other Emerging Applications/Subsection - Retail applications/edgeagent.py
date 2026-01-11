#!/usr/bin/env python3
# Minimal dependencies: opencv-python, paho-mqtt, tflite-runtime
import time, signal, sys, json
import cv2
import numpy as np
import paho.mqtt.client as mqtt
from tflite_runtime.interpreter import Interpreter, load_delegate

MODEL_PATH = "/models/shelf_model.tflite"
MQTT_BROKER = "mqtt.local"
MQTT_TOPIC = "store/edge/shelf/events"
CAM_IDX = 0
DEBOUNCE_SEC = 5
SCORE_THRESH = 0.5

running = True
def handle_exit(signum, frame):
    global running; running = False
signal.signal(signal.SIGINT, handle_exit)
signal.signal(signal.SIGTERM, handle_exit)

# Initialize MQTT
client = mqtt.Client()
client.loop_start()
client.connect(MQTT_BROKER, 1883, keepalive=60)

# Load TFLite with optional EdgeTPU delegate
try:
    delegate = load_delegate('libedgetpu.so.1')
    interp = Interpreter(MODEL_PATH, experimental_delegates=[delegate])
except Exception:
    interp = Interpreter(MODEL_PATH)
interp.allocate_tensors()
input_details = interp.get_input_details()
output_details = interp.get_output_details()

# Camera
cap = cv2.VideoCapture(CAM_IDX)
last_events = {}  # SKU -> last timestamp

def preprocess(frame):
    h, w = input_details[0]['shape'][1:3]
    img = cv2.resize(frame, (w, h))
    return np.expand_dims(img.astype(np.uint8), axis=0)

def postprocess(outputs):
    # Example: outputs -> list of (label, score, bbox)
    # Implement model-specific parsing here.
    return outputs

while running:
    ret, frame = cap.read()
    if not ret:
        time.sleep(0.1); continue
    inp = preprocess(frame)
    interp.set_tensor(input_details[0]['index'], inp)
    start = time.time()
    interp.invoke()
    inference_ms = (time.time() - start) * 1000
    # Read outputs (model-specific; placeholder)
    raw = [interp.get_tensor(o['index']) for o in output_details]
    detections = postprocess(raw)
    events = []
    now = time.time()
    for label, score, bbox in detections:
        if score < SCORE_THRESH: continue
        last = last_events.get(label, 0)
        if now - last < DEBOUNCE_SEC: continue
        last_events[label] = now
        events.append({'sku': label, 'score': float(score),
                       'bbox': [int(x) for x in bbox],
                       'ts': int(now), 'inference_ms': int(inference_ms)})
    if events:
        payload = json.dumps({'device': 'edge-01', 'events': events})
        client.publish(MQTT_TOPIC, payload, qos=1)
    # Backoff to respect CPU budget and keep latency predictable
    time.sleep(0.05)

cap.release()
client.loop_stop()
client.disconnect()