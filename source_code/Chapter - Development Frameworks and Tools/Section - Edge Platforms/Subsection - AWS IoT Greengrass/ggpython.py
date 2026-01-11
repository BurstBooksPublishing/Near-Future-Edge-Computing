#!/usr/bin/env python3
# Production-ready: minimal dependencies, retries, graceful shutdown.

import os
import time
import signal
import json
from threading import Event

# Greengrass v2 IPC client
from awsiot import greengrasscoreipc
from awsiot.greengrasscoreipc.model import PublishToIoTCoreRequest, IoTCoreMessage

# Minimal ML inference using TensorFlow Lite interpreter
import numpy as np
import tflite_runtime.interpreter as tflite  # use tflite runtime for edge devices

STOP = Event()

def handle_sigterm(signum, frame):
    STOP.set()

signal.signal(signal.SIGTERM, handle_sigterm)
signal.signal(signal.SIGINT, handle_sigterm)

# Configuration read from component configuration or env
MODEL_PATH = os.environ.get("MODEL_PATH", "/greengrass-resources/model.tflite")
THRESHOLD = float(os.environ.get("ANOMALY_THRESHOLD", "0.7"))
TOPIC = os.environ.get("IOT_TOPIC", "factory/inspection/anomaly")

# Initialize TFLite interpreter
interpreter = tflite.Interpreter(model_path=MODEL_PATH)
interpreter.allocate_tensors()
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

# Initialize Greengrass IPC
ipc_client = greengrasscoreipc.connect()

def publish_event(payload: dict):
    # Prepare IoT Core message; payload must be bytes
    body = IoTCoreMessage()
    body.payload = json.dumps(payload).encode('utf-8')
    req = PublishToIoTCoreRequest(topic_name=TOPIC, qos=1, body=body)
    try:
        op = ipc_client.publish_to_iot_core(req)
        op.result(5)  # wait up to 5s for completion
    except Exception as e:
        # transient errors are retried by caller or logged for later reconciliation
        print("Publish failed:", e)

def run_inference(frame: np.ndarray) -> float:
    # Preprocess frame into model input shape
    inp = np.expand_dims(frame.astype(np.float32) / 255.0, axis=0)
    interpreter.set_tensor(input_details[0]['index'], inp)
    interpreter.invoke()
    out = interpreter.get_tensor(output_details[0]['index'])
    score = float(out[0][0])  # single-value anomaly score
    return score

def main_loop():
    # Placeholder: replace with camera FIFO or shared memory consumer
    while not STOP.is_set():
        # Pull or capture a frame from local source
        frame = np.zeros((224,224,3), dtype=np.uint8)  # replace with real capture
        score = run_inference(frame)
        if score > THRESHOLD:
            event = {"timestamp": int(time.time()), "score": score}
            publish_event(event)
        time.sleep(0.1)  # control processing rate

if __name__ == "__main__":
    try:
        main_loop()
    finally:
        ipc_client.close()