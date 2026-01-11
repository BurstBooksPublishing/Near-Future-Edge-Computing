#!/usr/bin/env python3
import os, time, json, signal
import paho.mqtt.client as mqtt
import tensorrt as trt
import numpy as np
from threading import Event

# Configuration (adjust for target device)
ENGINE_PATH = "/opt/models/model_fp16.trt"   # TensorRT engine file
MQTT_BROKER = "mqtt.example.local"
MQTT_TOPIC = "factory/line1/device42/telemetry"
MAX_INFLIGHT = 4  # simple admission threshold

stop_evt = Event()
signal.signal(signal.SIGTERM, lambda *_: stop_evt.set())

def load_engine(path):
    TRT_LOGGER = trt.Logger(trt.Logger.WARNING)
    with open(path, "rb") as f, trt.Runtime(TRT_LOGGER) as runtime:
        return runtime.deserialize_cuda_engine(f.read())

def infer_loop(engine, mqtt_client):
    # minimal buffer setup omitted; use proper CUDA buffers in production
    context = engine.create_execution_context()
    inflight = 0
    while not stop_evt.is_set():
        # Simple admission control
        if inflight >= MAX_INFLIGHT:
            time.sleep(0.005)  # backoff when overloaded
            continue
        inflight += 1
        t0 = time.time()
        # Mock input: replace with camera/capture pipeline
        input_data = np.random.rand(1, 3, 224, 224).astype(np.float32)
        # Execute inference -- use proper memory copies in production
        context.execute_v2([])  # placeholder for real bindings
        t_inf = time.time() - t0
        inflight -= 1
        payload = {
            "timestamp": time.time(),
            "latency_ms": round(t_inf*1000, 2),
            "inflight": inflight
        }
        mqtt_client.publish(MQTT_TOPIC, json.dumps(payload), qos=1)
        # Adaptation example: reduce MAX_INFLIGHT if latency spikes
        if payload["latency_ms"] > 50:
            # operator policy: reduce concurrency to avoid thermal throttling
            try:
                MAX_INFLIGHT = max(1, MAX_INFLIGHT - 1)
            except Exception:
                pass

def main():
    # Connect MQTT securely; production must enable TLS and auth
    mqtt_client = mqtt.Client()
    mqtt_client.connect(MQTT_BROKER, 1883, 60)
    engine = load_engine(ENGINE_PATH)
    try:
        infer_loop(engine, mqtt_client)
    finally:
        mqtt_client.disconnect()

if __name__ == "__main__":
    main()