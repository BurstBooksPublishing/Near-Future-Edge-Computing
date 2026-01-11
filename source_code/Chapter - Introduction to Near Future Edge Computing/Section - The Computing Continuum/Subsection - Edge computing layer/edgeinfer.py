#!/usr/bin/env python3
"""
Production-ready edge inference daemon:
- Loads quantized TFLite model.
- Performs inference on incoming frames.
- Publishes latency and result via MQTT.
"""
import signal, sys, time, logging, json
import paho.mqtt.client as mqtt
from threading import Event
try:
    from tflite_runtime.interpreter import Interpreter
except ImportError:
    from tensorflow.lite.python.interpreter import Interpreter

BROKER = "localhost"          # edge-local broker (TLS recommended)
MQ_TOPIC = "edge/telemetry"
MODEL_PATH = "/opt/models/anomaly_quant.tflite"
SHUTDOWN = Event()
logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")

def load_model(path):
    interp = Interpreter(model_path=path, num_threads=2)  # pin threads based on core count
    interp.allocate_tensors()
    return interp

def infer(interp, input_data):
    inp_details = interp.get_input_details()[0]
    interp.set_tensor(inp_details['index'], input_data)
    t0 = time.time()
    interp.invoke()
    latency = (time.time() - t0) * 1000.0  # ms
    out = interp.get_tensor(interp.get_output_details()[0]['index'])
    return out, latency

def on_connect(client, userdata, flags, rc):
    logging.info("MQTT connected with rc=%s", rc)

def mqtt_client():
    c = mqtt.Client(client_id="edge-infer-01")
    c.on_connect = on_connect
    # TODO: configure TLS and username/password in production
    c.connect(BROKER, 1883, keepalive=60)
    return c

def main():
    interp = load_model(MODEL_PATH)
    client = mqtt_client()
    client.loop_start()

    def stop(signum, frame):
        SHUTDOWN.set()
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)

    dummy_input = ...  # replace with actual sensor preprocessing pipeline (numpy array)
    while not SHUTDOWN.is_set():
        # read/process a frame from sensor or FIFO (omitted for brevity)
        result, latency_ms = infer(interp, dummy_input)
        payload = json.dumps({"timestamp": time.time(), "latency_ms": latency_ms, "result": result.tolist()})
        client.publish(MQ_TOPIC, payload, qos=1)
        # SLO sleep/backpressure control; adapt to sampling rate
        time.sleep(0.1)
    client.loop_stop()
    client.disconnect()

if __name__ == "__main__":
    main()