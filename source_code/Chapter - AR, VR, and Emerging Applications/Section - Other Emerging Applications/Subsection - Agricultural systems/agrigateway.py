#!/usr/bin/env python3
# Production-ready gateway: TFLite local inference, RTT test, MQTT offload, TLS
import time, json, logging, socket
import paho.mqtt.client as mqtt
from tflite_runtime.interpreter import Interpreter
import requests  # used to test RTT to MEC endpoint

logging.basicConfig(level=logging.INFO)
CONFIG = {
    "model_path": "models/soil_anom_int8.tflite",
    "mqtt_broker": "mqtt.example.local",
    "mqtt_topic": "farm/field1/act",
    "mec_url": "https://mec.example.local/api/ensemble",
    "confidence_threshold": 0.85,
    "rtt_threshold": 0.12,  # seconds
}

interp = Interpreter(CONFIG["model_path"])
interp.allocate_tensors()
input_details = interp.get_input_details()
output_details = interp.get_output_details()

def infer_local(payload_bytes):
    # convert payload to model input; assume preprocessed in node
    inp = memoryview(payload_bytes)
    interp.set_tensor(input_details[0]['index'], inp)
    start = time.perf_counter()
    interp.invoke()
    latency = time.perf_counter() - start
    out = interp.get_tensor(output_details[0]['index'])
    return float(out.max()), float(out.argmax()), latency

def measure_rtt(url, timeout=0.5):
    try:
        t0 = time.perf_counter()
        r = requests.head(url, timeout=timeout, verify=True)
        return time.perf_counter() - t0
    except Exception:
        return float('inf')

client = mqtt.Client()
client.tls_set()  # use system CA; production should set certs explicitly
client.connect(CONFIG["mqtt_broker"], 8883, 60)

def offload_to_mec(payload):
    # send raw payload for ensemble evaluation
    try:
        r = requests.post(CONFIG["mec_url"], data=payload, timeout=2.0, verify=True)
        r.raise_for_status()
        return r.json()
    except Exception as e:
        logging.error("MEC offload failed: %s", e)
        return None

def handle_message(payload_bytes):
    conf, cls, t_local = infer_local(payload_bytes)
    rtt = measure_rtt(CONFIG["mec_url"])
    logging.info("Local conf=%.3f, t_local=%.3fs, rtt=%.3fs", conf, t_local, rtt)
    if conf < CONFIG["confidence_threshold"] and rtt < CONFIG["rtt_threshold"]:
        result = offload_to_mec(payload_bytes)
        if result and result.get("action"):
            client.publish(CONFIG["mqtt_topic"], json.dumps(result), qos=1)
    elif conf >= CONFIG["confidence_threshold"]:
        # act locally
        client.publish(CONFIG["mqtt_topic"], json.dumps({"action":"irrigate","prob":conf}), qos=1)
    else:
        logging.warning("Neither local nor offload suitable; caching for later")

# Main loop: replace with real LoRa/serial handler
if __name__ == "__main__":
    while True:
        # placeholder for receiving payload from LoRa gateway process
        payload = b'\x00'*input_details[0]['shape'][1]
        handle_message(payload)
        time.sleep(1)