#!/usr/bin/env python3
"""
Edge agent for smart-grid fault detection.
- MQTT ingest from IEC61850 gateway
- Local ONNX inference with dynamic offload to MEC HTTP gRPC endpoint
- Uses psutil for CPU load checks and requests for offload
"""
import os, time, json, asyncio, logging, requests
import paho.mqtt.client as mqtt
import onnxruntime as ort
import psutil

# Configuration (set via env)
MQTT_BROKER = os.getenv("MQTT_BROKER", "localhost")
MQTT_TOPIC = os.getenv("MQTT_TOPIC", "grid/feeder1/phases")
MEC_URL = os.getenv("MEC_URL", "https://mec.local/api/infer")
CPU_OFFLOAD_THRESHOLD = float(os.getenv("CPU_OFFLOAD_THRESHOLD", "0.80"))
LATENCY_SLA_MS = float(os.getenv("LATENCY_SLA_MS", "200"))

# Load ONNX model (quantized model for edge)
sess = ort.InferenceSession("/opt/models/fault_detector.onnx", providers=['CPUExecutionProvider'])

def local_infer(sample):
    """Run ONNX inference; sample: numpy array or list."""
    inp = {sess.get_inputs()[0].name: sample}
    out = sess.run(None, inp)
    return out[0]

def estimate_mec_latency():
    """Probe MEC RTT (simple HTTP HEAD); returns ms."""
    try:
        t0 = time.time()
        r = requests.head(MEC_URL, timeout=0.2, verify=True)
        return (time.time()-t0)*1000.0
    except Exception:
        return float('inf')

def should_offload():
    """Decision: offload if CPU overloaded and MEC latency allows SLA."""
    cpu = psutil.cpu_percent(interval=0.05)/100.0
    if cpu < CPU_OFFLOAD_THRESHOLD:
        return False
    rtt = estimate_mec_latency()
    # Add processing budget and serialization overhead (~10 ms)
    return (rtt + 10.0) < LATENCY_SLA_MS

def on_message(client, userdata, msg):
    try:
        sample = json.loads(msg.payload)  # expected numeric array
    except Exception:
        logging.exception("bad message")
        return
    if should_offload():
        # Offload to MEC: synchronous POST; include device id and metadata
        try:
            rsp = requests.post(MEC_URL, json={"device":"feeder1","sample":sample}, timeout=0.2)
            decision = rsp.json().get("decision")
        except Exception:
            logging.exception("offload failed; fallback to local")
            decision = local_infer(sample)
    else:
        decision = local_infer(sample)
    # Actuate or publish decision
    client.publish("grid/feeder1/act", json.dumps({"decision": decision.tolist()}), qos=1)

def main():
    logging.basicConfig(level=logging.INFO)
    client = mqtt.Client()
    client.tls_set()  # require broker TLS; certs managed by device
    client.connect(MQTT_BROKER, 8883, 60)
    client.on_message = on_message
    client.subscribe(MQTT_TOPIC, qos=1)
    client.loop_forever()

if __name__ == "__main__":
    main()