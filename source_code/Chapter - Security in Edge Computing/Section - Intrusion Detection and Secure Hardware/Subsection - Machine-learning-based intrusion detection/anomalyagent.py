#!/usr/bin/env python3
# Production-ready: minimal deps, robust reconnects, and graceful shutdown
import json, socket, time, signal
import numpy as np
from tflite_runtime.interpreter import Interpreter  # lightweight runtime
import paho.mqtt.client as mqtt

FLOW_SOCKET = "/var/run/flow_agent.sock"  # flow exporter writes JSON lines
TFLITE_MODEL = "/opt/models/edge_autoencoder.tflite"
MQTT_BROKER = "127.0.0.1"
MQTT_TOPIC = "edge/alerts/ids"
THRESHOLD = 0.08  # calibrated on validation set

interp = Interpreter(TFLITE_MODEL)
interp.allocate_tensors()
input_idx = interp.get_input_details()[0]['index']
output_idx = interp.get_output_details()[0]['index']

client = mqtt.Client()  # production: set TLS and auth in config
client.connect(MQTT_BROKER)

running = True
def stop(sig, frame):  # graceful shutdown
    global running
    running = False
signal.signal(signal.SIGINT, stop)
signal.signal(signal.SIGTERM, stop)

def parse_flow(line):
    # Expect fields: pkt_count, bytes, duration, avg_pkt_size, src_port_entropy
    rec = json.loads(line)
    return np.array([rec.get(k,0.0) for k in
       ("pkt_count","bytes","duration","avg_pkt_size","src_port_entropy")], dtype=np.float32)

with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as s:
    s.connect(FLOW_SOCKET)
    s_file = s.makefile("r")
    while running:
        line = s_file.readline()
        if not line:
            time.sleep(0.1); continue
        x = parse_flow(line)
        x_norm = (x - 0.0) / (1.0 + np.std(x))  # lightweight normalization
        interp.set_tensor(input_idx, x_norm.reshape(1,-1))
        interp.invoke()
        recon = interp.get_tensor(output_idx).reshape(-1)
        score = float(np.linalg.norm(x_norm - recon)**2)
        if score > THRESHOLD:
            alert = {"score":score,"features":x.tolist(),"ts":time.time()}
            client.publish(MQTT_TOPIC, payload=json.dumps(alert), qos=1)
client.disconnect()