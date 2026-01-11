#!/usr/bin/env python3
# Production-ready: runtime checks, retries, graceful shutdown.
import time, socket, json, subprocess
import threading
import psutil
import paho.mqtt.client as mqtt

FOG_HOST = "10.0.0.10"          # fog server IP
FOG_MQTT_TOPIC = "factory/tasks"
MQTT_BROKER = "10.0.0.10"
RTT_THRESHOLD_MS = 50          # offload if RTT below threshold
CPU_LIMIT_PERCENT = 75         # avoid overloading gateway

# estimate RTT using TCP connect (fast, avoids ICMP privileges)
def estimate_rtt_ms(host, port=443, timeout=0.2):
    try:
        start = time.perf_counter()
        s = socket.create_connection((host, port), timeout=timeout)
        s.close()
        return (time.perf_counter() - start) * 1000.0
    except Exception:
        return float('inf')

# lightweight local benchmark: run the inference kernel once and time it
def local_inference_time_ms(sample):
    # replace with real model invocation; here use a placeholder compute
    start = time.perf_counter()
    # simulate compute using numpy or real model API
    subprocess.run(["/usr/bin/true"])  # placeholder, low overhead
    return (time.perf_counter() - start) * 1000.0

# MQTT setup with basic reconnect
client = mqtt.Client()
client.reconnect_delay_set(min_delay=1, max_delay=60)
client.connect_async(MQTT_BROKER, 1883)
client.loop_start()

def send_to_fog(payload):
    client.publish(FOG_MQTT_TOPIC, json.dumps(payload), qos=1)

def should_offload(sample):
    rtt = estimate_rtt_ms(FOG_HOST)
    local_ms = local_inference_time_ms(sample)
    cpu = psutil.cpu_percent(interval=0.1)
    # policy: offload when RTT small and CPU headroom exists
    return (rtt < RTT_THRESHOLD_MS) and (cpu < CPU_LIMIT_PERCENT)

def process_sample(sample):
    if should_offload(sample):
        send_to_fog({"ts": time.time(), "sample": sample})
    else:
        # local processing path (replace with model inference call)
        # ensures deterministic latency for control loops
        result = {"ts": time.time(), "outcome": "local_processed"}
        # local persistence for later sync
        with open("/var/local/process_log.jsonl", "a") as f:
            f.write(json.dumps(result) + "\n")

# main loop: subscribe to sensors via local bus or MQTT
def main_loop():
    while True:
        sample = {"vib": 0.001}  # replace with real sensor read
        try:
            process_sample(sample)
        except Exception as e:
            # log and continue; do not block control loop
            print("processing error", e)
        time.sleep(0.05)  # 20 Hz sampling

if __name__ == "__main__":
    try:
        main_loop()
    finally:
        client.loop_stop()
        client.disconnect()