#!/usr/bin/env python3
"""
Energy-aware DVFS controller.
- Estimates cycles per job from recent execution times.
- Computes f_deploy per model and writes scaling_max_freq.
- Publishes telemetry to MQTT (paho-mqtt).
Requires: paho-mqtt, psutil. Run as root or with CAP_SYS_ADMIN for freq writes.
"""
import time, glob, subprocess, json, logging
from statistics import mean
import paho.mqtt.client as mqtt

# Parameters (tune per SoC)
k = 1e-28          # dynamic coefficient (example calibrated)
P_s = 1.5          # static power in watts (measured)
SAMPLES_WINDOW = 8
POLL_INTERVAL = 1.0
MQTT_BROKER = "mqtt.example.local"
MQTT_TOPIC = "edge/telemetry/energy"

logging.basicConfig(level=logging.INFO)

def read_recent_execs():
    # Placeholder: collect recent inference latencies from local FIFO or metrics
    # Replace with real metric source (Prometheus node_exporter, local socket)
    return [0.025, 0.023, 0.026, 0.024]  # seconds

def cpu_max_freq_paths():
    return glob.glob("/sys/devices/system/cpu/cpu[0-9]*/cpufreq/scaling_max_freq")

def write_freq_khz(khz):
    for path in cpu_max_freq_paths():
        try:
            with open(path, "w") as f:
                f.write(str(int(khz)))
        except Exception:
            logging.exception("Failed to write freq to %s", path)

def available_freqs_khz():
    # Read from cpu0 scaling_available_frequencies if available
    try:
        with open("/sys/devices/system/cpu/cpu0/cpufreq/scaling_available_frequencies") as f:
            return list(map(int, f.read().split()))
    except Exception:
        # Fallback to cpufreq-info probe (requires cpufrequtils)
        out = subprocess.run(["cpufreq-info","-l"], capture_output=True, text=True)
        if out.returncode==0:
            return list(map(int, out.stdout.split()))
    return []

def compute_f_deploy(C, D_max):
    # C in cycles, D_max seconds. Return frequency in Hz.
    f_opt = (P_s / (2*k))**(1/3)
    f_deadline = C / D_max
    return max(f_opt, f_deadline)

def cycles_from_latency(lat):
    # Estimate cycles from latency and nominal frequency probe
    try:
        with open("/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq") as f:
            cur_khz = int(f.read().strip())
        cur_hz = cur_khz*1000
    except Exception:
        cur_hz = 1.5e9
    return cur_hz * lat

def main():
    client = mqtt.Client()
    client.connect(MQTT_BROKER, 1883, 60)
    freqs = available_freqs_khz()
    if freqs:
        freqs_sorted = sorted(freqs)
    else:
        freqs_sorted = []
    while True:
        latencies = read_recent_execs()
        if len(latencies)==0:
            time.sleep(POLL_INTERVAL); continue
        lat = mean(latencies[-SAMPLES_WINDOW:])
        C = cycles_from_latency(lat)
        D_max = 0.03  # target deadline in seconds (tunable)
        f_hz = compute_f_deploy(C, D_max)
        # choose nearest available frequency (kHz)
        if freqs_sorted:
            khz = min(freqs_sorted, key=lambda x: abs(x - f_hz/1000))
        else:
            khz = int(f_hz/1000)
        write_freq_khz(khz)
        payload = {"timestamp": time.time(), "latency": lat, "f_khz": khz}
        client.publish(MQTT_TOPIC, json.dumps(payload), qos=1)
        logging.info("Set %d kHz, latency %.4fs", khz, lat)
        time.sleep(POLL_INTERVAL)

if __name__=="__main__":
    main()