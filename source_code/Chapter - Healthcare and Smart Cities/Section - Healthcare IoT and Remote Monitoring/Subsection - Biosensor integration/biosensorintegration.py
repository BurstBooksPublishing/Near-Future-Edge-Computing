#!/usr/bin/env python3
# Dependencies: smbus2, numpy, scipy, paho.mqtt.client, sqlite3, ssl
import time, ssl, sqlite3, json, logging
from smbus2 import SMBus
import numpy as np
from scipy.signal import butter, sosfiltfilt
import paho.mqtt.client as mqtt

# Configuration
I2C_BUS = 1
MAX30102_ADDR = 0x57
ADS1115_ADDR = 0x48
MQTT_BROKER = "broker.example.healthcare"
MQTT_PORT = 8883
MQTT_TOPIC = "edge/clinic/device01/observations"
TLS_CA = "/etc/ssl/certs/ca.pem"
TLS_CERT = "/etc/ssl/certs/device.crt"
TLS_KEY = "/etc/ssl/private/device.key"
DB_PATH = "/var/lib/biosensor/data.db"

# Initialize logger, DB, MQTT
logging.basicConfig(level=logging.INFO)
db = sqlite3.connect(DB_PATH, check_same_thread=False)
db.execute("CREATE TABLE IF NOT EXISTS observations(ts REAL, payload TEXT)")
client = mqtt.Client()
client.tls_set(ca_certs=TLS_CA, certfile=TLS_CERT, keyfile=TLS_KEY,
               cert_reqs=ssl.CERT_REQUIRED, tls_version=ssl.PROTOCOL_TLSv1_2)
client.connect(MQTT_BROKER, MQTT_PORT, keepalive=60)

# Simple bandpass design (example: ECG 0.5-40 Hz, Fs=300)
Fs = 300.0
sos = butter(4, [0.5, 40], btype='band', fs=Fs, output='sos')

def read_max30102(bus):
    # Minimal read stub: replace with production driver for sensor specifics.
    # Return PPG sample as integer.
    raw = bus.read_i2c_block_data(MAX30102_ADDR, 0x03, 3)
    sample = (raw[0] << 16) | (raw[1] << 8) | raw[2]
    return sample & 0x3FFFF

def read_ads1115_ecg():
    # Use I2C ADC read; here stub returns synthetic sample in production code replace read.
    return int(2048 * np.sin(2*np.pi*1.2*time.time()) + 2048)

def preprocess_ecg(ecg_buffer):
    # Zero-phase bandpass, remove baseline wander and high-frequency noise.
    filtered = sosfiltfilt(sos, np.array(ecg_buffer))
    return filtered

def extract_hr_from_ppg(ppg_window, fs=100.0):
    # Simple peak detection by derivative thresholding; production use validated algorithm.
    # Return beats per minute estimate.
    x = np.array(ppg_window)
    diff = np.diff(x)
    peaks = np.where((diff[:-1] > 0) & (diff[1:] <= 0))[0]
    if len(peaks) < 2: return None
    intervals = np.diff(peaks) / fs
    bpm = 60.0 / np.median(intervals)
    return float(bpm)

# Main loop
with SMBus(I2C_BUS) as bus:
    ecg_buf, ppg_buf = [], []
    try:
        while True:
            ppg = read_max30102(bus)
            ecg = read_ads1115_ecg()
            ppg_buf.append(ppg); ecg_buf.append(ecg)
            # Maintain windows
            if len(ppg_buf) > 500: ppg_buf.pop(0)
            if len(ecg_buf) > int(Fs*5): ecg_buf.pop(0)
            # Periodic processing every second
            if int(time.time() % 1) == 0 and len(ppg_buf) >= 100:
                ecg_f = preprocess_ecg(ecg_buf[-int(Fs*5):])
                hr_ppg = extract_hr_from_ppg(ppg_buf[-200:], fs=100.0)
                payload = {"ts": time.time(), "hr_ppg": hr_ppg, "ecg_snapshot": ecg_f[-300:].tolist()[:300]}
                # Local persistence then secure publish
                db.execute("INSERT INTO observations VALUES (?,?)", (payload["ts"], json.dumps(payload)))
                db.commit()
                client.publish(MQTT_TOPIC, json.dumps(payload), qos=1)
            time.sleep(0.01)
    except Exception as e:
        logging.exception("Acquisition loop failed: %s", e)