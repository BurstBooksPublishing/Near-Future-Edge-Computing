#!/usr/bin/env python3
# Production-ready edge agent for vibration condition monitoring.
import time, json, sqlite3, signal
import numpy as np
import paho.mqtt.client as mqtt
from tflite_runtime.interpreter import Interpreter  # lightweight runtime
from smbus2 import SMBus  # I2C access on Linux-based SBCs

I2C_ADDR = 0x1D  # example accelerometer address
SAMPLE_RATE = 2000
WINDOW = 256
MQTT_BROKER = "mqtt.local"
MODEL_PATH = "model.tflite"
DB_PATH = "/var/lib/condmon/measurements.db"

# Initialize resources
bus = SMBus(1)
interpreter = Interpreter(model_path=MODEL_PATH)
interpreter.allocate_tensors()
conn = sqlite3.connect(DB_PATH, check_same_thread=False)
c = conn.cursor()
c.execute("CREATE TABLE IF NOT EXISTS events(ts REAL, feat BLOB, score REAL)")

mqttc = mqtt.Client()
mqttc.reconnect_delay_set(min_delay=1, max_delay=60)
mqttc.connect_async(MQTT_BROKER)
mqttc.loop_start()

running = True
def shutdown(sig, frame):
    global running
    running = False
signal.signal(signal.SIGINT, shutdown)

def read_accel_batch(n):
    # Read n samples from accelerometer; implement efficient block reads per device API.
    samples = np.empty((n,3), dtype=np.float32)
    for i in range(n):
        # read 6 bytes, convert to signed integers, scale to g's
        data = bus.read_i2c_block_data(I2C_ADDR, 0x32, 6)
        x = np.int16(data[0] | (data[1]<<8)) * 0.000244
        y = np.int16(data[2] | (data[3]<<8)) * 0.000244
        z = np.int16(data[4] | (data[5]<<8)) * 0.000244
        samples[i] = (x,y,z)
    return samples

def extract_features(window):
    # Time-domain RMS, spectral centroid, and band powers.
    rms = np.sqrt(np.mean(window**2, axis=0))
    fft = np.fft.rfft(window[:,0]*np.hanning(window.shape[0]))
    freqs = np.fft.rfftfreq(window.shape[0], d=1.0/SAMPLE_RATE)
    mag = np.abs(fft)
    centroid = np.sum(freqs*mag)/np.sum(mag)
    band_power = np.sum(mag[(freqs>100)&(freqs<1000)])  # example band
    return np.hstack([rms, centroid, band_power]).astype(np.float32)

# Warm-up inference
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()
dummy = np.zeros(input_details[0]['shape'], dtype=np.float32)
interpreter.set_tensor(input_details[0]['index'], dummy)
interpreter.invoke()

while running:
    try:
        batch = read_accel_batch(WINDOW)
        feat = extract_features(batch)
        # run TFLite model
        interpreter.set_tensor(input_details[0]['index'], feat.reshape(input_details[0]['shape']))
        interpreter.invoke()
        score = float(interpreter.get_tensor(output_details[0]['index'])[0])
        ts = time.time()
        c.execute("INSERT INTO events VALUES(?,?,?)", (ts, feat.tobytes(), score))
        conn.commit()
        if score > 0.8:  # thresholded alarm
            payload = json.dumps({"ts": ts, "score": score})
            mqttc.publish("plant/line1/condmon/alarm", payload, qos=1)
        time.sleep(0.01)  # pacing to maintain sample rate
    except Exception as e:
        mqttc.publish("plant/line1/condmon/errors", json.dumps({"error": str(e)}), qos=0)
        time.sleep(1)
# cleanup
mqttc.loop_stop()
mqttc.disconnect()
conn.close()
bus.close()