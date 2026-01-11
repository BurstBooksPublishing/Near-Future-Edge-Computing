#!/usr/bin/env python3
# Production-ready edge service: local inference, persistence, MQTT publish with backoff.
import os, signal, time, sqlite3, json, logging
import paho.mqtt.client as mqtt
import tflite_runtime.interpreter as tflite
import cv2

# Config via env vars for container deployment
DEVICE_ID = os.getenv("DEVICE_ID","junction-42")
MQTT_BROKER = os.getenv("MQTT_BROKER","mqtt.example.local")
MQTT_TOPIC = f"city/{DEVICE_ID}/counts"
PUBLISH_INTERVAL = int(os.getenv("PUBLISH_INTERVAL",10))
MODEL_PATH = os.getenv("MODEL_PATH","/models/vehicle_detector.tflite")
DB_PATH = os.getenv("DB_PATH","/var/lib/edge/counts.db")

# init logging and DB
logging.basicConfig(level=logging.INFO)
conn = sqlite3.connect(DB_PATH, check_same_thread=False)
conn.execute("CREATE TABLE IF NOT EXISTS counts(ts INTEGER, count INTEGER)")
conn.commit()

# load TFLite model
interpreter = tflite.Interpreter(model_path=MODEL_PATH)
interpreter.allocate_tensors()
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

# MQTT client with exponential backoff publish
client = mqtt.Client(client_id=DEVICE_ID)
client.connect(MQTT_BROKER, 1883, keepalive=60)

running = True
def handle_sig(signum, frame):
    global running; running = False
signal.signal(signal.SIGTERM, handle_sig); signal.signal(signal.SIGINT, handle_sig)

cap = cv2.VideoCapture(0)  # hardware camera

def infer_count(frame):
    # preprocess frame to model input size, run inference, and map to count
    inp = cv2.resize(frame, (input_details[0]['shape'][2], input_details[0]['shape'][1]))
    inp = inp.astype('uint8'); inp = inp.reshape(input_details[0]['shape'])
    interpreter.set_tensor(input_details[0]['index'], inp)
    interpreter.invoke()
    out = interpreter.get_tensor(output_details[0]['index'])
    # simplified mapping: sum detections above threshold
    return int((out > 0.5).sum())

def publish_with_backoff(payload):
    backoff = 1.0
    for _ in range(6):
        try:
            client.publish(MQTT_TOPIC, payload, qos=1)
            return True
        except Exception as e:
            logging.warning("Publish failed: %s, retry in %.1fs", e, backoff)
            time.sleep(backoff); backoff *= 2
    return False

last_publish = time.time()
buffered = []
while running:
    ret, frame = cap.read()
    if not ret:
        time.sleep(0.1); continue
    count = infer_count(frame)
    ts = int(time.time())
    conn.execute("INSERT INTO counts(ts,count) VALUES(?,?)", (ts, count))
    conn.commit()
    buffered.append({'ts': ts, 'count': count})

    if time.time() - last_publish >= PUBLISH_INTERVAL:
        payload = json.dumps({'device': DEVICE_ID, 'batch': buffered})
        ok = publish_with_backoff(payload)
        if not ok:
            logging.error("Publish failed after retries; retaining local records.")
        else:
            # delete persisted rows up to last ts after successful publish
            max_ts = buffered[-1]['ts']
            conn.execute("DELETE FROM counts WHERE ts <= ?", (max_ts,))
            conn.commit()
            buffered = []
        last_publish = time.time()

cap.release(); conn.close(); client.disconnect()