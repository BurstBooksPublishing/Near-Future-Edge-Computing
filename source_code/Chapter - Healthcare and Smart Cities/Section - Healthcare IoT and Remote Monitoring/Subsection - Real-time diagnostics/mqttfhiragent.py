#!/usr/bin/env python3
# Minimal production edge agent: reconnecting MQTT client, TFLite inference, FHIR Observations POST.
import asyncio, ssl, json, time
import paho.mqtt.client as mqtt
import requests
from tflite_runtime.interpreter import Interpreter, load_delegate

MQTT_BROKER = "mqtt.local"
MQTT_TOPIC = "patient/+/imu"
FHIR_SERVER = "https://fhir.hospital.example/Observation"
FHIR_TOKEN = "REDACTED"  # use secure vault in production

# Load TFLite model, prefer Edge TPU delegate if available.
try:
    interpreter = Interpreter(model_path="fall_detector_quant.tflite",
                              experimental_delegates=[load_delegate("libedgetpu.so.1")])
except Exception:
    interpreter = Interpreter(model_path="fall_detector_quant.tflite")
interpreter.allocate_tensors()
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

def infer(frame):
    # frame: numpy array shaped to model input; ensure dtype matches quantization
    interpreter.set_tensor(input_details[0]['index'], frame)
    start = time.monotonic()
    interpreter.invoke()
    latency = (time.monotonic() - start)
    out = interpreter.get_tensor(output_details[0]['index'])
    return out, latency

def publish_fhir(patient_id, result, timestamp):
    obs = {
        "resourceType": "Observation",
        "status": "final",
        "category": [{"coding":[{"system":"http://terminology.hl7.org/CodeSystem/observation-category","code":"imaging"}]}],
        "code": {"coding":[{"system":"http://loinc.org","code":"XXX","display":"Fall detection"}]},
        "subject": {"reference": f"Patient/{patient_id}"},
        "effectiveDateTime": timestamp,
        "valueBoolean": bool(result)
    }
    headers = {"Authorization": f"Bearer {FHIR_TOKEN}", "Content-Type": "application/fhir+json"}
    r = requests.post(FHIR_SERVER, json=obs, headers=headers, timeout=5)
    r.raise_for_status()

# MQTT callbacks thread-safe push to asyncio queue
async_queue = asyncio.Queue()

def on_connect(client, userdata, flags, rc):
    client.subscribe(MQTT_TOPIC, qos=1)

def on_message(client, userdata, msg):
    # parse and forward to asyncio loop
    payload = msg.payload  # binary IMU samples or JSON from MCU
    asyncio.run_coroutine_threadsafe(async_queue.put((msg.topic, payload)), asyncio.get_event_loop())

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.tls_set_context(ssl.create_default_context())

async def main_loop():
    client.loop_start()
    while True:
        topic, payload = await async_queue.get()
        # parse patient id from topic 'patient/{id}/imu'
        patient_id = topic.split("/")[1]
        # Convert payload to model input (application-specific)
        frame = preprocess_payload(payload)  # implement robust parsing and validation
        out, infer_latency = infer(frame)
        is_fall = out[0,1] > 0.5  # model output probability for 'fall'
        # Local alerting policy
        if is_fall:
            timestamp = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
            publish_fhir(patient_id, True, timestamp)
        # metrics logging
        log_metrics(patient_id, infer_latency, is_fall)
asyncio.run(main_loop())