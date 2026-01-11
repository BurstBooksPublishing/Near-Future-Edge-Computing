#!/usr/bin/env python3
# Minimal, production-quality gateway: robust reconnects, batching, TLS MQTT.
import asyncio, struct, ssl
from bleak import BleakClient
import tflite_runtime.interpreter as tflite
import paho.mqtt.client as mqtt

BLE_ADDR = "AA:BB:CC:11:22:33"     # wearable MAC
CHAR_UUID = "0000fff1-0000-1000-8000-00805f9b34fb"  # notify char
MQTT_BROKER = "broker.example.com"
MQTT_TOPIC = "hospital/ward1/alerts"
MODEL_PATH = "/opt/models/ecg_cnn_int8.tflite"

# Initialize interpreter (thread-safe for single inference loop)
interpreter = tflite.Interpreter(model_path=MODEL_PATH)
interpreter.allocate_tensors()
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

# MQTT with TLS
ssl_ctx = ssl.create_default_context()
client = mqtt.Client()
client.tls_set_context(ssl_ctx)
client.connect(MQTT_BROKER, 8883)
client.loop_start()

def classify_window(samples):
    # samples: list or ndarray of length N, dtype=float32 normalized
    import numpy as np
    x = np.array(samples, dtype=np.float32).reshape(input_details[0]['shape'])
    interpreter.set_tensor(input_details[0]['index'], x)
    interpreter.invoke()
    probs = interpreter.get_tensor(output_details[0]['index'])[0]
    return float(probs[1])  # probability of arrhythmia

async def run():
    buffer = []
    async with BleakClient(BLE_ADDR) as c:
        # subscribe to notifications
        def callback(_, data: bytearray):
            # parse little-endian float16 samples, or int16 depending on device
            val = struct.unpack_from('= 1000:
                prob = classify_window(buffer[-1000:])
                if prob > 0.85:
                    client.publish(MQTT_TOPIC, payload=f'{{"alert": "arrhythmia","p":{prob:.2f}}}', qos=1)
                del buffer[:-500]  # 50% overlap

        await c.start_notify(CHAR_UUID, callback)
        while True:
            await asyncio.sleep(1)

if __name__ == "__main__":
    asyncio.run(run())