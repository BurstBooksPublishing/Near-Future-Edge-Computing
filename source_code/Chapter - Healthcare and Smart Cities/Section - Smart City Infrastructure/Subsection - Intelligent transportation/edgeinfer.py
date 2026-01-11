#!/usr/bin/env python3
import time, json, requests, logging
import paho.mqtt.client as mqtt
import onnxruntime as ort
import cv2
from tenacity import retry, wait_exponential, stop_after_attempt

# Configuration (use secure storage in production)
MQTT_BROKER = "mec-broker.local"
MQTT_TOPIC = "intersection/rsu/detections"
MEC_INFER_URL = "https://mec.example/api/v1/infer"
MODEL_PATH = "/opt/models/vehicle_detect.onnx"

# Initialize ONNX runtime with CUDA/TensorRT provider if available
providers = ['CUDAExecutionProvider', 'CPUExecutionProvider']
sess = ort.InferenceSession(MODEL_PATH, providers=providers)

mqttc = mqtt.Client()
mqttc.tls_set()                  # TLS config in production
mqttc.username_pw_set("edge", "secret")
mqttc.connect(MQTT_BROKER, 8883)

def preprocess(frame):
    # Resize, normalize; keep deterministic timing
    img = cv2.resize(frame, (640, 480))
    img = img.astype('float32') / 255.0
    return img.transpose(2,0,1)[None,...]

def infer_local(img_tensor):
    inputs = {sess.get_inputs()[0].name: img_tensor}
    out = sess.run(None, inputs)
    return out[0]  # model-specific postprocess required

@retry(wait=wait_exponential(multiplier=0.5, min=1, max=8),
       stop=stop_after_attempt(5))
def offload_to_mec(frame_bytes, metadata):
    # Secure POST with timeout; retries via tenacity
    resp = requests.post(MEC_INFER_URL, files={'frame': frame_bytes},
                         data={'meta': json.dumps(metadata)},
                         timeout=1.5, verify="/etc/ssl/certs/ca.pem")
    resp.raise_for_status()
    return resp.json()

def publish_detection(det):
    mqttc.publish(MQTT_TOPIC, json.dumps(det), qos=1)

def main_loop():
    cap = cv2.VideoCapture(0)  # PoE camera via V4L2 bridge
    while True:
        ret, frame = cap.read()
        if not ret:
            time.sleep(0.01); continue
        tensor = preprocess(frame)
        t0 = time.time()
        dets = infer_local(tensor)
        latency = time.time() - t0
        if latency > 0.08 or low_confidence(dets):
            # Serialize and offload frame to MEC
            _, jpg = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
            try:
                mec_resp = offload_to_mec(jpg.tobytes(), {"latency": latency})
                publish_detection(mec_resp)
                continue
            except Exception as e:
                logging.warning("MEC offload failed: %s", e)
        publish_detection({"local": dets.tolist(), "latency": latency})

if __name__ == "__main__":
    main_loop()