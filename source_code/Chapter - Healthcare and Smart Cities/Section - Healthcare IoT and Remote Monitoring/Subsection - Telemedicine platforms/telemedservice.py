from fastapi import FastAPI, WebSocket
import onnxruntime as ort
import numpy as np
import requests
import ssl
import os

app = FastAPI()
# load model using optimized provider for device (CUDA/CPU/Neural Inference)
sess = ort.InferenceSession("arrhythmia_model.onnx", providers=["CUDAExecutionProvider","CPUExecutionProvider"])

EHR_ENDPOINT = os.environ.get("EHR_ENDPOINT")  # FHIR server URL
TLS_CERT = "/etc/keys/edge_cert.pem"            # TLS cert stored in TEE/trusted store

def infer(ecg_segment: np.ndarray) -> float:
    # preprocess: normalize and reshape to model input
    x = (ecg_segment - np.mean(ecg_segment)) / (np.std(ecg_segment) + 1e-6)
    x = x.astype(np.float32).reshape(1, 1, -1)
    out = sess.run(None, {"input": x})[0]
    return float(out[0, 0])  # probability of arrhythmia

def push_to_ehr(observation: dict):
    # secure TLS request; client cert if mutual TLS required
    resp = requests.post(EHR_ENDPOINT + "/Observation",
                         json=observation,
                         verify=TLS_CERT, timeout=5)
    resp.raise_for_status()

@app.websocket("/ws/ecg")
async def ecg_ws(ws: WebSocket):
    await ws.accept()
    try:
        while True:
            data = await ws.receive_bytes()  # binary PCM or float32 samples
            ecg = np.frombuffer(data, dtype=np.float32)
            score = infer(ecg)
            if score > 0.8:  # clinically tuned threshold
                obs = {
                    "resourceType": "Observation",
                    "code": {"text": "Arrhythmia detection"},
                    "valueQuantity": {"value": score},
                    "status": "final"
                }
                push_to_ehr(obs)  # non-blocking option: enqueue if needed
                await ws.send_json({"alert": True, "score": score})
            else:
                await ws.send_json({"alert": False})
    finally:
        await ws.close()