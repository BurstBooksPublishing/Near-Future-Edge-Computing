# Compressor for PyTorch models: top-k + error feedback + MQTT send
import numpy as np
import zlib
import json
import torch
import paho.mqtt.client as mqtt

class TopKCompressor:
    def __init__(self, k, dtype=np.int8):
        self.k = int(k)
        self.dtype = dtype
        self.error = None

    def _flatten_params(self, model):
        vec = []
        for p in model.parameters():
            if p.grad is None: continue
            vec.append(p.grad.detach().cpu().view(-1).numpy())
        return np.concatenate(vec).astype(np.float32)

    def compress(self, model):
        grad = self._flatten_params(model)
        if self.error is None:
            self.error = np.zeros_like(grad, dtype=np.float32)
        corrected = grad + self.error
        # select top-k indices
        k = min(self.k, corrected.size)
        idx = np.argpartition(np.abs(corrected), -k)[-k:]
        vals = corrected[idx]
        # quantize values to specified dtype
        scale = np.max(np.abs(vals)) or 1.0
        q = (vals / scale * np.iinfo(self.dtype).max).astype(self.dtype)
        # store error feedback (residual)
        recon = (q.astype(np.float32) / np.iinfo(self.dtype).max) * scale
        residual = corrected
        residual[idx] = corrected[idx] - recon
        self.error = residual
        payload = {
            "idx": idx.tolist(),            # indices as list
            "vals": q.tobytes(),           # raw bytes of quantized values
            "dtype": str(self.dtype),
            "scale": float(scale)
        }
        b = zlib.compress(json.dumps(payload).encode('latin1'))
        return b

    def decompress(self, blob, model_shape):
        raw = zlib.decompress(blob).decode('latin1')
        payload = json.loads(raw)
        idx = np.array(payload['idx'], dtype=np.int64)
        q = np.frombuffer(payload['vals'].encode('latin1'), dtype=np.dtype(payload['dtype']))
        vals = (q.astype(np.float32) / np.iinfo(np.dtype(payload['dtype']).type).max) * payload['scale']
        grad = np.zeros(model_shape, dtype=np.float32)
        grad[idx] = vals
        return grad

# MQTT transport (blocking publish with QoS 1)
def publish_update(broker_url, topic, payload_bytes, client_id=None):
    client = mqtt.Client(client_id)
    client.tls_set()  # use TLS for production
    client.connect(broker_url, port=8883, keepalive=60)
    client.loop_start()
    client.publish(topic, payload_bytes, qos=1)
    client.loop_stop()
    client.disconnect()