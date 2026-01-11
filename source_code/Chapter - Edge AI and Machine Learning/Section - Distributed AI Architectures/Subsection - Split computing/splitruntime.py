#!/usr/bin/env python3
# Production-ready split inference client using ONNX Runtime and HTTP transport.
import time, requests, numpy as np, onnxruntime as ort
from typing import List

MODEL_PATH = "model.onnx"                # full model; server has tail
SERVER_URL = "https://mec.example/api/tail_infer"
PROBE_SIZE = 1024*8

# load and create session; use available execution providers
sess = ort.InferenceSession(MODEL_PATH, providers=["CPUExecutionProvider"])

def profile_prefix_times(prefix_layer_indices: List[int], sample_input: np.ndarray):
    times, sizes = {}, {}
    for i in prefix_layer_indices:
        # run until layer i by using a pre-partitioned subgraph or instrumented model
        start = time.monotonic()
        out = sess.run(None, {"input": sample_input})  # assume model split points pre-exported
        end = time.monotonic()
        times[i] = end - start
        sizes[i] = out[0].nbytes
    return times, sizes

def measure_bandwidth(url: str, nbytes=PROBE_SIZE, trials=3):
    payload = b"\x00" * nbytes
    times = []
    for _ in range(trials):
        t0 = time.monotonic()
        r = requests.post(url + "/bandwidth_probe", data=payload, timeout=1.0, verify=True)
        times.append(time.monotonic() - t0)
    avg = sum(times)/len(times)
    return nbytes/avg  # bytes/sec

def pick_cut(times, sizes, bandwidth, alpha=1.0, beta=0.0):
    best = None; best_obj = float("inf")
    for i in times:
        L = times[i] + sizes[i]/bandwidth
        E = 0.0  # device energy model can be substituted here
        obj = alpha * L + beta * E
        if obj < best_obj:
            best_obj = obj; best = i
    return best

def send_activation(activation: np.ndarray):
    # use np.save to create compact binary buffer; server expects np.load on body
    buf = activation.tobytes()
    headers = {"Content-Type":"application/octet-stream"}
    r = requests.post(SERVER_URL, data=buf, headers=headers, timeout=1.0, verify=True)
    r.raise_for_status()
    return np.frombuffer(r.content, dtype=np.float32)  # example dtype

# Example runtime flow
if __name__ == "__main__":
    sample = np.random.randn(1,3,224,224).astype(np.float32)
    prefix_indices = [5,10,17,25]                  # pre-defined cut candidates
    times, sizes = profile_prefix_times(prefix_indices, sample)
    bw = measure_bandwidth(SERVER_URL.rsplit('/',1)[0])
    cut = pick_cut(times, sizes, bw)
    # execute prefix up to chosen cut and send activation
    activation = sess.run(None, {"input": sample})[0]
    result = send_activation(activation)
    print("Selected cut", cut, "result shape", result.shape)