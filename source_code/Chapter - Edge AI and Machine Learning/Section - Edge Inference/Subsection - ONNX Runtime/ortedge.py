import onnxruntime as ort
import numpy as np
import time

# Load ONNX model and configure session options for edge
model_path = "resnet18.onnx"
so = ort.SessionOptions()
so.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL  # fuse/const-fold
so.intra_op_num_threads = 2  # tune to CPU cores and affinity
so.inter_op_num_threads = 1

# Provider preference: try OpenVINO, fallback to CPU
providers = []
try:
    # on Raspberry Pi with OpenVINO runtime installed
    providers = ['OpenVINOExecutionProvider', 'CPUExecutionProvider']
except Exception:
    providers = ['CPUExecutionProvider']

sess = ort.InferenceSession(model_path, sess_options=so, providers=providers)

# Prepare input (example preprocessed image)
input_name = sess.get_inputs()[0].name
dummy_input = np.random.rand(1, 3, 224, 224).astype(np.float32)

# Warm-up runs to populate caches and compile kernels on EPs
for _ in range(5):
    _ = sess.run(None, {input_name: dummy_input})

# Measure steady-state latency
n = 50
t0 = time.perf_counter()
for _ in range(n):
    _ = sess.run(None, {input_name: dummy_input})
t_mean_ms = (time.perf_counter() - t0) / n * 1000.0
print(f"Mean inference latency: {t_mean_ms:.2f} ms")