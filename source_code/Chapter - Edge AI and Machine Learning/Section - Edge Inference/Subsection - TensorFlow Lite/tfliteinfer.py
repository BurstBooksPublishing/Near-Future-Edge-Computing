#!/usr/bin/env python3
# Production-ready TFLite runner: loads delegate, measures latency, returns top-k.
import sys, time, numpy as np
from PIL import Image

# Use tensorflow.lite.Interpreter when TensorFlow is available, else tflite_runtime.
try:
    from tensorflow.lite import Interpreter, load_delegate
except Exception:
    from tflite_runtime.interpreter import Interpreter, load_delegate  # type: ignore

MODEL_PATH = sys.argv[1]  # e.g., 'model.tflite' or 'model_edgetpu.tflite'
IMAGE_PATH = sys.argv[2]  # input image
INPUT_SIZE = (224, 224)   # adapt to model
TOP_K = 3

# Try to attach Edge TPU delegate; fall back to CPU.
delegate = None
try:
    delegate = load_delegate('libedgetpu.so.1')  # Coral USB/PCIe
except Exception:
    delegate = None

interpreter = Interpreter(model_path=MODEL_PATH,
                          experimental_delegates=[delegate] if delegate else None)
interpreter.allocate_tensors()

# Input tensor details
input_idx = interpreter.get_input_details()[0]['index']
input_dtype = interpreter.get_input_details()[0]['dtype']

# Preprocess: resize, convert, normalize depending on dtype
img = Image.open(IMAGE_PATH).convert('RGB').resize(INPUT_SIZE)
arr = np.asarray(img, dtype=np.float32)

if input_dtype == np.uint8:
    # Typical int8 models expect 0-255 uint8
    input_data = arr.astype(np.uint8)
else:
    # Float models often expect [-1,1] or [0,1]; normalize to [0,1]
    input_data = (arr / 255.0).astype(np.float32)

input_data = np.expand_dims(input_data, axis=0)

# Warmup and timed runs
for _ in range(2):
    interpreter.set_tensor(input_idx, input_data)
    interpreter.invoke()

N = 50
t0 = time.perf_counter()
for _ in range(N):
    interpreter.set_tensor(input_idx, input_data)
    interpreter.invoke()
t1 = time.perf_counter()

latency_ms = (t1 - t0) / N * 1000.0
output_idx = interpreter.get_output_details()[0]['index']
output = interpreter.get_tensor(output_idx)[0]

# Postprocess: softmax and top-k
probs = np.exp(output - np.max(output))
probs /= probs.sum()
topk = probs.argsort()[-TOP_K:][::-1]

print(f"delegate={bool(delegate)} latency_ms={latency_ms:.2f}")
for i in topk:
    print(f"class={i} prob={probs[i]:.3f}")