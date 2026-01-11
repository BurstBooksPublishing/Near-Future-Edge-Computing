import os, numpy as np, onnxruntime as ort
# Ensure TensorRT and CUDA EPs available on Jetson system
model_path = "/opt/models/my_model.onnx"
opts = ort.SessionOptions()
opts.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
opts.intra_op_num_threads = 2  # tune for platform
# Reduce memory arena fragmentation in constrained environments
opts.enable_cpu_mem_arena = True

# Preferred execution providers: TensorRT -> CUDA -> CPU
eps = ['TensorrtExecutionProvider', 'CUDAExecutionProvider', 'CPUExecutionProvider']
sess = ort.InferenceSession(model_path, sess_options=opts, providers=eps)

# Prepare input with explicit dtype and pinned memory if available
def preprocess(frame):
    # model expects NHWC float32 normalized to [0,1]
    img = frame.astype(np.float32) / 255.0
    return np.expand_dims(img, axis=0)

# Single-shot synchronous inference with minimal copies
frame = np.zeros((224,224,3), dtype=np.uint8)  # placeholder from camera
inp = preprocess(frame)
input_name = sess.get_inputs()[0].name
try:
    outputs = sess.run(None, {input_name: inp})
except Exception as e:
    # Fallback and logging for production resilience
    print("Inference error:", e)
    outputs = sess.run(None, {input_name: inp}, run_options=ort.RunOptions())
# Postprocess as needed
pred = outputs[0]