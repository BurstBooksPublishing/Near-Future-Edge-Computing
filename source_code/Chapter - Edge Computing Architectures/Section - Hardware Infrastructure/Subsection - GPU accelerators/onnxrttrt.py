import onnxruntime as ort
import numpy as np

# Configure session options to use TensorRT then CUDA as fallback
so = ort.SessionOptions()
so.intra_op_num_threads = 2
so.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL

providers = ['TensorrtExecutionProvider', 'CUDAExecutionProvider', 'CPUExecutionProvider']
# Create session; provide device memory limit via session options if needed
sess = ort.InferenceSession('model.onnx', sess_options=so, providers=providers)

# Prepare input; use pinned host memory for zero-copy transfer where supported
input_name = sess.get_inputs()[0].name
def preprocess(image):  # image: HWC uint8 -> NCHW float32 normalized
    x = image.astype(np.float32) / 255.0
    x = np.transpose(x, (2,0,1))[None, ...]
    return x

# Inference loop with asynchronous batching strategy
image = np.random.randint(0,255,(224,224,3),dtype=np.uint8)
input_tensor = preprocess(image)
# Run (synchronous here; for production consider CUDA streams or multiple threads)
outputs = sess.run(None, {input_name: input_tensor})
# Post-process outputs as required