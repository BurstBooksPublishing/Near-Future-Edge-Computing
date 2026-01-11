from pycoral.utils.edgetpu import make_interpreter  # create TPU interpreter
from pycoral.adapters import classify, common          # helpers for TFLite
from PIL import Image
import numpy as np

# load compiled model; file compiled by edgetpu_compiler
interpreter = make_interpreter("mobilenet_v2_1.0_224_quant_edgetpu.tflite")
interpreter.allocate_tensors()

# prepare input; resize and quantize per model spec
img = Image.open("frame.jpg").convert("RGB").resize((224,224))
common.set_input(interpreter, np.asarray(img))

interpreter.invoke()                                 # runs on Edge TPU
results = classify.get_classes(interpreter, top_k=5) # returns (id, score)
for obj in results:
    print(f"label:{obj.id} score:{obj.score:.3f}")  # minimal postprocess