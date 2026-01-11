import time
import numpy as np
import tensorflow as tf
# Convert Keras model to TFLite int8 with representative dataset
def representative_data_gen(dataset, input_shape, samples=100):
    for i, x in enumerate(dataset):
        if i >= samples: break
        yield [np.expand_dims(x.astype(np.float32), axis=0)]

model = tf.keras.models.load_model('teacher_pruned_finetuned.h5')  # trained/finetuned model
converter = tf.lite.TFLiteConverter.from_keras_model(model)
converter.optimizations = [tf.lite.Optimize.DEFAULT]
# representative_dataset must yield float32 inputs in training scale
converter.representative_dataset = lambda: representative_data_gen(my_dataset, model.input_shape[1:])
converter.target_spec.supported_ops = [tf.lite.OpsSet.TFLITE_BUILTINS_INT8]
converter.inference_input_type = tf.uint8   # or tf.int8 depending on delegate
converter.inference_output_type = tf.uint8
tflite_model = converter.convert()
open('model_int8.tflite','wb').write(tflite_model)

# Benchmark helper using tflite_runtime for small edge images
from tflite_runtime.interpreter import Interpreter, load_delegate
def benchmark(model_path, use_edgetpu=False, input_shape=(224,224,3), runs=200):
    delegates = [load_delegate('libedgetpu.so.1')] if use_edgetpu else None
    interp = Interpreter(model_path, experimental_delegates=delegates)
    interp.allocate_tensors()
    input_index = interp.get_input_details()[0]['index']
    dummy = np.random.randint(0,255,size=(1,*input_shape),dtype=np.uint8)
    # warmup
    for _ in range(10):
        interp.set_tensor(input_index, dummy); interp.invoke()
    # timed runs
    t0 = time.perf_counter()
    for _ in range(runs):
        interp.set_tensor(input_index, dummy); interp.invoke()
    t1 = time.perf_counter()
    print(f"{'EdgeTPU' if use_edgetpu else 'CPU'} avg latency: {(t1-t0)/runs*1000:.2f} ms")
# Example usage:
# benchmark('model_int8.tflite', use_edgetpu=False)
# benchmark('model_int8.tflite', use_edgetpu=True)