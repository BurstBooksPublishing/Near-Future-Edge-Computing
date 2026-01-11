import numpy as np
import tensorflow as tf

# Load SavedModel; use a graph exported after training/optimization.
saved_model_dir = "/models/my_model_saved"
converter = tf.lite.TFLiteConverter.from_saved_model(saved_model_dir)

# Enable full integer quantization.
converter.optimizations = [tf.lite.Optimize.DEFAULT]
# Representative dataset generator yields batches of input arrays.
def representative_data_gen():
    for _ in range(100):                # calibration samples for edge sensors
        # Replace with real preprocessed sensor/frame arrays.
        sample = np.random.rand(1,224,224,3).astype(np.float32)
        yield [sample]

converter.representative_dataset = representative_data_gen
# Force int8 for both ops and IO to support accelerators like Coral Edge TPU.
converter.target_spec.supported_ops = [tf.lite.OpsSet.TFLITE_BUILTINS_INT8]
converter.inference_input_type = tf.int8
converter.inference_output_type = tf.int8

tflite_quant = converter.convert()
with open("/out/my_model_int8.tflite", "wb") as f:
    f.write(tflite_quant)
# Validate on-device using TFLite Interpreter and representative inputs.