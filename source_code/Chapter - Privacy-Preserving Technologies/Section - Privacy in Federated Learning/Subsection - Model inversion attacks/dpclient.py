import grpc
import numpy as np
import tensorflow as tf
# TensorFlow Privacy utilities
from tensorflow_privacy.privacy.optimizers.dp_optimizer_keras import \
    DPKerasAdamOptimizer

# Model and data prepared on edge device
model = tf.keras.models.load_model('local_model.h5')  # small CNN on device
optimizer = DPKerasAdamOptimizer(
    l2_norm_clip=1.0,            # clip norm C
    noise_multiplier=0.8,       # sigma relative to C
    num_microbatches=1,
    learning_rate=1e-3)

# Train one local epoch with DP optimizer (computes clipped noisy grads)
model.compile(optimizer=optimizer, loss='sparse_categorical_crossentropy')
model.fit(local_dataset, epochs=1, verbose=0)

# Serialize model weight deltas to send (server expects aggregated updates)
deltas = [w.numpy() - base.numpy() for w, base in zip(model.weights, base_weights)]
payload = np.concatenate([d.ravel() for d in deltas]).astype(np.float32).tobytes()

# Send via secure channel (gRPC over TLS) to aggregator
with grpc.secure_channel('aggregator.example.com:443', grpc.ssl_channel_credentials()) as ch:
    stub = AggregationStub(ch)  # generated from proto; server implements aggregator API
    req = UpdateRequest(client_id='edge-gateway-42', payload=payload)
    resp = stub.SubmitUpdate(req, timeout=10.0)  # short timeout for edge scenario
    # server returns acknowledgment or next round token