import os, struct, math
from typing import Sequence
import numpy as np
import secrets

def compute_sigma(delta2: float, eps: float, delta: float) -> float:
    # Use standard sufficient bound for (eps, delta)-DP
    if not (0 < eps < 1):
        raise ValueError("eps should be in (0,1) for this bound")
    return (delta2 * math.sqrt(2.0 * math.log(1.25 / delta))) / eps

def secure_normal(shape, sigma: float) -> np.ndarray:
    # Sample from N(0, sigma^2) using secure entropy; transform via Box-Muller.
    # Use secrets.token_bytes as entropy source, convert to uint64 stream.
    n = int(np.prod(shape))
    out = np.empty(n, dtype=np.float64)
    i = 0
    while i < n:
        # draw 16 bytes of secure randomness -> two 64-bit integers
        rb = secrets.token_bytes(16)
        u1 = (struct.unpack(">Q", rb[:8])[0] + 1) / (2**64 + 1)
        u2 = (struct.unpack(">Q", rb[8:])[0] + 1) / (2**64 + 1)
        z0 = math.sqrt(-2.0 * math.log(u1)) * math.cos(2 * math.pi * u2)
        z1 = math.sqrt(-2.0 * math.log(u1)) * math.sin(2 * math.pi * u2)
        out[i] = z0 * sigma; i += 1
        if i < n:
            out[i] = z1 * sigma; i += 1
    return out.reshape(shape)

def gaussian_mechanism(q_value: np.ndarray, delta2: float, eps: float, delta: float) -> np.ndarray:
    sigma = compute_sigma(delta2, eps, delta)
    noise = secure_normal(q_value.shape, sigma)
    return q_value.astype(np.float64) + noise

# Example usage on a Raspberry Pi gateway: add noise to a scalar mean
# mean_val = np.array([sensor_mean])
# noisy = gaussian_mechanism(mean_val, delta2=0.08, eps=0.5, delta=1e-6)