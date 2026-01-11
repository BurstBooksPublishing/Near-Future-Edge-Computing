import os
import numpy as np

def _secure_seed():
    # Use strong OS entropy; falls back to time if unavailable.
    try:
        return int.from_bytes(os.urandom(8), 'big')
    except Exception:
        return None

def laplace_mechanism(value, sensitivity, epsilon, size=None):
    """
    Add Laplace noise calibrated to sensitivity/epsilon.
    - value: scalar or NumPy array statistic.
    - sensitivity: global L1 sensitivity (float).
    - epsilon: privacy parameter (float > 0).
    - size: output shape for multiple samples (optional).
    Returns a noised copy; does not mutate inputs.
    """
    if epsilon <= 0:
        raise ValueError("epsilon must be > 0")
    scale = float(sensitivity) / float(epsilon)
    seed = _secure_seed()
    rng = np.random.default_rng(seed)  # seeded from OS entropy
    noise = rng.laplace(loc=0.0, scale=scale, size=size or np.shape(value))
    return np.asarray(value) + noise

# Example usage: one-noise count reporting on an edge gateway
if __name__ == "__main__":
    count = 42  # local aggregation result
    noised = laplace_mechanism(count, sensitivity=1.0, epsilon=0.5)
    print("private_count:", float(noised))