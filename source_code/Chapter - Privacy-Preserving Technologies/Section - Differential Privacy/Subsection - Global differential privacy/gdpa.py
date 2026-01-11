import os, secrets
import numpy as np

# Cryptographic seed for RNG (use os.urandom or hardware RNG where available).
_seed = secrets.randbits(64)
_rng = np.random.default_rng(_seed)

def compute_sigma(delta: float, epsilon: float, sensitivity: float) -> float:
    # Gaussian mechanism scale per Dwork & Roth.
    if epsilon <= 0 or delta <= 0:
        raise ValueError("epsilon and delta must be positive")
    return (sensitivity / epsilon) * np.sqrt(2.0 * np.log(1.25 / delta))

def clip_and_aggregate(contributions: np.ndarray, clip_bound: float) -> float:
    # contributions: array of per-device scalar contributions.
    clipped = np.clip(contributions, -clip_bound, clip_bound)
    return float(np.sum(clipped))

def add_gaussian_noise(value: float, sigma: float) -> float:
    noise = _rng.normal(loc=0.0, scale=sigma)
    return value + float(noise)

# Example usage in an edge aggregator loop.
def dp_aggregate(contributions: np.ndarray, clip_bound: float,
                 epsilon: float, delta: float) -> (float,float):
    sensitivity = 2.0 * clip_bound  # worst-case L2 change for one device removed/added.
    sigma = compute_sigma(delta, epsilon, sensitivity)
    aggregated = clip_and_aggregate(contributions, clip_bound)
    noisy = add_gaussian_noise(aggregated, sigma)
    return noisy, sigma