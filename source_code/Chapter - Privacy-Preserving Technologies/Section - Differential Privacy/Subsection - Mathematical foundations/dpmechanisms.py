import os, secrets, numpy as np
# Securely seed numpy RNG using secrets (suitable for constrained Linux-based edges)
_seed = secrets.randbits(64)
_rng = np.random.default_rng(_seed)

def clip_values(x, low, high):
    # Clip per-device contributions to control sensitivity
    return np.minimum(np.maximum(x, low), high)

def laplace_mechanism_scalar(value, epsilon, sensitivity):
    # Return DP-noised scalar using Laplace mechanism
    scale = sensitivity / float(epsilon)
    noise = _rng.laplace(0.0, scale)
    return value + noise

def gaussian_mechanism_scalar(value, epsilon, delta, sensitivity):
    # Return DP-noised scalar using Gaussian mechanism (per Eq. (3))
    sigma = sensitivity * np.sqrt(2.0 * np.log(1.25 / delta)) / float(epsilon)
    noise = _rng.normal(0.0, sigma)
    return value + noise

def aggregate_mean_with_global_dp(values, epsilon, clip_low, clip_high):
    # values: numpy array of sensor readings
    x = clip_values(values, clip_low, clip_high)
    N = len(x)
    sensitivity = (clip_high - clip_low) / float(N)
    noisy_mean = laplace_mechanism_scalar(x.mean(), epsilon, sensitivity)
    return noisy_mean