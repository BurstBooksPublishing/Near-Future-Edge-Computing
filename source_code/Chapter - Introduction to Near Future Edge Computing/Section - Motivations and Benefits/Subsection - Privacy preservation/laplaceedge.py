import math, secrets

# Add Laplace noise using secure RNG; safe for constrained edge nodes.
def laplace_noise(scale: float) -> float:
    # Inverse transform sampling for Laplace(0, scale)
    u = secrets.randbits(53) / (1 << 53) - 0.5  # uniform in (-0.5,0.5)
    return -scale * math.copysign(1.0, u) * math.log(1.0 - 2.0 * abs(u))

def privatize_scalar(value: float, sensitivity: float, epsilon: float) -> float:
    # Compute scale b = sensitivity / epsilon
    if epsilon <= 0:
        raise ValueError("epsilon must be positive")
    scale = sensitivity / epsilon
    return value + laplace_noise(scale)

# Example usage on sensor sampling loop
# sensor_reading = read_adc()   # hardware-specific call
# pub_value = privatize_scalar(sensor_reading, sensitivity=1.0, epsilon=0.5)
# publish(pub_value)            # e.g., MQTT via Eclipse Paho or AWS IoT Greengrass