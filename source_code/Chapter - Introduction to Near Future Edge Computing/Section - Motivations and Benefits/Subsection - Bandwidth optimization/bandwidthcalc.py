#!/usr/bin/env python3
"""Compute effective bandwidth for sensor fleets with batching and headers."""
from math import ceil
from typing import NamedTuple

class Params(NamedTuple):
    N: int          # number of sensors
    f: float        # samples per second per sensor
    S: int          # bits per raw sample
    rho: float      # compression ratio (0 float:
    """Return bandwidth in bits per second given Params."""
    compressed_sample = p.rho * p.S
    # average packets per sample (ceil to represent discrete packets)
    pkts_per_sample = ceil(compressed_sample / p.P)
    # total bits per transmitted sample including headers
    bits_per_sample = pkts_per_sample * (p.P + p.H) * p.alpha
    return p.N * p.f * bits_per_sample

if __name__ == "__main__":
    # Example: 40 sensors, 10 windows/s, raw 48 bits/sample, 128-bit summary
    params = Params(N=40, f=10.0, S=48*4000//100, rho=128/(48*4000//100),
                    P=1024*8, H=64*8, alpha=0.9)
    print(f"Effective bandwidth: {bandwidth(params)/1e6:.3f} Mbps")
    # Use this output to size 5G slice or Ethernet port and set QoS policies.