#!/usr/bin/env python3
"""Compute minimal server count for target latency using Erlang C."""

import math
from typing import Tuple

def erlang_c(c: int, rho: float) -> float:
    """Return Erlang C probability of wait for given servers c and load rho."""
    if c <= 0:
        raise ValueError("c must be positive")
    # compute Poisson traffic term
    inv_fact = sum((rho**k) / math.factorial(k) for k in range(c))
    last = (rho**c) / (math.factorial(c) * (1 - rho / c))
    p0 = 1.0 / (inv_fact + last)
    ec = last * p0
    return ec

def mean_response_time(lambda_rate: float, mu: float, c: int) -> float:
    """Compute mean response time R (seconds) for M/M/c queue."""
    rho = lambda_rate / (c * mu)
    if rho >= 1.0:
        return float('inf')  # unstable
    ec = erlang_c(c, lambda_rate / mu)
    wait = ec / (c * mu - lambda_rate)
    return wait + 1.0 / mu

def required_servers(lambda_rate: float, mu: float, r_target: float,
                     c_max: int = 256) -> Tuple[int, float]:
    """Return minimal c and achieved R. Raises if not feasible up to c_max."""
    for c in range(1, c_max + 1):
        r = mean_response_time(lambda_rate, mu, c)
        if r <= r_target:
            return c, r
    raise RuntimeError("Target latency not achievable within c_max")

if __name__ == "__main__":
    # Example: 150 req/s, 20 req/s per server, target 0.05 s
    c_required, achieved = required_servers(150.0, 20.0, 0.05)
    print(f"Required servers: {c_required}, Achieved R: {achieved:.4f}s")
    # Integrate with inventory to map c to actual node types and power budget.