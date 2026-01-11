#!/usr/bin/env python3
"""Production-ready utility: minimal CPU cores required to meet XR latency budget.
Assumes Poisson arrivals and exponential service (M/M/c). Units: seconds.
"""
import math

def erlang_c_Pwait(lmbda, mu, c):
    # Prevent unrealistic load
    rho = lmbda / (c * mu)
    if rho >= 1.0:
        return 1.0
    a = lmbda / mu
    # compute sum_{n=0}^{c-1} a^n/n!
    sum_terms = sum((a**n) / math.factorial(n) for n in range(c))
    term_c = (a**c) / math.factorial(c)
    P0 = 1.0 / (sum_terms + term_c * (1.0 / (1.0 - rho)))
    # Erlang C
    Pwait = term_c * (1.0 / (1.0 - rho)) * P0
    return Pwait

def expected_queue_delay(lmbda, mu, c):
    Pwait = erlang_c_Pwait(lmbda, mu, c)
    denom = c * mu - lmbda
    if denom <= 0:
        return float('inf')
    Wq = Pwait / denom
    return Wq

def minimal_cores_for_budget(lmbda, service_time, comm_latency, others_latency, budget, max_cores=128):
    mu = 1.0 / service_time
    for c in range(1, max_cores+1):
        Wq = expected_queue_delay(lmbda, mu, c)
        server_latency = Wq + 1.0 / mu
        total = comm_latency + server_latency + others_latency
        if total <= budget:
            return c, total, Wq
    return None, None, None  # not achievable within max_cores

if __name__ == "__main__":
    # Example parameters for XR offload
    lambda_rate = 90.0           # frames per second
    service_time = 0.004         # 4 ms per frame on one core
    comm_latency = 0.005         # 5 ms network RTT (one-way budgeted)
    others_latency = 0.006       # sensing + local + display
    budget = 0.020               # 20 ms motion-to-photon budget

    cores, total_latency, queue_delay = minimal_cores_for_budget(
        lambda_rate, service_time, comm_latency, others_latency, budget
    )
    # Print succinct engineering result
    print(f"required_cores={cores}, total_latency_s={total_latency:.4f}, queue_delay_s={queue_delay:.6f}")