"""Compute Erlang C waiting probability and mean delays (M/M/c)."""
from math import factorial, exp
from typing import Tuple

def erlang_c(lambda_rate: float, mu: float, c: int) -> Tuple[float, float, float]:
    """
    Return (P_wait, W_q, R) for given arrival rate, service rate, and servers.
    Raises ValueError if system unstable or invalid args.
    """
    if lambda_rate <= 0 or mu <= 0 or c < 1:
        raise ValueError("Invalid parameters")
    a = lambda_rate / mu
    if a >= c:
        raise ValueError("System unstable: arrival rate >= c * mu")
    # compute denominator sum
    sum_terms = sum((a**n) / factorial(n) for n in range(c))
    last = (a**c) / factorial(c) * (c / (c - a))
    p_wait = last / (sum_terms + last)
    w_q = p_wait / (c * mu - lambda_rate)  # mean wait in queue (seconds)
    r = w_q + 1.0 / mu                      # mean response time
    return p_wait, w_q, r

# Example usage (instrumented values): Jetson NX with 4 servers, mu=50/s, lambda=120/s
if __name__ == "__main__":
    p_wait, w_q, r = erlang_c(120.0, 50.0, 4)
    print(f"P_wait={p_wait:.4f}, W_q={w_q:.3f}s, R={r:.3f}s")