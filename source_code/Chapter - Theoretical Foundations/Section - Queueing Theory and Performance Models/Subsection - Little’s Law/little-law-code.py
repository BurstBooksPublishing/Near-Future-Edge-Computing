import numpy as np
from typing import Sequence, Tuple

def littles_validate(arrival_ts: Sequence[float],
                     completion_ts: Sequence[float],
                     queue_samples: Sequence[Tuple[float,int]],
                     window: Tuple[float,float]) -> dict:
    """
    arrival_ts: ingress timestamps (monotonic seconds)
    completion_ts: egress timestamps (monotonic seconds), same items as arrivals
    queue_samples: list of (timestamp, sampled_queue_length)
    window: (t_start, t_end) for steady-state analysis
    Returns dict with lambda, W, L_sample, L_calc, relative_error.
    """
    t0, t1 = window
    # arrivals in window
    arrivals = [t for t in arrival_ts if t0 <= t <= t1]
    completions = [t for t in completion_ts if t0 <= t <= t1]
    # throughput estimate (events per second)
    lambda_hat = len(arrivals) / (t1 - t0)
    # mean sojourn time: match arrivals to completions by order
    # assume same number and order; in production use IDs and join.
    n = min(len(arrivals), len(completions))
    if n == 0:
        raise ValueError("No events in window")
    sojourns = np.array(completions[:n]) - np.array(arrivals[:n])
    W_hat = float(np.mean(sojourns))
    # sampled occupancy: weighted average over samples
    samples = [(ts, q) for ts, q in queue_samples if t0 <= ts <= t1]
    if not samples:
        L_sample = float('nan')
    else:
        times = np.array([s[0] for s in samples])
        qs = np.array([s[1] for s in samples], dtype=float)
        # compute area under sampled curve via trapezoid on sample times
        if len(times) == 1:
            L_sample = float(qs[0])
        else:
            area = np.trapz(qs, x=times)
            L_sample = float(area / (t1 - t0))
    L_calc = lambda_hat * W_hat
    rel_err = abs(L_calc - L_sample) / (L_sample + 1e-9) if not np.isnan(L_sample) else float('nan')
    return {"lambda": lambda_hat, "W": W_hat, "L_sample": L_sample,
            "L_calc": L_calc, "relative_error": rel_err}
# Example usage: integrate with Prometheus or local trace store to supply arrays.