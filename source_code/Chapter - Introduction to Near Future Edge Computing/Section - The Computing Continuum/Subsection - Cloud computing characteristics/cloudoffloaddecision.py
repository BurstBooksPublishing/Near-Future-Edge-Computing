#!/usr/bin/env python3
"""Probe cloud endpoint, measure RTT and throughput, compute offload decision."""

import time, logging, requests
from statistics import mean

# Tunable weights for cost model
W_LATENCY = 0.7
W_ENERGY = 0.2
W_MONEY = 0.1

# Example energy model: joules per byte transmitted (device-specific)
J_PER_BYTE = 1e-6

# Probe settings
PROBE_URL = "https://example-cloud-endpoint/ping"   # lightweight endpoint
DOWNLOAD_URL = "https://example-cloud-endpoint/test-object"  # small object for throughput

session = requests.Session()
session.headers.update({"User-Agent": "edge-probe/1.0"})

def measure_rtt(n=3, timeout=2.0):
    rtts = []
    for _ in range(n):
        t0 = time.time()
        try:
            r = session.get(PROBE_URL, timeout=timeout)
            r.raise_for_status()
            rtts.append(time.time() - t0)
        except requests.RequestException:
            rtts.append(timeout)
    return mean(rtts)

def measure_upload_time(payload_size_bytes, timeout=5.0):
    # POST small payload to measure uplink effective time
    data = b"x" * payload_size_bytes
    t0 = time.time()
    try:
        r = session.post(PROBE_URL, data=data, timeout=timeout)
        r.raise_for_status()
        return time.time() - t0
    except requests.RequestException:
        return timeout

def measure_download_bandwidth(timeout=5.0):
    t0 = time.time()
    try:
        r = session.get(DOWNLOAD_URL, stream=True, timeout=timeout)
        total = 0
        for chunk in r.iter_content(8192):
            total += len(chunk)
        dt = time.time() - t0
        return total / dt if dt > 0 else 0.0
    except requests.RequestException:
        return 0.0

def offload_decision(payload_size, result_size, proc_time_cloud, cost_per_request):
    rtt = measure_rtt()
    up_time = measure_upload_time(payload_size)
    bw_up = payload_size / up_time if up_time > 0 else 0.0
    down_bw = measure_download_bandwidth()
    L_cloud = rtt + (payload_size / bw_up if bw_up>0 else float('inf')) + proc_time_cloud + (result_size / down_bw if down_bw>0 else 0.0)
    energy_tx = payload_size * J_PER_BYTE
    cost = W_LATENCY * L_cloud + W_ENERGY * energy_tx + W_MONEY * cost_per_request
    # Decision threshold: prefer offload if cost lower than local processing baseline
    local_processing_time = 0.3  # seconds: device-specific baseline
    local_energy = 0.05          # joules: device-specific baseline
    baseline_cost = W_LATENCY*local_processing_time + W_ENERGY*local_energy
    logging.info("L_cloud=%.3fs cost=%.6f baseline=%.6f", L_cloud, cost, baseline_cost)
    return cost < baseline_cost, {"L_cloud": L_cloud, "cost": cost, "baseline": baseline_cost}

# Example usage
if __name__ == "__main__":
    decision, meta = offload_decision(250_000, 2_000, 0.05, 0.0005)
    print("Offload?" , decision, meta)