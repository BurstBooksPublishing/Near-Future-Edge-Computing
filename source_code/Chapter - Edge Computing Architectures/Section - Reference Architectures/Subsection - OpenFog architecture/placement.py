#!/usr/bin/env python3
"""
Placement decision: returns 'local' or 'remote'.
Requires: requests, psutil (for CPU info), aioping (optional) or custom RTT probe.
"""
import time
import socket
import requests
import psutil

# CONFIG: adjust for your platform
REMOTE_TELEMETRY_URL = "https://fog-controller.example.local/api/v1/capacity"
REMOTE_HOST = "10.0.0.5"               # remote micro-DC endpoint
REMOTE_PORT = 443
S = 2e9                                # required cycles per request (example)

def measure_rtt(host, port, timeout=1.0):
    # TCP connect-based RTT probe (no ICMP required)
    start = time.time()
    try:
        s = socket.create_connection((host, port), timeout)
        s.close()
        return (time.time() - start)
    except Exception:
        return float('inf')

def get_remote_capacity():
    # Query remote telemetry for advertised cycles/second; fallback defaults used.
    try:
        r = requests.get(REMOTE_TELEMETRY_URL, timeout=1.0)
        r.raise_for_status()
        data = r.json()
        return float(data.get('cycles_per_second', 80e9)), float(data.get('arrival_rate', 1.0))
    except Exception:
        return 80e9, 1.0

def decide_placement():
    # Local compute estimate (cycles/sec available for our task)
    cpu_freq = psutil.cpu_freq().current * 1e6   # Hz -> cycles/s approx
    cores = psutil.cpu_count(logical=False) or 1
    C_local = cpu_freq * cores * 0.6             # conservative core utilization factor
    C_remote, arrival_rate = get_remote_capacity()
    rtt = measure_rtt(REMOTE_HOST, REMOTE_PORT)
    # compute rates and M/M/1 waiting time
    mu = C_remote / S
    lam = arrival_rate
    if mu <= lam:
        W_remote = float('inf')
    else:
        W_remote = 1.0 / (mu - lam)
    t_local = S / C_local
    t_remote_proc = S / C_remote
    t_remote_total = rtt + t_remote_proc + W_remote
    return 'local' if t_local <= t_remote_total else 'remote'

if __name__ == "__main__":
    placement = decide_placement()
    print(placement)