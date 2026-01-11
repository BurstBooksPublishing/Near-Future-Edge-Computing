#!/usr/bin/env python3
# Production-ready: requires psutil, requests; run as non-root for ICMP library alternatives.
import time, subprocess, psutil, requests
from typing import Dict

# Tunable weights (set per SLA)
ALPHA, BETA, GAMMA = 0.7, 0.25, 0.05

# Simple energy model parameters (Joules per inference)
ENERGY_LOCAL = 7.0  # watts * inference_time_sec
ENERGY_NET_TX = 0.5

# Remote service endpoint and expected remote proc time in seconds
REMOTE_URL = "https://fog.example.local/infer"
REMOTE_PROC_SEC = 0.02  # 20 ms

def measure_rtt(host: str, timeout: float = 1.0) -> float:
    # Use system ping; replace with aioping or application probe if needed.
    try:
        out = subprocess.check_output(["ping","-c","1","-W",str(int(timeout*1000)), host],
                                      stderr=subprocess.DEVNULL).decode()
        # parse time=xx ms
        for token in out.split():
            if token.startswith("time="):
                return float(token.split("=")[1].replace("ms",""))/1000.0
    except Exception:
        return float('inf')

def local_inference_time() -> float:
    # Heuristic: infer_time = baseline * (1 + cpu_load/100)
    baseline = 0.04  # 40 ms baseline
    load = psutil.cpu_percent(interval=0.1)
    return baseline * (1.0 + load/100.0)

def score_local() -> Dict[str,float]:
    t = local_inference_time()
    energy = ENERGY_LOCAL * t
    bandwidth = 0.0
    score = ALPHA*t + BETA*energy + GAMMA*bandwidth
    return {"time": t, "energy": energy, "score": score}

def score_remote(host: str) -> Dict[str,float]:
    rtt = measure_rtt(host)
    if rtt == float('inf'):
        return {"time": float('inf'), "energy": float('inf'), "score": float('inf')}
    t = rtt + REMOTE_PROC_SEC
    energy = ENERGY_NET_TX  # device transmit energy approximation
    bandwidth = 1.0  # normalized
    score = ALPHA*t + BETA*energy + GAMMA*bandwidth
    return {"time": t, "energy": energy, "score": score}

def decide_offload(host: str) -> bool:
    local = score_local()
    remote = score_remote(host)
    return remote["score"] < local["score"]

if __name__ == "__main__":
    host = "fog.example.local"
    decision = decide_offload(host)
    print("OFFLOAD" if decision else "LOCAL")