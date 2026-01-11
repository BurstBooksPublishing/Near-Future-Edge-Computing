#!/usr/bin/env python3
# Production-ready asyncio offload decision example.
import asyncio
import random
import time
from statistics import mean, median
import aiohttp

# Configuration
REMOTE_HEALTH_URL = "https://mec.example/api/v1/estimate"  # returns JSON {"proc_ms": int}
OFFLOAD_THRESHOLD_MARGIN_MS = 5  # safety margin
PROBE_INTERVAL = 1.0  # seconds
CIRCUIT_OPEN_MS = 5000  # open circuit for 5s after failures

async def probe_rtt(session, host, port=443, timeout=0.5):
    start = time.monotonic()
    try:
        async with session.head(host, timeout=timeout):
            pass
        rtt_ms = (time.monotonic() - start) * 1000
        return rtt_ms
    except Exception:
        return None

async def query_remote_proc(session):
    try:
        async with session.get(REMOTE_HEALTH_URL, timeout=0.5) as resp:
            data = await resp.json()
            return float(data.get("proc_ms", 0.0))
    except Exception:
        return None

def local_inference_latency_estimate():
    # Replace with actual timed inference call or maintained moving window.
    samples = [random.uniform(90, 130) for _ in range(5)]
    return median(samples)

async def decision_loop():
    circuit_open_until = 0
    async with aiohttp.ClientSession() as session:
        while True:
            now = time.monotonic() * 1000
            if now < circuit_open_until:
                # Circuit is open: force local processing
                use_offload = False
            else:
                rtt = await probe_rtt(session, REMOTE_HEALTH_URL)
                remote_proc = await query_remote_proc(session)
                local_proc = local_inference_latency_estimate()
                if rtt is None or remote_proc is None:
                    # Network or remote failed: open circuit briefly and local process
                    circuit_open_until = now + CIRCUIT_OPEN_MS
                    use_offload = False
                else:
                    total_remote = rtt + remote_proc + OFFLOAD_THRESHOLD_MARGIN_MS
                    use_offload = total_remote < local_proc
            # Act based on decision (placeholder calls)
            if use_offload:
                # invoke offload RPC (implement robust retry and backpressure)
                print("Offloading: expected remote total latency lower")
            else:
                # run local inference (ensure CPU/GPU affinity and priority)
                print("Running local inference")
            await asyncio.sleep(PROBE_INTERVAL + random.uniform(0, 0.1))

if __name__ == "__main__":
    asyncio.run(decision_loop())