import asyncio
import aiohttp
import psutil
import time

ASYNC_TIMEOUT = 1.0  # seconds
REMOTE_API = "https://edge-mec.example/v1/infer/ping"

async def probe_rtt(session: aiohttp.ClientSession) -> float:
    """Send a small ping to remote MEC service; return RTT in seconds."""
    start = time.perf_counter()
    try:
        async with session.get(REMOTE_API, timeout=ASYNC_TIMEOUT) as resp:
            await resp.read()
    except Exception:
        return float('inf')
    return time.perf_counter() - start

def local_compute_estimate(model_flops: float, cpu_freq_ghz: float) -> float:
    """Estimate local compute latency using simplified FLOPS model."""
    cpu_efficiency = 0.5  # conservative cycles-to-FLOP mapping
    cycles_needed = model_flops / cpu_efficiency
    cycles_per_sec = cpu_freq_ghz * 1e9 * psutil.cpu_count(logical=False)
    return cycles_needed / cycles_per_sec

async def should_offload(model_flops: float, remote_compute_s: float) -> bool:
    """Decision: offload if remote path likely faster."""
    async with aiohttp.ClientSession() as session:
        rtt = await probe_rtt(session)
    upload_download = rtt  # conservative one-way granularity accounted in RTT
    local_cpu_freq = psutil.cpu_freq().current / 1e3  # MHz->GHz
    t_local = local_compute_estimate(model_flops, local_cpu_freq)
    t_remote = upload_download + remote_compute_s
    # simple policy: offload only if 20% faster and network is reliable
    return (t_remote + 0.0) < 0.8 * t_local

# Example usage in asyncio loop:
# decision = await should_offload(model_flops=1e9, remote_compute_s=0.01)