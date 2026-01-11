#!/usr/bin/env python3
# Production-ready asyncio service: measures metrics and decides local vs remote validation.
import asyncio
import time
import logging
from typing import Dict
import psutil  # system metrics; apt-get install python3-psutil
import aiohttp

LOG = logging.getLogger("edge_decider")
logging.basicConfig(level=logging.INFO)

# Operator-configurable weights (tune per deployment)
WEIGHTS = {"alpha": 0.5, "beta": 0.3, "gamma": 0.15, "delta": 0.05}
REMOTE_VALIDATOR = "https://fog-validator.example.local/validate"
MEASURE_INTERVAL = 2.0  # seconds

async def measure_rtt(session: aiohttp.ClientSession) -> float:
    start = time.time()
    try:
        async with session.get(REMOTE_VALIDATOR, timeout=1.0) as resp:
            await resp.read()
    except Exception:
        return float("inf")
    return (time.time() - start)

def normalize(value: float, bounds: Dict[str, float]) -> float:
    lo, hi = bounds["min"], bounds["max"]
    if value <= lo: return 0.0
    if value >= hi: return 1.0
    return (value - lo) / (hi - lo)

def compute_J(latency: float, energy: float, security: float, privacy: float) -> float:
    # Normalize using deployment bounds
    nL = normalize(latency, {"min":0.0, "max":2.0})
    nE = normalize(energy, {"min":0.0, "max":10.0})
    nS = normalize(security, {"min":0.0, "max":1.0})
    nP = normalize(privacy, {"min":0.0, "max":1.0})
    return WEIGHTS["alpha"]*nL + WEIGHTS["beta"]*nE - WEIGHTS["gamma"]*nS + WEIGHTS["delta"]*nP

async def decide_and_act(tx_payload: bytes):
    async with aiohttp.ClientSession() as session:
        rtt = await measure_rtt(session)
        cpu_pct = psutil.cpu_percent(interval=None) / 100.0
        battery = psutil.sensors_battery()
        battery_pct = battery.percent/100.0 if battery else 1.0
        # simple models
        latency_local = 0.05 + 0.20*c=0.0  # placeholder: signature + consensus estimate
        # realistic local latency estimate (seconds)
        latency_local = 0.05 + 0.20 * (1.0 + cpu_pct)
        energy_local = 3.0 * latency_local  # Watts * seconds -> Joules proxy
        security_local = 0.99  # local attestation available
        privacy_local = 0.0
        latency_remote = rtt + 0.04  # tx RTT + fog processing
        energy_remote = 0.5 * latency_remote
        security_remote = 0.95
        privacy_remote = 0.2
        J_local = compute_J(latency_local, energy_local, security_local, privacy_local)
        J_remote = compute_J(latency_remote, energy_remote, security_remote, privacy_remote)
        if J_local <= J_remote or battery_pct < 0.2:
            LOG.info("Validating locally")
            # call into local signer/ledger client -- placeholder function
            await local_validate_and_commit(tx_payload)
        else:
            LOG.info("Forwarding to remote validator")
            await forward_with_retry(session, tx_payload)

async def forward_with_retry(session, payload, retries=3):
    backoff = 0.5
    for attempt in range(retries):
        try:
            async with session.post(REMOTE_VALIDATOR, data=payload, timeout=5) as resp:
                resp.raise_for_status()
                return await resp.json()
        except Exception as exc:
            LOG.warning("Forward attempt %d failed: %s", attempt+1, exc)
            await asyncio.sleep(backoff)
            backoff *= 2
    raise RuntimeError("Forwarding failed after retries")

async def local_validate_and_commit(payload: bytes):
    # Integrate with local ledger client (tendermint or fabric SDK)
    await asyncio.sleep(0.05)  # simulate work; replace with real SDK calls
    return {"status":"committed"}

# Entry loop omitted for brevity; integrate with device message bus (MQTT/gRPC).