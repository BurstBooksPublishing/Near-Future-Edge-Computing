#!/usr/bin/env python3
# Production-ready edge client: profiles local estimator, queries QPU status,
# decides, submits OpenQASM job, polls, and fetches results with retries.

import time, json, os, requests
from typing import Dict

QPU_STATUS_URL = os.getenv("QPU_STATUS_URL")          # secure gateway URL
QPU_SUBMIT_URL  = os.getenv("QPU_SUBMIT_URL")
AUTH_TOKEN      = os.getenv("AUTH_TOKEN")             # JWT or mTLS handled by infra
MAX_RETRIES     = 5

def profile_local(task_payload: Dict) -> float:
    # Fast local estimator runtime (replace with accurate profiler)
    return 0.20  # seconds

def query_qpu_status() -> Dict:
    resp = requests.get(QPU_STATUS_URL, headers={"Authorization": f"Bearer {AUTH_TOKEN}"}, timeout=2.0)
    resp.raise_for_status()
    return resp.json()

def submit_quantum_job(circuit_openqasm: str, params: Dict) -> str:
    payload = {"qasm": circuit_openqasm, "params": params}
    for attempt in range(1, MAX_RETRIES+1):
        try:
            r = requests.post(QPU_SUBMIT_URL, json=payload,
                              headers={"Authorization": f"Bearer {AUTH_TOKEN}"}, timeout=5.0)
            r.raise_for_status()
            return r.json()["job_id"]
        except requests.RequestException:
            backoff = min(2**attempt * 0.1, 5.0)
            time.sleep(backoff)
    raise RuntimeError("Failed to submit quantum job after retries")

def poll_job(job_id: str, poll_url: str, timeout: float=60.0) -> Dict:
    deadline = time.time() + timeout
    while time.time() < deadline:
        r = requests.get(f"{poll_url}/{job_id}", headers={"Authorization": f"Bearer {AUTH_TOKEN}"}, timeout=2.0)
        r.raise_for_status()
        j = r.json()
        if j.get("status") in ("COMPLETED","FAILED"):
            return j
        time.sleep(0.5)
    raise TimeoutError("Quantum job polling timed out")

def decide_and_execute(task_payload: Dict):
    local_latency = profile_local(task_payload)
    status = query_qpu_status()
    offload_latency = status["avg_queue"] + status["exec_time"] + status["rtt"]/2.0
    # Simple decision: prefer offload only if faster and fidelity acceptable
    if offload_latency < local_latency and status["estimated_fidelity"] >= 0.7:
        job_id = submit_quantum_job(task_payload["qasm"], task_payload.get("params", {}))
        result = poll_job(job_id, QPU_SUBMIT_URL)
        return result
    else:
        # fallback to local classical processing
        return {"result": "local_solution", "latency": local_latency}

# Entrypoint for orchestrator
if __name__ == "__main__":
    # task would be provided by local pipeline in production
    sample_task = {"qasm": "OPENQASM 2.0; ...", "params": {}}
    print(decide_and_execute(sample_task))