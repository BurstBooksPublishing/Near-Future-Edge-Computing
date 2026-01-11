import time, requests, numpy as np
# production: replace with real TFLite interpreter and hardware timers
LOCAL_SERVICE_RATE = 50.0  # in inferences/sec (measured)
NETWORK_RTT = 0.02          # measured round-trip seconds to MEC
CLOUD_SERVICE_RATE = 200.0  # inferences/sec at MEC
LATENCY_BUDGET = 0.05       # seconds per inference

def estimate_local_latency():
    # conservative per-inference processing time (s)
    return 1.0 / LOCAL_SERVICE_RATE

def estimate_remote_latency():
    # transmit + cloud service + receive (s)
    service = 1.0 / CLOUD_SERVICE_RATE
    return NETWORK_RTT + service

def decide_offload():
    l_local = estimate_local_latency()
    l_remote = estimate_remote_latency()
    # choose option that meets latency and minimizes energy proxy (not shown)
    if l_local <= LATENCY_BUDGET:
        return 'local'
    if l_remote <= LATENCY_BUDGET:
        return 'remote'
    # fallback: prefer local for resilience when both miss budget
    return 'local'

def run_inference(sample):
    choice = decide_offload()
    if choice == 'local':
        # call local TFLite interpreter (placeholder)
        time.sleep(estimate_local_latency())  # simulate work
        return {'result':'ok','source':'local'}
    else:
        # offload: send compressed payload to MEC endpoint
        resp = requests.post('https://mec.example/api/infer', json={'x':sample}, timeout=0.5)
        resp.raise_for_status()
        return resp.json()

# Example usage loop
if __name__ == "__main__":
    sample = np.zeros(128).tolist()
    out = run_inference(sample)
    print(out)  # integrate with telemetry exporter (Prometheus pushgateway)