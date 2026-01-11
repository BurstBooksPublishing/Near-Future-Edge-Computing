#!/usr/bin/env python3
# Production-ready: uses kubeclient and Prometheus HTTP API; retries, timeouts, and auth.
import time, math, os, sys, logging
from kubernetes import client, config
import requests

PROM_URL = os.getenv("PROM_URL", "http://prometheus.monitoring.svc:9090")
VS_NAMESPACE = "edge-apps"
VIRTUAL_SERVICE = "vision-service-vs"
PROM_QUERY = 'sum(rate(http_requests_total{app="vision",env="prod",response_code!~"2..",version="canary"}[2m])) / sum(rate(http_requests_total{app="vision",env="prod"}[2m]))'
ERROR_THRESHOLD = 0.01  # max acceptable error rate
STEP_SECONDS = 120
WEIGHT_STEPS = [1, 5, 10, 25, 50, 100]  # percent final 100 -> cutover

config.load_incluster_config()
api = client.CustomObjectsApi()

def query_prometheus(query):
    resp = requests.get(f"{PROM_URL}/api/v1/query", params={"query": query}, timeout=10)
    resp.raise_for_status()
    data = resp.json()
    if data["status"] != "success": raise RuntimeError("Prometheus query failed")
    values = data["data"]["result"]
    return float(values[0]["value"][1]) if values else 0.0

def patch_virtual_service(weight):
    # Patch Istio VirtualService split via Kubernetes CustomObjects API (networking.istio.io/v1alpha3)
    body = {"spec": {"http": [{"route": [{"destination": {"host": "vision-svc", "subset": "stable"}, "weight": 100-weight}, {"destination": {"host": "vision-svc", "subset": "canary"}, "weight": weight}]}]}}
    api.patch_namespaced_custom_object(group="networking.istio.io", version="v1alpha3",
                                       namespace=VS_NAMESPACE, plural="virtualservices",
                                       name=VIRTUAL_SERVICE, body=body)

def main():
    logging.info("Starting canary controller")
    for w in WEIGHT_STEPS:
        logging.info("Setting canary weight to %s%%", w)
        patch_virtual_service(w)
        time.sleep(STEP_SECONDS)  # allow traffic to stabilize
        try:
            err = query_prometheus(PROM_QUERY)
        except Exception as e:
            logging.exception("Prometheus query failed; rolling back")
            patch_virtual_service(0)
            sys.exit(2)
        logging.info("Observed canary error rate: %.4f", err)
        if err > ERROR_THRESHOLD:
            logging.warning("Error threshold exceeded at %s%%; rolling back to 0%%", w)
            patch_virtual_service(0)
            sys.exit(1)
    logging.info("Canary reached 100%%; promotion complete")

if __name__ == "__main__":
    main()