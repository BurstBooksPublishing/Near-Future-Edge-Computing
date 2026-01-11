#!/usr/bin/env python3
# Carbon-aware scaler: scale down noncritical workloads when carbon intensity rises.

import os
import requests
from kubernetes import client, config

# Configuration (set via environment or secret management)
ELECTRICITYMAP_API = os.getenv("ELECMAP_KEY")  # secure storage recommended
DEPLOYMENT_NS = "edge-batch"
DEPLOYMENT_NAME = "predictive-batch"
SCALE_DOWN_REPLICAS = 0
SCALE_UP_REPLICAS = 2
CARBON_THRESHOLD = 200.0  # gCO2e/kWh threshold

# Load kubeconfig (in-cluster or local)
try:
    config.load_incluster_config()
except Exception:
    config.load_kube_config()

apps_v1 = client.AppsV1Api()

def get_region_from_node(node_name):
    # read node labels to map to electricity region label
    v1 = client.CoreV1Api()
    node = v1.read_node(node_name)
    return node.metadata.labels.get("edge.region", "EU")  # default region

def fetch_carbon_intensity(region):
    url = f"https://api.electricitymap.org/v3/carbon-intensity/latest?zone={region}"
    headers = {"auth-token": ELECTRICITYMAP_API}
    resp = requests.get(url, headers=headers, timeout=5)
    resp.raise_for_status()
    return resp.json()["data"]["carbonIntensity"]

def scale_deployment(replicas):
    body = {"spec": {"replicas": replicas}}
    apps_v1.patch_namespaced_deployment_scale(
        name=DEPLOYMENT_NAME, namespace=DEPLOYMENT_NS, body=body)

def main():
    # pick a representative node (controller can iterate nodes for multi-region)
    node_name = os.getenv("NODE_NAME", "edge-node-1")
    region = get_region_from_node(node_name)
    ci = fetch_carbon_intensity(region)  # gCO2e/kWh

    if ci > CARBON_THRESHOLD:
        scale_deployment(SCALE_DOWN_REPLICAS)  # reduce carbon footprint
    else:
        scale_deployment(SCALE_UP_REPLICAS)  # restore capacity

if __name__ == "__main__":
    main()