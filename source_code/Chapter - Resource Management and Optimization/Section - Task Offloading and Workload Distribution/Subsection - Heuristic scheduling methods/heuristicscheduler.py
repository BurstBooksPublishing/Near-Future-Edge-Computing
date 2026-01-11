#!/usr/bin/env python3
# Lightweight greedy scheduler: queries Prometheus for node metrics,
# computes cost per node, assigns tasks respecting capacity, and creates Jobs.

import os, time, math, requests
from kubernetes import client, config

PROM_URL = os.getenv("PROM_URL", "http://prometheus:9090/api/v1/query")
NAMESPACE = "edge-tasks"
ALPHA, BETA, GAMMA = 0.6, 0.3, 0.1

config.load_incluster_config()  # runs inside cluster; use load_kube_config() for local
k8s = client.BatchV1Api()
core = client.CoreV1Api()

def prom_query(q):
    r = requests.get(PROM_URL, params={"query": q}, timeout=5)
    r.raise_for_status()
    data = r.json()["data"]["result"]
    return {item["metric"].get("instance",""): float(item["value"][1]) for item in data}

def fetch_node_metrics():
    cpu = prom_query('node_cpu_seconds_total')  # placeholder metric names; adapt per exporter
    mem = prom_query('node_memory_MemAvailable_bytes')
    latency = prom_query('edge_node_network_latency_ms')
    return cpu, mem, latency

def compute_cost(lat_ms, energy, reliability):
    return ALPHA*lat_ms + BETA*energy + GAMMA*(1.0 - reliability)

def schedule_task(task_id, task_size, latency_slo):
    nodes = core.list_node().items
    cpu, mem, lat = fetch_node_metrics()
    candidates = []
    for n in nodes:
        name = n.metadata.name
        available = float(mem.get(name, 1e9))  # bytes
        if available < task_size: continue
        node_lat = float(lat.get(name, 9999.0))
        if node_lat > latency_slo: continue
        # simple proxies for energy and reliability
        energy = 1.0 - (float(cpu.get(name, 0.0)) / 100.0)
        reliability = 0.99  # placeholder; use node exporter or heartbeat in prod
        cost = compute_cost(node_lat, energy, reliability)
        candidates.append((cost, name))
    if not candidates:
        # fallback: run locally as highest-priority pod on edge device
        pod_template = build_pod_spec(task_id, local=True)
    else:
        candidates.sort()
        chosen = candidates[0][1]
        pod_template = build_pod_spec(task_id, node_selector=chosen)
    k8s.create_namespaced_job(NAMESPACE, pod_template)

# build_pod_spec omitted for brevity; implement Job spec with nodeSelector or tolerations.