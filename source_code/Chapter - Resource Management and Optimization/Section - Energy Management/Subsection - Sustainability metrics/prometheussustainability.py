#!/usr/bin/env python3
"""
Query Prometheus for power and inference counters, compute energy and CO2,
push summarized metrics to a Prometheus Pushgateway.
"""
import requests
import time
from prometheus_client import CollectorRegistry, Gauge, push_to_gateway

PROM_API = "http://prometheus.local:9090/api/v1/query"  # adjust
PUSHGW = "http://pushgateway.local:9091"               # adjust
EF = 0.4  # kgCO2 per kWh, configurable per region

def prom_query(q):
    r = requests.get(PROM_API, params={'query': q}, timeout=10)
    r.raise_for_status()
    data = r.json()
    if data['status'] != 'success':
        raise RuntimeError("Prometheus query failed")
    return data['data']['result']

def sample_metrics(instance_label):
    # Read instantaneous power gauge metric (watched by node exporter or custom exporter)
    power = prom_query(f'power_watts{{instance="{instance_label}"}}')
    # Read cumulative inference counter (monotonic)
    inf = prom_query(f'inference_count{{instance="{instance_label}"}}')
    p_watts = float(power[0]['value'][1]) if power else 0.0
    inf_count = float(inf[0]['value'][1]) if inf else 0.0
    return p_watts, inf_count

def compute_window_energy(p_samples, sample_interval_s):
    # p_samples in watts sampled at fixed interval
    avg_p = sum(p_samples) / len(p_samples)
    energy_kwh = avg_p * (sample_interval_s * len(p_samples)) / 3600.0
    return energy_kwh

def push_metrics(instance_label, energy_kwh, co2_kg, pue=None):
    reg = CollectorRegistry()
    g_energy = Gauge('edge_energy_kwh', 'Energy consumed by edge instance', ['instance'], registry=reg)
    g_co2 = Gauge('edge_co2_kg', 'CO2 emissions attributable to instance', ['instance'], registry=reg)
    g_energy.labels(instance=instance_label).set(energy_kwh)
    g_co2.labels(instance=instance_label).set(co2_kg)
    if pue is not None:
        g_pue = Gauge('facility_pue', 'Facility PUE', ['instance'], registry=reg)
        g_pue.labels(instance=instance_label).set(pue)
    push_to_gateway(PUSHGW, job='sustainability_aggregator', registry=reg)

def main_loop(instance_label, window_s=300, sample_interval_s=5):
    samples = []
    last_inf = None
    while True:
        p, inf = sample_metrics(instance_label)
        samples.append(p)
        if len(samples) * sample_interval_s >= window_s:
            energy_kwh = compute_window_energy(samples, sample_interval_s)
            co2 = energy_kwh * EF
            # optional: query facility energy and compute PUE here
            push_metrics(instance_label, energy_kwh, co2, pue=None)
            samples = []
        time.sleep(sample_interval_s)

if __name__ == "__main__":
    main_loop("jetson-01", window_s=300, sample_interval_s=5)