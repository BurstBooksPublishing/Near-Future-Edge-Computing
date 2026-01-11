#!/usr/bin/env python3
"""
Collects synchronized power and utilization samples, fits linear model,
and exports coefficients. Requires psutil, numpy, sklearn, adafruit-ina219 (optional).
Run with root for RAPL/sysfs access or i2c device access.
"""
import time, argparse, os, json
import psutil, numpy as np
from sklearn.linear_model import Ridge  # robust regularized fit

def read_rapl_energy(domain="/sys/class/powercap/intel-rapl:0/energy_uj"):
    with open(domain, "r") as f:
        return int(f.read().strip()) * 1e-6  # J

def read_ina219_power(bus=None):
    # placeholder: integrate Adafruit library in production
    raise NotImplementedError("Install adafruit-circuitpython-ina219 and implement sensor read.")

def collect_sample(use_rapl=True, rapl_path=None):
    t0 = time.time()
    cpu_pct = psutil.cpu_percent(interval=None) / 100.0
    net = psutil.net_io_counters()
    # store cumulative bytes; caller must compute delta over sampling interval
    sample = {"ts": t0, "cpu": cpu_pct, "net_bytes": net.bytes_sent + net.bytes_recv}
    if use_rapl:
        sample["energy_j"] = read_rapl_energy(rapl_path)  # cumulative energy
    return sample

def build_dataset(samples):
    # convert cumulative energy to instantaneous power by diffing
    X, y = [], []
    for i in range(1, len(samples)):
        dt = samples[i]["ts"] - samples[i-1]["ts"]
        if dt <= 0: continue
        dE = samples[i]["energy_j"] - samples[i-1]["energy_j"]
        P_inst = dE / dt
        net_rate = (samples[i]["net_bytes"] - samples[i-1]["net_bytes"]) / dt
        X.append([1.0, samples[i]["cpu"], net_rate])
        y.append(P_inst)
    return np.array(X), np.array(y)

def fit_model(X, y):
    reg = Ridge(alpha=1e-3, fit_intercept=False)  # solve for coefficients including constant term
    reg.fit(X, y)
    preds = reg.predict(X)
    rmse = np.sqrt(np.mean((y - preds) ** 2))
    return reg.coef_, rmse

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--samples", type=int, default=120)
    parser.add_argument("--interval", type=float, default=1.0)
    parser.add_argument("--rapl", default="/sys/class/powercap/intel-rapl:0/energy_uj")
    args = parser.parse_args()

    samples = []
    # warmup measurement for cumulative counters
    samples.append(collect_sample(use_rapl=True, rapl_path=args.rapl))
    for _ in range(args.samples):
        time.sleep(args.interval)
        samples.append(collect_sample(use_rapl=True, rapl_path=args.rapl))

    X, y = build_dataset(samples)
    coeffs, rmse = fit_model(X, y)
    out = {"coeffs": coeffs.tolist(), "rmse": float(rmse)}
    print(json.dumps(out, indent=2))

if __name__ == "__main__":
    main()