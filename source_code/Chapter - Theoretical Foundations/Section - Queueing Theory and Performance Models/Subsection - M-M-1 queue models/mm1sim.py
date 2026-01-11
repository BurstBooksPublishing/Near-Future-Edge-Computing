#!/usr/bin/env python3
# Production-ready M/M/1 analytic calculator and SimPy validator.
import math
import random
import statistics
import argparse
import simpy

def analytic_metrics(lmbda, mu):
    if lmbda >= mu:
        raise ValueError("System unstable: lambda >= mu")
    rho = lmbda / mu
    L = rho / (1 - rho)
    Lq = rho**2 / (1 - rho)
    W = 1.0 / (mu - lmbda)
    Wq = rho / (mu - lmbda)
    return {'rho': rho, 'L': L, 'Lq': Lq, 'W': W, 'Wq': Wq}

def sim_mm1(lmbda, mu, n_jobs=200000, seed=42):
    random.seed(seed)
    env = simpy.Environment()
    server = simpy.Resource(env, capacity=1)
    wait_times = []

    def arrival_process():
        for _ in range(n_jobs):
            inter = random.expovariate(lmbda)
            yield env.timeout(inter)
            env.process(service_task(env, server, mu, wait_times))

    def service_task(env, server, mu, wait_times):
        t_arr = env.now
        with server.request() as req:
            yield req
            wait = env.now - t_arr
            wait_times.append(wait)
            service = random.expovariate(mu)
            yield env.timeout(service)

    env.process(arrival_process())
    env.run()
    return {'Wq_sim': statistics.mean(wait_times),
            'Wq_p95': statistics.quantiles(wait_times, n=100)[94],
            'num_samples': len(wait_times)}

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="M/M/1 analytic and SimPy validation")
    parser.add_argument('--lambda', dest='lmbda', type=float, required=True)
    parser.add_argument('--mu', type=float, required=True)
    args = parser.parse_args()
    a = analytic_metrics(args.lmbda, args.mu)
    s = sim_mm1(args.lmbda, args.mu)
    print(f"Analytic: rho={a['rho']:.4f}, W={a['W']*1000:.2f}ms, Wq={a['Wq']*1000:.2f}ms")
    print(f"Simulated mean wait (Wq)={s['Wq_sim']*1000:.2f}ms, 95th={s['Wq_p95']*1000:.2f}ms, samples={s['num_samples']}")