# Requires: pip install pulp
from pulp import LpProblem, LpMinimize, LpVariable, lpSum, LpBinary
import numpy as np

def solve_placement(latency, cpu_demand, cpu_capacity, weights=None):
    # latency: (N_tasks, M_hosts) matrix of estimated L_e2e
    # cpu_demand: length-N array of CPU demand per task (units/sec)
    # cpu_capacity: length-M array of CPU capacity per host (units/sec)
    N, M = latency.shape
    weights = np.ones(N) if weights is None else np.array(weights)
    prob = LpProblem("mec_placement", LpMinimize)

    # Decision variables x[i][j] == 1 if task i assigned to host j
    x = [[LpVariable(f"x_{i}_{j}", cat=LpBinary) for j in range(M)]
         for i in range(N)]

    # Objective: minimize weighted total latency
    prob += lpSum(weights[i] * latency[i, j] * x[i][j]
                  for i in range(N) for j in range(M))

    # Each task assigned to exactly one host
    for i in range(N):
        prob += lpSum(x[i][j] for j in range(M)) == 1

    # Capacity constraints on hosts
    for j in range(M):
        prob += lpSum(cpu_demand[i] * x[i][j] for i in range(N)) <= cpu_capacity[j]

    prob.solve()
    assignment = [next(j for j in range(M) if x[i][j].value()==1)
                  for i in range(N)]
    return assignment
# Example usage: compute assignment for measured latency matrix.