import numpy as np
from scipy.optimize import linprog

# Problem data: 3 tasks, 3 nodes (edge1, edge2, cloud)
L = np.array([[0.02, 0.03, 0.08],   # latency per unit for task0 -> nodes
              [0.015, 0.025, 0.07], # task1
              [0.03, 0.02, 0.09]])  # task2
r = np.array([2.0, 1.5, 3.0])       # compute units per task
C = np.array([4.0, 3.0, 10.0])      # capacities for nodes

n_tasks, n_nodes = L.shape
c = L.flatten()                      # objective vector

# Equality constraints: each task must sum fractions to 1
A_eq = np.zeros((n_tasks, n_tasks * n_nodes))
for i in range(n_tasks):
    A_eq[i, i * n_nodes:(i + 1) * n_nodes] = 1.0
b_eq = np.ones(n_tasks)

# Inequality constraints: node capacity: sum_i r_i * x_{i,j} <= C_j
A_ub = np.zeros((n_nodes, n_tasks * n_nodes))
for j in range(n_nodes):
    for i in range(n_tasks):
        A_ub[j, i * n_nodes + j] = r[i]
b_ub = C.copy()

# Bounds: 0 <= x_{i,j} <= 1
bounds = [(0.0, 1.0)] * (n_tasks * n_nodes)

# Solve with HiGHS (default in SciPy for linprog)
res = linprog(c, A_ub=A_ub, b_ub=b_ub, A_eq=A_eq, b_eq=b_eq, bounds=bounds, method='highs')

if not res.success:
    raise RuntimeError(f"LP solver failed: {res.message}")

x = res.x.reshape((n_tasks, n_nodes))
print("Fractional offload matrix (tasks x nodes):\n", x)
print("Total expected latency:", res.fun)