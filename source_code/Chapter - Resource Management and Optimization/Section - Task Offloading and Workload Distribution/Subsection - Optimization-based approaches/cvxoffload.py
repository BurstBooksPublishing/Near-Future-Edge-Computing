import numpy as np
import cvxpy as cp

# Parameters (measured or estimated)
n = len(w)                          # number of tasks
w = np.array(w)                     # cycles per task
d = np.array(d)                     # bits per task
r = np.maximum(np.array(r), 1e-6)   # measured uplink rates (bits/s)
f_loc = float(f_loc)                # local cycles/s
F_edge = float(F_edge)              # total edge cycles/s
beta = 1e-6                         # energy trade-off weight

# Energy coefficients (tune per hardware)
eps_loc = 1e-9                      # Joule per cycle (local)
eps_tx = 1e-7                       # J/bit (transmit)

# Decision variables (relaxed)
x = cp.Variable(n)                  # relax to [0,1]
f = cp.Variable(n)                  # edge cycles allocated

# Objective: delay + beta*energy
delay_local = (1 - x) * (w / f_loc)
delay_off = x * (d / r + cp.multiply(w, 1.0 / cp.maximum(f, 1e-6)))
energy = (1 - x) * eps_loc * w + x * eps_tx * d
obj = cp.sum(delay_local + delay_off) + beta * cp.sum(energy)

# Constraints
constraints = [
    x >= 0, x <= 1,
    f >= 0,                # f can be zero if x=0
    f <= cp.multiply(x, 1e12),  # big-M style upper bound (per-task)
    cp.sum(f) <= F_edge
]

prob = cp.Problem(cp.Minimize(obj), constraints)
prob.solve(solver=cp.OSQP, eps_abs=1e-4, eps_rel=1e-4, max_iter=10000)

x_relaxed = np.clip(x.value, 0, 1)
f_alloc = np.maximum(f.value, 0)

# Greedy round: prioritize by marginal delay reduction per edge cycle
marginal = (w / f_loc) - (d / r + w / np.maximum(f_alloc, 1e-6))
priority = marginal / (f_alloc + 1e-9)
order = np.argsort(-priority)
x_integer = np.zeros(n, dtype=int)
remaining = F_edge
for i in order:
    need = int(np.ceil(w[i] * 1e-6))   # heuristic mapping cycles to allocation unit
    if need <= remaining and marginal[i] > 0:
        x_integer[i] = 1
        remaining -= need
# x_integer maps to offload decisions; enforce via orchestration APIs