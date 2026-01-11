import numpy as np
import cvxpy as cp

# Parameters: 4 approaches, 12 time slots
I, T = 4, 12
a = np.maximum(0, np.random.normal(loc=2.5, scale=0.5, size=(T, I)))  # measured arrivals per slot
mu = np.array([6.0, 6.0, 5.5, 5.0])  # saturation flows (veh/slot)
q0 = np.zeros(I)                         # initial queues
min_green = 1.0                          # slots
max_cycle = 6.0                          # slots per cycle

# Decision variables: relaxed green fraction per approach per slot
u = cp.Variable((T, I))
q = cp.Variable((T+1, I))

# Constraints
constraints = [q[0, :] == q0]
for t in range(T):
    constraints += [
        q[t+1, :] == q[t, :] + a[t, :] - cp.multiply(mu, u[t, :]),  # discrete queue dynamics
        q[t+1, :] >= 0,
        u[t, :] >= 0, u[t, :] <= 1
    ]
    # cycle constraint: sum of green fractions within a cycle window <= max_cycle
    # here we enforce per-slot sum <= max_cycle/T to approximate cycle budget
    constraints += [cp.sum(u[t, :]) <= max_cycle / 1.0]

# Minimum green enforcement (relaxed)
constraints += [cp.sum(u, axis=0) >= min_green * (T / max_cycle)]

# Objective: minimize total queued vehicles (proxy for delay)
objective = cp.Minimize(cp.sum(q[1:, :]))

# Solve with OSQP (fast for QP/LP)
prob = cp.Problem(objective, constraints)
prob.solve(solver=cp.OSQP, max_iter=10000, eps_abs=1e-3, eps_rel=1e-3)

# Output schedule (convert relaxed fractions to millisecond timings)
green_schedule = (u.value * 5.0)  # each slot = 5 seconds
print("Green schedule (s) per slot per approach:\n", green_schedule)