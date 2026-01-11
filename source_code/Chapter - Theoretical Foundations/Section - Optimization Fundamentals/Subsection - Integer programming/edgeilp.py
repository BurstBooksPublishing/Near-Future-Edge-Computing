from ortools.sat.python import cp_model

# Problem data (example for M tasks, N nodes)
M = 6
N = 3
# Example latency and energy cost matrices (integers, scaled)
L = [
  [10, 50, 100],  # task 0 latencies to nodes 0..2
  [15, 60, 120],
  [12, 55, 110],
  [20, 70, 130],
  [18, 65, 125],
  [11, 52, 105],
]
E = [
  [5, 20, 40],
  [6, 24, 48],
  [5, 22, 44],
  [7, 28, 52],
  [6, 26, 50],
  [5, 21, 42],
]
w = [10, 20, 15, 25, 18, 12]   # CPU demand (units)
C = [60, 80, 100]              # node capacities (same units)
D = [200]*M                    # deadlines (large here)
alpha = 2                      # energy weight (integer scaling)

model = cp_model.CpModel()
x = {}
for i in range(M):
    for j in range(N):
        x[i, j] = model.NewBoolVar(f'x_{i}_{j}')

# Each task assigned to exactly one node
for i in range(M):
    model.Add(sum(x[i, j] for j in range(N)) == 1)

# Capacity constraints
for j in range(N):
    model.Add(sum(w[i] * x[i, j] for i in range(M)) <= C[j])

# Optional deadline enforcement (here trivial due to large D)
Mbig = 10**6
for i in range(M):
    for j in range(N):
        model.Add(L[i][j] <= D[i] + Mbig * (1 - x[i, j]))

# Objective: minimize total latency + alpha * energy
obj_terms = []
for i in range(M):
    for j in range(N):
        obj_terms.append((L[i][j] + alpha * E[i][j]) * x[i, j])
model.Minimize(sum(obj_terms))

solver = cp_model.CpSolver()
solver.parameters.max_time_in_seconds = 30.0
solver.parameters.num_search_workers = 8
status = solver.Solve(model)

if status == cp_model.OPTIMAL or status == cp_model.FEASIBLE:
    for i in range(M):
        for j in range(N):
            if solver.Value(x[i, j]):
                print(f"Task {i} -> Node {j}")
    print("Objective:", solver.ObjectiveValue())
else:
    print("No feasible assignment found")