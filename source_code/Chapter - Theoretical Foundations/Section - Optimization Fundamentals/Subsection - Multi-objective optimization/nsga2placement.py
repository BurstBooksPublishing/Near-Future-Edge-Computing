import numpy as np
from pymoo.core.problem import Problem
from pymoo.algorithms.moo.nsga2 import NSGA2
from pymoo.optimize import minimize
from pymoo.factory import get_sampling, get_crossover, get_mutation

# Scenario parameters (example)
n_nodes = 4  # cloud, microdc, gateway1, gateway2
n_services = 6
r = np.random.randint(1, 4, size=n_services)        # resource demand per service
C = np.array([100, 50, 20, 20])                     # capacities per node
d = np.random.uniform(5, 100, size=(n_nodes, n_services))  # latencies ms
power_idle = np.array([50, 30, 5, 5])               # idle power W
power_per_unit = np.array([0.2, 0.5, 1.0, 1.0])     # W per resource unit used

class EdgePlacementProblem(Problem):
    def __init__(self):
        super().__init__(n_var=n_nodes*n_services, n_obj=2, n_constr=0,
                         xl=0.0, xu=1.0)  # continuous encoding, rounded in evaluate
    def _evaluate(self, X, out, *args, **kwargs):
        n_pop = X.shape[0]
        F = np.zeros((n_pop, 2))
        for k in range(n_pop):
            x_cont = X[k]
            x_bin = (x_cont >= 0.5).astype(int).reshape((n_nodes, n_services))
            # Enforce one-placement-per-service by selecting highest assignment if multiple
            for j in range(n_services):
                col = x_bin[:, j]
                if col.sum() == 0:
                    # assign to cloud by default (index 0)
                    x_bin[0, j] = 1
                elif col.sum() > 1:
                    # keep node with minimal latency
                    best = np.argmin(d[:, j])
                    x_bin[:, j] = 0
                    x_bin[best, j] = 1
            # Capacity check and penalty
            usage = (x_bin * r).sum(axis=1)
            penalty = np.maximum(0, usage - C).sum() * 1e3
            # Objective 1: average latency (ms)
            avg_latency = (x_bin * d).sum() / n_services
            # Objective 2: total energy (W, simplistic model)
            energy = (power_idle + power_per_unit * usage).sum()
            F[k, 0] = avg_latency + penalty
            F[k, 1] = energy + penalty
        out["F"] = F

# Configure NSGA2
algorithm = NSGA2(
    pop_size=100,
    sampling=get_sampling("real_random"),
    crossover=get_crossover("real_sbx", prob=0.9, eta=15),
    mutation=get_mutation("real_pm", eta=20)
)

problem = EdgePlacementProblem()
res = minimize(problem, algorithm, ('n_gen', 200), verbose=False)

# Extract Pareto front (filtered by feasibility from penalties)
pareto = res.F[res.F[:,0] < 1e6]
print("Pareto front points (latency_ms, energy_W):")
print(np.round(pareto, 2))