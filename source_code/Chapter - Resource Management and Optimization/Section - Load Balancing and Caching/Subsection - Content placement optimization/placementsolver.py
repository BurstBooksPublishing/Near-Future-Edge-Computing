from pulp import LpProblem, LpMinimize, LpVariable, LpBinary, lpSum, PULP_CBC_CMD

# Example input (replace with telemetry-driven arrays)
nodes = ["n0","n1","n2"]
users = ["u0","u1"]
objects = ["o0","o1"]
d = {("u0","o0"):5,("u0","o1"):1,("u1","o0"):2,("u1","o1"):8}  # request rates
lat = {("u0","n0"):5,("u0","n1"):20,("u0","n2"):50,
       ("u1","n0"):40,("u1","n1"):10,("u1","n2"):30}           # ms
size = {"o0":100,"o1":50}   # MB
cap = {"n0":150,"n1":200,"n2":100}  # MB
placement_cost = {("n","o"):1 for n in nodes for o in objects}

# Build MILP
prob = LpProblem("content_placement", LpMinimize)
x = {(n,o): LpVariable(f"x_{n}_{o}", cat=LpBinary) for n in nodes for o in objects}
y = {(u,n,o): LpVariable(f"y_{u}_{n}_{o}", cat=LpBinary) for u in users for n in nodes for o in objects}

# Objective: latency cost + small placement cost weight
beta = 0.01
prob += lpSum(d[(u,o)]*lat[(u,n)]*y[(u,n,o)] for u in users for n in nodes for o in objects) \
        + beta*lpSum(placement_cost[(n,o)]*x[(n,o)] for n in nodes for o in objects)

# Constraints
for u in users:
    for o in objects:
        prob += lpSum(y[(u,n,o)] for n in nodes) == 1  # every request served
for u in users:
    for n in nodes:
        for o in objects:
            prob += y[(u,n,o)] <= x[(n,o)]               # serve only if placed
for n in nodes:
    prob += lpSum(size[o]*x[(n,o)] for o in objects) <= cap[n]  # storage limits

# Solve
prob.solve(PULP_CBC_CMD(msg=False))
# Extract placements
placements = [(n,o) for n in nodes for o in objects if x[(n,o)].varValue > 0.5]
print("Placements:", placements)