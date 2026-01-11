#!/usr/bin/env python3
# Production-ready: requires 'pulp' and a control-plane integration.
from pulp import LpProblem, LpVariable, lpSum, LpMaximize, LpBinary, PULP_CBC_CMD
def optimize_placement(objects, nodes, request_rate, latency_edge, latency_cloud):
    # objects: dict id->size
    # nodes: dict id->capacity
    # request_rate: dict (user,object)->rate aggregated per node region
    # latency_edge: dict (user,node)->latency_ms
    # latency_cloud: dict user->latency_ms
    prob = LpProblem("edge_placement", LpMaximize)
    x = {(o,n): LpVariable(f"x_{o}_{n}", cat=LpBinary) for o in objects for n in nodes}
    # Approximate benefit as saved latency * request_rate
    benefit = lpSum(
        request_rate.get((u,o),0) * (latency_cloud[u] - min(latency_edge.get((u,n), latency_cloud[u]) for n in nodes) )
        * x[o,n]
        for o in objects for n in nodes for u in set(u for (u,_) in request_rate if _==o)
    )
    prob += benefit
    # capacity constraints
    for n in nodes:
        prob += lpSum(objects[o]*x[o,n] for o in objects) <= nodes[n]
    # optional: at-most-one-placement per object per region can be added
    prob.solve(PULP_CBC_CMD(msg=0))
    return {(o,n): int(x[o,n].value()) for o in objects for n in nodes}
# Example usage: integrate returned mapping with an operator to push caches via rsync/OCI image layers.