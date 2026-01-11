import numpy as np
import random
from typing import List, Dict

# Evaluate latency+energy for an assignment; penalize capacity violations.
def evaluate(assignment: np.ndarray, tasks: List[Dict], nodes: List[Dict],
             alpha: float, penalty: float=1e6) -> float:
    total_lat, total_eng = 0.0, 0.0
    used_cycles = np.zeros(len(nodes), dtype=float)
    for t_idx, n_idx in enumerate(assignment):
        task = tasks[t_idx]; node = nodes[n_idx]
        tx = task['bytes'] / node['bw'] + node['rtt']
        cpu = task['cycles'] / (node['freq'] * node['eff'])
        total_lat += tx + cpu
        total_eng += tx * node['p_tx'] + cpu * node['p_cpu']
        used_cycles[n_idx] += task['cycles']
    # capacity penalty
    for n_idx, node in enumerate(nodes):
        if used_cycles[n_idx] > node['cap']:
            total_lat += penalty
            total_eng += penalty
    return alpha * total_lat + (1 - alpha) * total_eng

# Repair: move tasks from overloaded nodes to nodes with spare capacity.
def repair(assignment: np.ndarray, tasks: List[Dict], nodes: List[Dict]) -> np.ndarray:
    used = np.zeros(len(nodes), dtype=float)
    for t, n in enumerate(assignment): used[n] += tasks[t]['cycles']
    for n_idx, node in enumerate(nodes):
        while used[n_idx] > node['cap']:
            # pick a task assigned to this node with largest cycles
            candidates = [i for i,a in enumerate(assignment) if a==n_idx]
            t_move = max(candidates, key=lambda i: tasks[i]['cycles'])
            # find target node with spare capacity
            targets = sorted(range(len(nodes)), key=lambda j: used[j])
            for tgt in targets:
                if tgt==n_idx: continue
                if used[tgt] + tasks[t_move]['cycles'] <= nodes[tgt]['cap']:
                    assignment[t_move] = tgt
                    used[n_idx] -= tasks[t_move]['cycles']
                    used[tgt] += tasks[t_move]['cycles']
                    break
            else:
                # no feasible move; break to avoid infinite loop
                break
    return assignment

# GA main loop (deterministic seed for reproducibility)
def ga_schedule(tasks: List[Dict], nodes: List[Dict], alpha: float=0.5,
                pop_size: int=100, gens: int=200, seed: int=0):
    random.seed(seed); np.random.seed(seed)
    T, N = len(tasks), len(nodes)
    pop = [np.random.randint(0, N, size=T) for _ in range(pop_size)]
    for gen in range(gens):
        scored = [(evaluate(repair(p.copy(), tasks, nodes), tasks, nodes, alpha), p) for p in pop]
        scored.sort(key=lambda x: x[0])
        elites = [p for _,p in scored[:pop_size//10]]
        newpop = elites.copy()
        while len(newpop) < pop_size:
            # tournament selection
            a = min(random.sample(scored, 3))[1]; b = min(random.sample(scored, 3))[1]
            # one-point crossover
            cut = random.randint(1, T-1)
            child = np.concatenate([a[:cut], b[cut:]])
            # mutation: random-reset
            if random.random() < 0.1:
                idx = random.randrange(T); child[idx] = random.randrange(N)
            newpop.append(repair(child, tasks, nodes))
        pop = newpop
    best = min(pop, key=lambda p: evaluate(p, tasks, nodes, alpha))
    return best
# End of GA scheduler