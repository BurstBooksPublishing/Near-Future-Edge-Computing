from typing import List, Dict, Tuple
import math

# Example node record: {'id':'gw-1','cpu':2.0,'mem':2048,'latency_ms':8.0,'energy_j':0.15,'trust':0.95}
def score_nodes(nodes: List[Dict], task: Dict,
                weights: Dict = None) -> List[Tuple[str, float]]:
    """
    Compute score U_i for each node and return sorted list by score desc.
    'task' contains 'deadline_ms', 'energy_budget_j', 'min_trust'.
    """
    if weights is None:
        weights = {'cpu':0.4,'latency':0.3,'energy':0.2,'trust':0.1}
    # normalization bases
    Cmax = max(n['cpu'] for n in nodes) or 1.0
    Emax = task.get('energy_budget_j', 1.0)
    scored = []
    eps = 1e-6
    for n in nodes:
        # compute normalized components
        cpu_norm = n['cpu'] / Cmax
        latency = n['latency_ms'] + task.get('sense_ms', 0.0)
        latency_score = 1.0 / (latency + eps)
        energy_score = 1.0 - min(n['energy_j'] / Emax, 1.0)
        trust = float(n.get('trust', 0.0))
        # hard constraints filter
        if latency > task.get('deadline_ms', float('inf')): 
            continue
        if trust < task.get('min_trust', 0.0):
            continue
        # aggregate score
        U = (weights['cpu']*cpu_norm +
             weights['latency']*latency_score +
             weights['energy']*energy_score +
             weights['trust']*trust)
        scored.append((n['id'], U))
    return sorted(scored, key=lambda x: x[1], reverse=True)

# Scheduler integrates result with KubeEdge by annotating Pod spec or using device twin APIs.