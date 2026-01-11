from typing import List, Tuple, Dict
import networkx as nx
import itertools
import logging

logger = logging.getLogger(__name__)

def compute_pareto_paths(
    G: nx.DiGraph,
    source: str,
    target: str,
    k: int = 10,
    energy_limit: float = None
) -> List[Dict]:
    """
    Return up to k Pareto-optimal paths between source and target.
    Each edge must have 'latency' and 'energy' attributes (floats).
    """
    if source not in G or target not in G:
        raise nx.NodeNotFound("source or target not in graph")

    paths = []
    try:
        # generator yields simple paths in order of increasing 'latency' weight
        gen = nx.shortest_simple_paths(G, source, target, weight='latency')
        for path in itertools.islice(gen, k*5):  # oversample candidates
            # compute aggregated metrics
            lat = 0.0
            en = 0.0
            bw = float('inf')
            for u, v in zip(path[:-1], path[1:]):
                data = G.get_edge_data(u, v)
                lat += float(data.get('latency', 0.0))
                en += float(data.get('energy', 0.0))
                bw = min(bw, float(data.get('bandwidth', bw)))
            if energy_limit is not None and en > energy_limit:
                continue
            paths.append({'path': path, 'latency': lat, 'energy': en, 'bandwidth': bw})
            if len(paths) >= k*5:
                break
    except nx.NetworkXNoPath:
        logger.debug("no path found between %s and %s", source, target)

    # extract Pareto frontier by non-dominance on (latency, energy)
    pareto = []
    for cand in sorted(paths, key=lambda x: (x['latency'], x['energy'])):
        dominated = False
        for p in pareto:
            if p['latency'] <= cand['latency'] and p['energy'] <= cand['energy']:
                dominated = True
                break
        if not dominated:
            pareto.append(cand)
            if len(pareto) >= k:
                break
    return pareto