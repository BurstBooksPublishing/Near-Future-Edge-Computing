from typing import Dict, Set, List, Tuple
import heapq

def greedy_weighted_set_cover(universe: Set[int],
                              subsets: Dict[int, Set[int]],
                              costs: Dict[int, float]) -> List[int]:
    """
    universe: set of client ids
    subsets: mapping node_id -> set of clients covered by node
    costs: mapping node_id -> deployment cost (energy+SLA+HW)
    returns: list of selected node_ids
    """
    uncovered = set(universe)
    # current coverage counts
    cover_count = {j: len(subsets[j] & uncovered) for j in subsets}
    # heap items: (cost_per_new, version, node_id)
    heap: List[Tuple[float,int,int]] = []
    version = {j:0 for j in subsets}
    for j in subsets:
        if cover_count[j] > 0:
            heapq.heappush(heap, (costs[j]/cover_count[j], version[j], j))

    selected: List[int] = []
    while uncovered and heap:
        ratio, v, j = heapq.heappop(heap)
        if v != version[j]:
            continue  # stale entry
        new_cov = subsets[j] & uncovered
        if not new_cov:
            continue
        # select node j
        selected.append(j)
        uncovered -= new_cov
        # update affected subsets' counts
        for k in subsets:
            if k == j:
                continue
            if cover_count[k] == 0:
                continue
            reduced = len(subsets[k] & new_cov)
            if reduced:
                cover_count[k] -= reduced
                version[k] += 1
                if cover_count[k] > 0:
                    heapq.heappush(heap, (costs[k]/cover_count[k], version[k], k))
    # if uncovered non-empty, no feasible cover at given tau
    return selected
# Example usage: compute RTT matrix using platform measurement, build subsets,
# then call this function in an edge orchestrator loop (KubeEdge controller).