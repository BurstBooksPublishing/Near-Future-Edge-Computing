from typing import Dict, List, Tuple
import secrets
import bisect

def select_committee(stakes: Dict[str, int], k: int) -> List[str]:
    """
    Select k distinct validators by stake weights.
    stakes: mapping of validator_id -> integer stake units.
    k: committee size (k <= len(stakes)).
    Returns list of selected validator_ids.
    Secure random source via secrets module for edge deployments.
    """
    if k <= 0 or k > len(stakes):
        raise ValueError("Invalid committee size")
    # Build cumulative weights list
    items: List[Tuple[str,int]] = list(stakes.items())
    ids = [v[0] for v in items]
    weights = [v[1] for v in items]
    total = sum(weights)
    if total <= 0:
        raise ValueError("Total stake must be positive")
    cum = []
    s = 0
    for w in weights:
        s += w
        cum.append(s)
    selected: List[str] = []
    # Sample without replacement by removing selected weight
    for _ in range(k):
        r = secrets.randbelow(total)  # secure int in [0, total)
        idx = bisect.bisect_right(cum, r)
        chosen = ids[idx]
        selected.append(chosen)
        # Remove chosen and adjust cumulative arrays
        removed_weight = weights[idx]
        total -= removed_weight
        del ids[idx], weights[idx], cum[idx]
        for j in range(idx, len(cum)):
            cum[j] -= removed_weight
    return selected

# Example usage:
# stakes = {"nodeA":100, "nodeB":50, "nodeC":200}
# committee = select_committee(stakes, k=2)