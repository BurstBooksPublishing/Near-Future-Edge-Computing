from dataclasses import dataclass
from typing import Dict, List, Optional
import math, logging

log = logging.getLogger(__name__)

@dataclass
class NodeProfile:
    id: str
    cpu: float            # available vCPU
    mem_mb: int           # available memory in MB
    accels: Dict[str,int] # {'tensorrt':1, 'npu':1}
    flops: float          # aggregate FLOPS
    load: float           # 0..1 current utilization
    rtt_ms: float         # network round-trip to orchestrator

@dataclass
class TaskProfile:
    id: str
    cpu_req: float
    mem_req_mb: int
    accel_req: Optional[str]  # e.g. 'tensorrt' or None
    flops_req: float
    latency_budget_ms: float

def suitability(node: NodeProfile, task: TaskProfile) -> float:
    # capacity factor penalizes high load and RTT
    cap = max(0.0, 1.0 - node.load) * math.exp(-node.rtt_ms/200.0)
    dot = (node.flops * task.flops_req)
    norm = math.sqrt(node.flops**2) * math.sqrt(task.flops_req**2 + 1e-9)
    flops_score = dot / norm
    return cap * flops_score

def compatible(node: NodeProfile, task: TaskProfile) -> bool:
    if node.cpu < task.cpu_req or node.mem_mb < task.mem_req_mb:
        return False
    if task.accel_req and node.accels.get(task.accel_req,0) == 0:
        return False
    return True

def assign_task(nodes: List[NodeProfile], task: TaskProfile) -> Optional[str]:
    best, best_score = None, -1.0
    for n in nodes:
        if not compatible(n, task):
            continue
        s = suitability(n, task)
        # simple latency estimate: processing proportional to flops_req/flops
        est_proc_ms = (task.flops_req / (n.flops + 1e-9)) * 1000.0
        est_total = n.rtt_ms + est_proc_ms
        if est_total > task.latency_budget_ms:
            continue
        if s > best_score:
            best_score, best = s, n.id
    if best:
        log.info("Assigned task %s -> node %s (score=%.3f)", task.id, best, best_score)
    return best