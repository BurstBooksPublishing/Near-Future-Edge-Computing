#!/usr/bin/env python3
# Weighted rendezvous hashing for capacity-aware static assignment.
import hashlib
import json
from typing import Dict, Iterable, Tuple

def score(key: bytes, node_id: str, weight: float) -> float:
    # deterministic 256-bit hash -> float in (0,1]; divide by weight
    h = hashlib.sha256(key + node_id.encode('utf8')).digest()
    val = int.from_bytes(h, 'big') / (1 << 256)
    return val / max(weight, 1e-9)

def choose_node(key_str: str, nodes: Dict[str, Tuple[str, int, float]]) -> str:
    # nodes: id -> (host, port, weight)
    key = key_str.encode('utf8')
    best, best_score = None, -1.0
    for nid, (_host, _port, weight) in nodes.items():
        s = score(key, nid, weight)
        if s > best_score:
            best, best_score = nid, s
    return best

def build_envoy_upstreams(nodes: Dict[str, Tuple[str, int, float]]):
    # produce Envoy static cluster endpoints JSON fragment
    hosts = []
    for nid, (host, port, _weight) in nodes.items():
        hosts.append({"socket_address":{"address": host, "port_value": port}})
    return {"clusters":[{"name":"edge_static","connect_timeout":"1s",
                         "type":"STATIC","load_assignment":{"cluster_name":"edge_static",
                         "endpoints":[{"lb_endpoints":[{"endpoint":{"address":h}} for h in hosts]}]}}]}

if __name__ == "__main__":
    # example nodes: id -> (host, port, weight)
    nodes = {
        "mec1": ("10.0.0.11", 10000, 200.0),
        "mec2": ("10.0.0.12", 10000, 100.0),
        "mec3": ("10.0.0.13", 10000, 50.0),
    }
    # sample mapping for device IDs
    device_ids = ["cam-{}".format(i) for i in range(1,101)]
    mapping = {d: choose_node(d, nodes) for d in device_ids}
    print(json.dumps({"mapping": mapping}, indent=2))
    # output envoy config fragment to file
    with open("envoy_clusters.json","w") as f:
        json.dump(build_envoy_upstreams(nodes), f, indent=2)