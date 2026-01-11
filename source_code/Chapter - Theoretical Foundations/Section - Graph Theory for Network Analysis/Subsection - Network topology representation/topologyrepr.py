#!/usr/bin/env python3
"""
Production-ready topology tool:
- Reads CSV inventory with fields: id,type,ip,connect_to (semicolon-separated),latency_ms,bandwidth_mbps
- Builds weighted NetworkX graph, computes metrics, exports GraphML for visualization/orchestration.
"""
import argparse
import csv
import logging
from pathlib import Path
from typing import Dict, Tuple
import networkx as nx
import numpy as np
from scipy.linalg import eigvalsh

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

def parse_inventory(path: Path) -> Dict[str, dict]:
    nodes = {}
    with path.open() as f:
        rdr = csv.DictReader(f)
        for row in rdr:
            nodes[row['id']] = row
    return nodes

def build_graph(nodes: Dict[str, dict]) -> nx.Graph:
    G = nx.Graph()
    for nid, meta in nodes.items():
        G.add_node(nid, **meta)
    for nid, meta in nodes.items():
        conns = meta.get('connect_to', '')
        for peer in (c.strip() for c in conns.split(';') if c.strip()):
            if peer not in nodes:
                logger.warning("Unknown peer %s referenced by %s", peer, nid)
                continue
            latency = float(meta.get('latency_ms', '10'))
            bw = float(meta.get('bandwidth_mbps', '10'))
            # Use latency as primary weight; keep bandwidth attribute
            if G.has_edge(nid, peer):
                # keep the minimum latency if multiple declarations exist
                prev = G[nid][peer]['weight']
                weight = min(prev, latency)
            else:
                weight = latency
            G.add_edge(nid, peer, weight=weight, bandwidth_mbps=bw)
    return G

def compute_metrics(G: nx.Graph) -> Tuple[float, Dict[str, float]]:
    # Algebraic connectivity: second-smallest eigenvalue of Laplacian
    L = nx.laplacian_matrix(G).astype(float).toarray()
    eigs = eigvalsh(L)
    alg_conn = float(eigs[1]) if eigs.size > 1 else 0.0
    # Betweenness centrality (normalized)
    bet = nx.betweenness_centrality(G, weight='weight', normalized=True)
    return alg_conn, bet

def main():
    p = argparse.ArgumentParser()
    p.add_argument('inventory', type=Path, help='CSV device inventory')
    p.add_argument('--out', type=Path, default=Path('topology.graphml'))
    args = p.parse_args()

    nodes = parse_inventory(args.inventory)
    G = build_graph(nodes)
    alg_conn, bet = compute_metrics(G)
    logger.info("Algebraic connectivity: %.4f", alg_conn)
    # attach centrality as node attribute and export
    nx.set_node_attributes(G, bet, 'betweenness')
    nx.write_graphml(G, args.out)
    logger.info("Graph exported to %s", args.out)

if __name__ == '__main__':
    main()