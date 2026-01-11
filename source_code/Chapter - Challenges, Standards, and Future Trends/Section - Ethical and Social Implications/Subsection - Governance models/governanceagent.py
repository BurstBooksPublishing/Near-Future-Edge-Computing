#!/usr/bin/env python3
"""Edge governance agent: compute G, enforce threshold, log and sign decision."""
import json, sqlite3, hmac, hashlib, time, os
from typing import Dict

DB_PATH = '/var/lib/edge_governance/decisions.db'
HMAC_KEY_PATH = '/etc/edge_governance/hmac_key'  # provisioned from HSM/TPM

def load_key() -> bytes:
    with open(HMAC_KEY_PATH,'rb') as f:
        return f.read().strip()

def init_db():
    os.makedirs(os.path.dirname(DB_PATH), exist_ok=True)
    with sqlite3.connect(DB_PATH) as c:
        c.execute('''CREATE TABLE IF NOT EXISTS decisions
                     (ts INTEGER, workload TEXT, score REAL, threshold REAL, decision TEXT, signature TEXT)''')

def compute_score(metrics: Dict[str,float], weights: Dict[str,float]) -> float:
    # metrics: C,S,P,L normalized 0..1. P and L are risks (higher worse).
    C = metrics['C']; S = metrics['S']; P = metrics['P']; L = metrics['L']
    return weights['wc']*C + weights['ws']*S + weights['wp']*(1.0-P) + weights['wl']*(1.0-L)

def sign_record(record: bytes, key: bytes) -> str:
    return hmac.new(key, record, hashlib.sha256).hexdigest()

def log_decision(workload: str, score: float, threshold: float, decision: str, signature: str):
    with sqlite3.connect(DB_PATH) as c:
        c.execute('INSERT INTO decisions VALUES (?,?,?,?,?,?)',
                  (int(time.time()), workload, score, threshold, decision, signature))

def enforce(workload: str, metrics: Dict[str,float], weights: Dict[str,float], threshold: float):
    key = load_key()
    score = compute_score(metrics, weights)
    decision = 'accept' if score >= threshold else 'defer' if score >= threshold*0.8 else 'reject'
    payload = json.dumps({'ts':int(time.time()), 'workload':workload, 'score':score, 'threshold':threshold, 'decision':decision}).encode()
    signature = sign_record(payload, key)
    log_decision(workload, score, threshold, decision, signature)
    # Return decision and signed payload for upstream transfer.
    return decision, payload, signature

if __name__ == '__main__':
    init_db()
    # Example invocation: metrics are collected from attestation, privacy probe, and latency monitor.
    metrics = {'C':0.95,'S':0.9,'P':0.05,'L':0.12}
    weights = {'wc':0.4,'ws':0.3,'wp':0.2,'wl':0.1}
    decision, payload, sig = enforce('camera-model-v2', metrics, weights, threshold=0.8)
    print(decision, sig)