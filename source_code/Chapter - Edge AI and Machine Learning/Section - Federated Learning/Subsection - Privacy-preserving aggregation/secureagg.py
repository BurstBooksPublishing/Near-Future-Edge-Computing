# secure_agg.py -- minimal secure aggregation primitives for edge devices
from typing import Dict, List
import secrets, hashlib
import numpy as np

PRIME = 2**61 - 1  # Mersenne prime for fast modular reduction

def rand_vector(dim: int) -> np.ndarray:
    # cryptographically secure random vector in [0, PRIME)
    return np.frombuffer(secrets.token_bytes(8*dim), dtype=np.uint64) % PRIME

def seed_pairwise(client_a: str, client_b: str, dim: int) -> np.ndarray:
    # deterministic seed from client IDs via HMAC-like construction
    key = (client_a + '||' + client_b).encode()
    h = hashlib.sha256(key).digest()
    # expand hash to vector deterministically
    rng = int.from_bytes(h, 'big')
    out = np.empty(dim, dtype=np.int64)
    for i in range(dim):
        rng = (rng * 6364136223846793005 + 1) & ((1<<64)-1)
        out[i] = rng % PRIME
    return out

class Client:
    def __init__(self, client_id: str, dim: int):
        self.id = client_id
        self.dim = dim
        self.update = np.zeros(dim, dtype=np.int64)

    def set_update(self, float_update: np.ndarray, scale: float = 1e6):
        # quantize to integers in finite field before masking
        q = np.round(float_update * scale).astype(np.int64) % PRIME
        self.update = q

    def masked_share(self, peers: List[str]) -> np.ndarray:
        mask = np.zeros(self.dim, dtype=np.int64)
        for peer in peers:
            if peer == self.id: continue
            if self.id < peer:
                s = seed_pairwise(self.id, peer, self.dim)
                mask = (mask + s) % PRIME
            else:
                s = seed_pairwise(peer, self.id, self.dim)
                mask = (mask - s) % PRIME
        return (self.update + mask) % PRIME

class Aggregator:
    def __init__(self, dim: int, scale: float = 1e6):
        self.dim = dim
        self.scale = scale

    def recover_sum(self, masked_shares: List[np.ndarray]) -> np.ndarray:
        # sum modulo PRIME and convert back to floats
        s = sum((m % PRIME) for m in masked_shares) % PRIME
        # handle signed representation
        s_signed = s.astype(np.int64)
        s_signed[s_signed > PRIME//2] -= PRIME
        return s_signed.astype(np.float64) / self.scale