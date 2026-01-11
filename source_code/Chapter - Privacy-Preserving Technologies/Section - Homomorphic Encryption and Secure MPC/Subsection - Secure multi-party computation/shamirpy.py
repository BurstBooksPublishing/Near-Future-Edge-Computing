# shamir.py -- simple Shamir secret sharing (production: add authentication and constant-time ops)
import secrets
from typing import List, Tuple

PRIME = 2**61 - 1  # large prime < 2^63 for safe Python int ops

def _eval_poly(coeffs: List[int], x: int, p: int) -> int:
    # Horner's method, modulo p
    res = 0
    for a in reversed(coeffs):
        res = (res * x + a) % p
    return res

def share_secret(secret: int, n: int, t: int, p: int = PRIME) -> List[Tuple[int,int]]:
    # returns list of (i, share_i)
    if not (0 <= secret < p):
        raise ValueError("secret out of field range")
    coeffs = [secret] + [secrets.randbelow(p) for _ in range(t)]
    shares = [(i, _eval_poly(coeffs, i, p)) for i in range(1, n+1)]
    return shares

def _lagrange_interpolate(x: int, points: List[Tuple[int,int]], p: int) -> int:
    # reconstruct f(x) given points using Lagrange basis
    total = 0
    for j, (xj, yj) in enumerate(points):
        num = 1
        den = 1
        for m, (xm, _) in enumerate(points):
            if m == j: continue
            num = (num * (x - xm)) % p
            den = (den * (xj - xm)) % p
        inv_den = pow(den, -1, p)
        total = (total + yj * num * inv_den) % p
    return total

def reconstruct(shares: List[Tuple[int,int]], p: int = PRIME) -> int:
    # reconstruct secret f(0)
    return _lagrange_interpolate(0, shares, p)

# Example usage (edge device): split secret, send shares to peers via secure channel
if __name__ == "__main__":
    s = 12345
    shares = share_secret(s, n=5, t=2)
    # distribute shares to devices; later collect any 3 for reconstruction
    secret_rec = reconstruct(shares[:3])
    assert secret_rec == s