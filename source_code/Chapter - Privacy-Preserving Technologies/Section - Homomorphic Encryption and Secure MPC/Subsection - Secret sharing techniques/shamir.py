from secrets import randbelow
from typing import List, Tuple

# Modular arithmetic helpers (p must be prime)
def mod_inv(a: int, p: int) -> int:
    # Fermat inverse, safe when p is prime
    return pow(a % p, p - 2, p)

def eval_poly(coeffs: List[int], x: int, p: int) -> int:
    # Horner evaluation of polynomial at x modulo p
    result = 0
    for c in reversed(coeffs):
        result = (result * x + c) % p
    return result

def make_shares(secret: int, n: int, t: int, p: int) -> List[Tuple[int,int]]:
    """
    Generate n shares with threshold t over GF(p).
    Returns list of (x, y) with x in 1..p-1.
    """
    if not (1 < t <= n):
        raise ValueError("Require 1 < t <= n")
    if not (0 <= secret < p):
        raise ValueError("Secret must be in GF(p)")
    # random polynomial coefficients: f(0)=secret
    coeffs = [secret] + [randbelow(p) for _ in range(t-1)]
    xs = list(range(1, n+1))
    return [(x, eval_poly(coeffs, x, p)) for x in xs]

def reconstruct(shares: List[Tuple[int,int]], p: int) -> int:
    """
    Reconstruct secret from at least t shares using Lagrange interpolation at 0.
    """
    x_s = [x for x,_ in shares]
    y_s = [y for _,y in shares]
    secret = 0
    for j, xj in enumerate(x_s):
        num = 1
        den = 1
        for m, xm in enumerate(x_s):
            if m == j:
                continue
            num = (num * xm) % p
            den = (den * (xm - xj)) % p
        lag = num * mod_inv(den, p) % p
        secret = (secret + y_s[j] * lag) % p
    return secret