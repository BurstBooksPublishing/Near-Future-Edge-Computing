# Markov utilities: compute stationary distribution, hitting times, spectral gap, simulate
import numpy as np
from scipy import linalg

def stationary_distribution(P):
    # Solve pi = pi P with normalization. Works for irreducible chains.
    vals, vecs = linalg.eig(P.T)
    # find eigenvector for eigenvalue 1
    idx = np.argmin(np.abs(vals - 1.0))
    pi = np.real(vecs[:, idx])
    pi = pi / pi.sum()
    return pi

def spectral_gap(P):
    vals = linalg.eigvals(P)
    # sort eigenvalues by magnitude, ignore the 1 eigenvalue
    mags = np.sort(np.abs(vals))[::-1]
    if mags[0] < 1-1e-12:
        return 0.0
    return 1.0 - mags[1]

def expected_hitting_time(P, target):
    # target: list or array of absorbing state indices
    n = P.shape[0]
    target = np.atleast_1d(target)
    mask = np.ones(n, dtype=bool)
    mask[target] = False
    Q = P[np.ix_(mask, mask)]
    # Solve (I - Q) h = 1 for non-target states
    I = np.eye(Q.shape[0])
    b = np.ones(Q.shape[0])
    h = linalg.solve(I - Q, b)
    # build full vector, zeros in target states
    full_h = np.zeros(n)
    full_h[mask] = h
    return full_h

def simulate_markov(P, init, steps, rng=None):
    rng = np.random.default_rng(rng)
    state = init
    traj = [state]
    for _ in range(steps):
        state = rng.choice(len(P), p=P[state])
        traj.append(state)
    return np.array(traj)