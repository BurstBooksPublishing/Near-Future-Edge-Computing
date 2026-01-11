# Production-ready Python: computes steady-state availability for k-of-n CTMC
import numpy as np
from scipy.linalg import solve

def kn_ctmc_availability(n, k, lam, mu):
    # build generator Q for states 0..n (number of UP nodes)
    Q = np.zeros((n+1, n+1))
    for i in range(n+1):
        if i > 0:
            Q[i, i-1] = i * lam         # one of i up nodes fails
        if i < n:
            Q[i, i+1] = (n - i) * mu    # one of (n-i) down nodes repaired
        Q[i, i] = -(Q[i].sum())
    # Solve transpose balance equations: Q^T pi^T = 0 with normalization
    A = np.vstack((Q.T, np.ones(n+1)))
    b = np.zeros(n+2)
    b[-1] = 1.0
    # least-squares solve for numerical robustness on singular matrix
    pi, *_ = np.linalg.lstsq(A, b, rcond=None)
    availability = pi[k:].sum()
    return availability, pi

# Example parameters for edge cluster
n = 3
k = 2
lam = 1e-4   # failures per hour per node
mu = 0.1     # repairs per hour per node
avail, pi = kn_ctmc_availability(n, k, lam, mu)
print(f"2-of-3 availability: {avail:.6f}")
print("steady-state distribution:", np.round(pi, 6))