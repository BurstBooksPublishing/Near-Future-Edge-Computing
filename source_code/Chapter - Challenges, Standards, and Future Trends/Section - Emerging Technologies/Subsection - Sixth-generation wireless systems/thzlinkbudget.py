#!/usr/bin/env python3
# Compute required transmit power for target rate at THz including molecular absorption.
import numpy as np

# Constants
k_B = 1.380649e-23  # Boltzmann constant (J/K)
T0 = 290.0          # Reference temperature (K)

def thermal_noise(bw_hz, noise_figure_db):
    # Returns noise power in Watts
    nf = 10**(noise_figure_db/10)
    return k_B * T0 * bw_hz * nf

def absorption_coeff(freq_hz):
    # Simple empirical placeholder; replace with measured k(f) for accuracy
    # Here we model increasing absorption with f (not a physical model).
    return 1e-3 * (freq_hz / 1e11)  # per meter

def required_pt(target_rate_bps, bw_hz, distance_m, gt=1.0, gr=1.0,
                freq_hz=1e11, noise_figure_db=5.0):
    # Solve for Pt from Shannon and Eq. (2)
    noise = thermal_noise(bw_hz, noise_figure_db)
    # target SNR from capacity inversion
    snr_linear = 2**(target_rate_bps / bw_hz) - 1
    lam = 3e8 / freq_hz
    kf = absorption_coeff(freq_hz)
    path_gain = (lam / (4*np.pi*distance_m))**2 * np.exp(-kf * distance_m)
    # Pt = (SNR * noise) / (Gt*Gr*path_gain)
    return snr_linear * noise / (gt * gr * path_gain)

# Example: 10 Gbps over 10 GHz at 100 GHz, 10 m
if __name__ == "__main__":
    pt_w = required_pt(target_rate_bps=10e9, bw_hz=10e9, distance_m=10,
                       gt=8.0, gr=8.0, freq_hz=1e11, noise_figure_db=5.0)
    print(f"Required Pt = {pt_w:.3e} W ({10*np.log10(pt_w)+30:.1f} dBm)")