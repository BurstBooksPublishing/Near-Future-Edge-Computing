#!/usr/bin/env python3
"""Compute THz link budget and approximate max range for given SNR target.
Integrate into edge RRM modules or use for pre-deployment planning.
"""
import math
from typing import Tuple

k_B = 1.38064852e-23  # Boltzmann constant (J/K)

def dbm_to_watts(p_dbm: float) -> float:
    return 10**((p_dbm - 30.0) / 10.0)

def watts_to_dbm(p_w: float) -> float:
    return 10.0 * math.log10(p_w) + 30.0

def linear_gain_from_dbi(g_dbi: float) -> float:
    return 10**(g_dbi / 10.0)

def received_power_watts(pt_dbm: float, gt_dbi: float, gr_dbi: float,
                         freq_hz: float, distance_m: float, k_abs: float) -> float:
    pt = dbm_to_watts(pt_dbm)
    gt = linear_gain_from_dbi(gt_dbi)
    gr = linear_gain_from_dbi(gr_dbi)
    c = 299792458.0
    lam = c / freq_hz
    fspl = (lam / (4.0 * math.pi * distance_m))**2
    absorption = math.exp(-k_abs * distance_m)
    return pt * gt * gr * fspl * absorption

def noise_power_watts(bw_hz: float, temp_k: float = 290.0) -> float:
    return k_B * temp_k * bw_hz

def max_range_binary_search(pt_dbm: float, gt_dbi: float, gr_dbi: float,
                            freq_hz: float, k_abs: float, bw_hz: float,
                            snr_target_db: float, max_r=1000.0, tol=1e-3) -> float:
    """Binary search range where SNR falls to target (meters)."""
    snr_target = 10**(snr_target_db / 10.0)
    n = noise_power_watts(bw_hz)
    lo, hi = 1e-3, max_r
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        pr = received_power_watts(pt_dbm, gt_dbi, gr_dbi, freq_hz, mid, k_abs)
        snr = pr / n
        if snr >= snr_target:
            lo = mid
        else:
            hi = mid
        if hi - lo < tol:
            break
    return lo

# Example usage integrated into deployment scripts or simulation pipelines
if __name__ == "__main__":
    # parameters selected by edge systems engineer during planning
    pt_dbm = 10.0             # transmit power (dBm) from front-end PA
    gt_dbi = 30.0             # TX array gain (dBi)
    gr_dbi = 30.0             # RX array gain (dBi)
    freq_hz = 300e9           # 300 GHz carrier
    k_abs = 0.02              # absorption coeff (1/m) -- vary per atmospheric model
    bw_hz = 10e9              # 10 GHz channel bandwidth
    snr_target_db = 10.0      # required linear SNR in dB

    max_r = max_range_binary_search(pt_dbm, gt_dbi, gr_dbi, freq_hz, k_abs,
                                    bw_hz, snr_target_db, max_r=200.0)
    print(f"Approx max range for SNR {snr_target_db} dB: {max_r:.2f} m")