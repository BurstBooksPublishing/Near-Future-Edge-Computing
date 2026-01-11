#!/usr/bin/env python3
"""
Compute T_min = E_emb / (e_old - e_new) per device.
Reads a CSV with columns: id,E_emb,e_old,e_new,expected_T_new
Outputs candidate list for replacement where expected_T_new >= T_min.
"""
import csv
import sys
from math import isclose

def compute_Tmin(E_emb, e_old, e_new):
    # Guard against non-positive denominator
    denom = (e_old - e_new)
    if denom <= 0:
        return float('inf')  # replacement never beneficial
    return E_emb / denom

def process(infile, outfile):
    reader = csv.DictReader(open(infile))
    fieldnames = reader.fieldnames + ['T_min','replace_recommended']
    writer = csv.DictWriter(open(outfile,'w',newline=''), fieldnames=fieldnames)
    writer.writeheader()
    for r in reader:
        E_emb = float(r['E_emb']); e_old = float(r['e_old']); e_new = float(r['e_new'])
        expected_T_new = float(r.get('expected_T_new', 0))
        T_min = compute_Tmin(E_emb, e_old, e_new)
        replace = expected_T_new >= T_min and not isclose(T_min, float('inf'))
        r.update({'T_min':f'{T_min:.2f}','replace_recommended':str(replace)})
        writer.writerow(r)

if __name__ == '__main__':
    if len(sys.argv) != 3:
        print("Usage: replace_thresh.py input.csv output.csv", file=sys.stderr); sys.exit(2)
    process(sys.argv[1], sys.argv[2])