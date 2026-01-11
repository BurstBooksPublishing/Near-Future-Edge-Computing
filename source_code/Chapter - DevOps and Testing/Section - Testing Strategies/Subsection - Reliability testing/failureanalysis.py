#!/usr/bin/env python3
"""Compute reliability metrics from a CSV of events and push to Pushgateway."""
import csv, argparse, datetime, requests, math

def parse_iso(s): return datetime.datetime.fromisoformat(s)

def analyze(events_file):
    # CSV: device_id,event_type,timestamp (event_type in {failure,repair})
    by_device = {}
    with open(events_file) as f:
        for r in csv.DictReader(f):
            d=r['device_id']; t=parse_iso(r['timestamp'])
            by_device.setdefault(d,[]).append((t,r['event_type']))
    mtbf_sum=0.0; mttr_sum=0.0; count=0
    for dev,evs in by_device.items():
        evs.sort()
        last_up=None; down_start=None
        for t,typ in evs:
            if typ=='repair' and down_start:
                mttr_sum += (t-down_start).total_seconds()
                down_start=None
            if typ=='failure':
                if last_up: mtbf_sum += (t-last_up).total_seconds()
                down_start = t
            if typ=='recovered':
                last_up = t
        # if last_up not set, approximate from first event
        count += 1
    mtbf = mtbf_sum/count if count else float('nan')
    mttr = mttr_sum/count if count else float('nan')
    availability = mtbf/(mtbf+mttr) if mtbf+mttr>0 else 0.0
    return mtbf, mttr, availability

def push_metric(pushgw, job, metric, value):
    data = f"# TYPE {metric} gauge\n{metric} {value}\n"
    url=f"{pushgw}/metrics/job/{job}"
    requests.put(url, data=data, timeout=5)

if __name__ == '__main__':
    p=argparse.ArgumentParser()
    p.add_argument('events_csv'); p.add_argument('--pushgw', default=None)
    args=p.parse_args()
    mtbf,mttr,avail = analyze(args.events_csv)
    print(f"MTBF={mtbf:.1f}s MTTR={mttr:.1f}s Availability={avail:.6f}")
    if args.pushgw:
        push_metric(args.pushgw,'reliability_test','edge_availability',avail)