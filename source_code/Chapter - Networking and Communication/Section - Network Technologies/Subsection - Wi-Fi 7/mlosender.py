#!/usr/bin/env python3
import socket, time, threading, subprocess, struct

# Configuration: two local source IPs mapped to interfaces.
SRC_A = "192.0.2.10"   # bound to interface for link A
SRC_B = "192.0.2.20"   # bound to interface for link B
DST = ("198.51.100.5", 5005)  # edge server endpoint
REPLICA = True         # replicate critical packets

def measure_rtt(src_ip, dst_ip="198.51.100.5"):
    # Use system ping for robustness; parse average RTT (ms).
    p = subprocess.run(["ping","-c","1","-I",src_ip,dst_ip],
                       stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
    out = p.stdout
    if "time=" in out:
        try:
            t = out.split("time=")[1].split()[0]
            return float(t)
        except Exception:
            return 1000.0
    return 1000.0

def send_pkt(src_ip, payload):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind((src_ip, 0))              # select source IP -> interface
    s.sendto(payload, DST)
    s.close()

def scheduler_loop():
    seq = 0
    while True:
        seq += 1
        payload = struct.pack(">I", seq) + b"|frame|"  # simple header+payload
        # Measure RTTs periodically (could be async); inexpensive here.
        rtt_a = measure_rtt(SRC_A)
        rtt_b = measure_rtt(SRC_B)
        # Choose primary link by lower RTT; replicate if configured.
        primary = SRC_A if rtt_a <= rtt_b else SRC_B
        send_pkt(primary, payload)               # main transmission
        if REPLICA:
            # Fire-and-forget replica on secondary to reduce tail latency.
            secondary = SRC_B if primary == SRC_A else SRC_A
            threading.Thread(target=send_pkt, args=(secondary, payload), daemon=True).start()
        time.sleep(0.005)  # pacing for high-rate streaming (200 PPS)

if __name__ == "__main__":
    scheduler_loop()