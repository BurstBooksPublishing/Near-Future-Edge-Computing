#!/usr/bin/env python3
"""
Apply netem delay on remote edge gateway via SSH.
Requires: paramiko (pip install paramiko)
Use with CI/CD secrets and Prometheus alerting enabled.
"""
import os, sys, time
import paramiko

HOST = os.environ.get("EDGE_HOST")         # e.g., 192.0.2.10
USER = os.environ.get("EDGE_USER")         # SSH user
KEY = os.environ.get("EDGE_SSH_KEY_PATH")  # path to private key
IFACE = os.environ.get("EDGE_IFACE", "eth0")
DELAY_MS = int(os.environ.get("DELAY_MS", "200"))
DURATION = int(os.environ.get("DURATION_S", "60"))

def run_ssh_command(client, cmd):
    stdin, stdout, stderr = client.exec_command(cmd)
    out = stdout.read().decode()
    err = stderr.read().decode()
    rc = stdout.channel.recv_exit_status()
    return rc, out, err

def apply_netem(client):
    cmd = f"sudo tc qdisc add dev {IFACE} root netem delay {DELAY_MS}ms"
    return run_ssh_command(client, cmd)

def clear_netem(client):
    cmd = f"sudo tc qdisc del dev {IFACE} root netem || true"
    return run_ssh_command(client, cmd)

def main():
    if not (HOST and USER and KEY):
        print("Missing EDGE_HOST/EDGE_USER/EDGE_SSH_KEY_PATH", file=sys.stderr); sys.exit(2)
    key = paramiko.RSAKey.from_private_key_file(KEY)
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(HOST, username=USER, pkey=key, timeout=10)
    try:
        rc, out, err = apply_netem(client)
        if rc != 0:
            print("Failed to apply netem:", err, file=sys.stderr); sys.exit(3)
        print(f"Applied {DELAY_MS}ms delay on {HOST}:{IFACE} for {DURATION}s")
        time.sleep(DURATION)
    finally:
        rc, out, err = clear_netem(client)
        if rc != 0:
            print("Failed to clear netem:", err, file=sys.stderr)
        else:
            print("Cleared netem, system restored")
        client.close()

if __name__ == "__main__":
    main()