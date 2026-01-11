#!/usr/bin/env python3
# Production-ready fault injector: uses paramiko to run tc and stress-ng on remote edge nodes.
import paramiko, logging, time, sys
from contextlib import closing

logging.basicConfig(level=logging.INFO)
NODES = [
    {"host":"edge1.example.local","user":"pi","port":22},
    {"host":"edge2.example.local","user":"ubuntu","port":22},
]
SSH_KEY = "/home/ci/.ssh/id_rsa"  # use SSH key, not password in production
NET_DEV = "eth0"
DELAY = "200ms"
LOSS = "5%"
STRESS_DURATION = 60  # seconds

def run_ssh_cmd(node, cmd):
    logging.info("(%s) %s", node["host"], cmd)
    k = paramiko.RSAKey.from_private_key_file(SSH_KEY)
    with closing(paramiko.SSHClient()) as client:
        client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        client.connect(node["host"], username=node["user"], port=node["port"], pkey=k, timeout=10)
        stdin, stdout, stderr = client.exec_command(cmd)
        out = stdout.read().decode()
        err = stderr.read().decode()
        if err:
            logging.debug("(%s) stderr: %s", node["host"], err.strip())
        return out.strip()

def inject_network(node):
    # add netem; guard with 'tc qdisc show' to avoid duplicates
    run_ssh_cmd(node, f"sudo tc qdisc replace dev {NET_DEV} root netem delay {DELAY} loss {LOSS}")

def remove_network(node):
    run_ssh_cmd(node, f"sudo tc qdisc del dev {NET_DEV} root || true")

def start_cpu_stress(node):
    # start stress-ng in background; ensure package installed in deployment pipeline
    run_ssh_cmd(node, f"nohup sudo stress-ng --cpu 2 --timeout {STRESS_DURATION}s >/dev/null 2>&1 &")

def main():
    try:
        for n in NODES:
            inject_network(n)
            start_cpu_stress(n)
        logging.info("Faults injected, waiting for %ds", STRESS_DURATION)
        time.sleep(STRESS_DURATION + 5)
    except Exception as e:
        logging.error("Injection failed: %s", e)
        raise
    finally:
        for n in NODES:
            remove_network(n)
        logging.info("Cleanup complete")

if __name__ == "__main__":
    main()