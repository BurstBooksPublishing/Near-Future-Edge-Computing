#!/usr/bin/env python3
"""
Edge attack-surface scanner:
- collects listening ports, processes, container runtimes, systemd services
- checks firmware/kernel, TPM2 presence, and docker/k3s API sockets
- writes JSON report for CI/CD or remote attestation
"""
import json, subprocess, shutil, logging, sys
from pathlib import Path

logging.basicConfig(level=logging.INFO)
OUT = Path('/var/tmp/edge_attack_surface.json')

def run(cmd):
    return subprocess.run(cmd, shell=True, stdout=subprocess.PIPE,
                          stderr=subprocess.PIPE, text=True)

def check_listening():
    # use ss for portability and low overhead
    r = run("ss -lntu -H")
    lines = [ln.strip() for ln in r.stdout.splitlines() if ln.strip()]
    return lines

def check_containers():
    # detect common runtimes
    runtimes = {}
    if shutil.which('docker'):
        runtimes['docker'] = run("docker ps --format '{{.Names}}'").stdout.splitlines()
    if shutil.which('crictl'):
        runtimes['crictl'] = run("crictl ps -o json").stdout
    # check k3s/kubelet presence
    runtimes['k3s'] = bool(shutil.which('k3s'))
    return runtimes

def check_systemd_services():
    if not shutil.which('systemctl'):
        return {}
    r = run("systemctl list-units --type=service --no-pager --no-legend")
    svc = [ln.split()[0] for ln in r.stdout.splitlines() if ln.strip()]
    return svc

def check_tpm():
    # simple TPM2 detection via tpm2_getrandom or /dev/tpm0
    if Path('/dev/tpm0').exists() or shutil.which('tpm2_getrandom'):
        return True
    return False

def kernel_and_fw():
    uname = run("uname -a").stdout.strip()
    fw = run("cat /sys/class/dmi/id/bios_version 2>/dev/null").stdout.strip()
    return {'uname': uname, 'bios': fw}

def main():
    report = {
        'listening': check_listening(),
        'containers': check_containers(),
        'services': check_systemd_services(),
        'tpm_present': check_tpm(),
        'kernel_fw': kernel_and_fw(),
    }
    OUT.write_text(json.dumps(report, indent=2))
    logging.info("Report written to %s", OUT)

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        logging.exception("Scanner failed: %s", e)
        sys.exit(2)