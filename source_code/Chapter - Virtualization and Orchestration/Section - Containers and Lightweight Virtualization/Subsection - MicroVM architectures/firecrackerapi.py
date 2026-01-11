#!/usr/bin/env python3
# Production-ready: requires requests_unixsocket and pre-created tap device (tap0).
import json, sys, time
import requests_unixsocket

FC_SOCKET = "/tmp/firecracker.socket"         # Firecracker VMM socket path
BASE = "http+unix://%2F" + FC_SOCKET.lstrip("/")  # URL-encoded socket base

session = requests_unixsocket.Session()

def put(path, body):
    url = f"{BASE}{path}"
    r = session.put(url, json=body, timeout=5)
    r.raise_for_status()
    return r

try:
    # 1) Set boot source (kernel image present on host)
    put("/boot-source", {
        "kernel_image_path": "/srv/vm/vmlinux",
        "boot_args": "console=ttyS0 reboot=k panic=1 pci=off"
    })

    # 2) Attach rootfs (pre-created ext4 image)
    put("/drives/rootfs", {
        "drive_id": "rootfs",
        "path_on_host": "/srv/vm/rootfs.ext4",
        "is_root_device": True,
        "is_read_only": False
    })

    # 3) Configure network interface (tap0 must exist and be owned by VMM user)
    put("/network-interfaces/eth0", {
        "iface_id": "eth0",
        "host_dev_name": "tap0",
        "guest_mac": "AA:FC:00:00:00:01"
    })

    # 4) Set machine configuration (vCPU and memory)
    put("/machine-config", {"vcpu_count": 1, "mem_size_mib": 128, "smt": False})

    # 5) Start instance
    put("/actions", {"action_type": "InstanceStart"})

    print("MicroVM started, waiting for guest to initialize...")
    time.sleep(1)  # allow minimal time for guest console to come up

except Exception as e:
    print("Failed to launch microVM:", e, file=sys.stderr)
    sys.exit(1)