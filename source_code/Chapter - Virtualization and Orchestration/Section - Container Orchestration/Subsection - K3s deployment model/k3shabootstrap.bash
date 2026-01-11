#!/usr/bin/env bash
set -euo pipefail
# Usage: ./bootstrap.sh role host_ip
# role: "server-init" (first server), "server" (additional server), "agent"
ROLE=${1:-server-init}
HOST_IP=${2:-127.0.0.1}
TOKEN_FILE=/etc/rancher/k3s/token
# Common environment for all installs
export K3S_DATASTORE_ENDPOINT=""        # empty -> embedded sqlite or etcd cluster
export INSTALL_K3S_VERSION="v1.28.*/k3s" # pin to minor for stability
# First server: initialize embedded etcd cluster
if [ "$ROLE" = "server-init" ]; then
  curl -sfL https://get.k3s.io | \
    K3S_TOKEN="$(head -c 32 /dev/urandom | base64)" \
    sh -s - server --cluster-init \
      --disable=metrics-server \
      --kube-apiserver-arg=apiserver-count=3 \
      --node-label edge-role=control-plane \
      --tls-san "$HOST_IP"
  # token is created at /var/lib/rancher/k3s/server/node-token
  cp /var/lib/rancher/k3s/server/node-token $TOKEN_FILE
  chmod 600 $TOKEN_FILE
  echo "Initialized server-init on $HOST_IP"
  exit 0
fi
# Additional server or agent join
TOKEN=$(cat $TOKEN_FILE)
if [ "$ROLE" = "server" ]; then
  # join as server to form etcd quorum (use same token)
  curl -sfL https://get.k3s.io | K3S_TOKEN="$TOKEN" sh -s - server \
    --server "https://$HOST_IP:6443" \
    --disable=metrics-server \
    --node-label edge-role=control-plane
  exit 0
fi
if [ "$ROLE" = "agent" ]; then
  # join as worker node
  curl -sfL https://get.k3s.io | K3S_URL="https://$HOST_IP:6443" \
    K3S_TOKEN="$TOKEN" sh -s - agent \
    --node-label edge-role=worker \
    --kubelet-arg=eviction-hard=imagefs.available<1%,nodefs.available<1%
  exit 0
fi