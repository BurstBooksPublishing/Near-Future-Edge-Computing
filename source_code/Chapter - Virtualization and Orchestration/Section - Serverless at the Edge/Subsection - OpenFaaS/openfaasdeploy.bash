#!/usr/bin/env bash
set -euo pipefail

# Usage: ./deploy.sh registry.example.com/edge-anom:0.1 my-gateway:8080
IMAGE=${1:-registry.example.com/edge-anom:latest}  # target multi-arch image
GATEWAY=${2:-127.0.0.1:8080}                        # OpenFaaS gateway
SRC_DIR=${3:-.}                                    # function source directory

# ensure builder exists
docker buildx inspect edge-builder >/dev/null 2>&1 || docker buildx create --name edge-builder --use

# build multi-arch image (arm/v7, arm64, amd64) and push
docker buildx build --platform linux/arm/v7,linux/arm64,linux/amd64 \
  -t "$IMAGE" --push "$SRC_DIR"

# create a minimal stack.yml for faas-cli deploy (resource limits tuned for edge)
cat > stack.yml <<EOF
version: 1.0
provider:
  name: openfaas
  gateway: http://${GATEWAY}
functions:
  edge-anom:
    lang: dockerfile
    handler: .
    image: ${IMAGE}
    limits:
      cpu: 200m
      memory: 128Mi
    requests:
      cpu: 50m
      memory: 64Mi
    environment:
      read_timeout: 20s
      write_timeout: 20s
      exec_timeout: 20s
EOF

# deploy to cluster
faas-cli deploy -f stack.yml --gateway "http://${GATEWAY}"