#!/usr/bin/env bash
set -euo pipefail

IMAGE="registry.example.com/edge/sensor-proc:1.2.0"
PLATFORMS="linux/arm64,linux/arm/v7,linux/amd64"

# Buildx: create builder once (persistent)
docker buildx create --use --name edgebuilder || true

# Build and push multi-arch image (parallelized, cross-compile)
docker buildx build \
  --platform "$PLATFORMS" \
  --push \
  --tag "$IMAGE" \
  --file Dockerfile .

# On each edge node: run with strict resource and security controls
# Example for Jetson/ARM64 or Raspberry Pi
docker run -d \
  --name sensor-proc \
  --restart unless-stopped \
  --cpus="0.5" \                # allocate half a core-equivalent
  --memory="256m" \            # memory limit to reduce eviction risk
  --pids-limit=64 \            # protect host from fork-bombs
  --security-opt seccomp=/etc/docker/seccomp.json \ # restrict syscalls
  --cap-drop ALL \             # drop Linux capabilities
  --read-only \                # immutability for filesystem
  -v /var/log/sensor:/var/log/sensor:rw \ # persistent logs
  -v /dev/i2c-1:/dev/i2c-1:rwm \           # device access for sensors
  --health-cmd='curl -fsS http://localhost:9000/health || exit 1' \
  --health-interval=10s --health-retries=3 \
  "$IMAGE"
# End of script