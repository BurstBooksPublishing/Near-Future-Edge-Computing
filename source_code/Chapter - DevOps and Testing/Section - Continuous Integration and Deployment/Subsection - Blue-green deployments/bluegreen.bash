#!/usr/bin/env bash
set -euo pipefail
# Usage: ./blue_green_switch.sh NAMESPACE SERVICE TARGET_LABEL_KEY TARGET_LABEL_VALUE HEALTH_PATH TIMEOUT
NAMESPACE="${1:-default}"             # namespace
SERVICE="${2:?service name required}" # service to switch
LABEL_KEY="${3:-app}"                 # selector label key
TARGET="${4:?target label value}"     # e.g. green
HEALTH_PATH="${5:-/healthz}"          # health endpoint path
TIMEOUT="${6:-60s}"                   # rollout wait timeout

# wait for deployment(s) matching label=TARGET to be available
kubectl -n "$NAMESPACE" wait --for=condition=available \
  deployment -l "$LABEL_KEY=$TARGET" --timeout="$TIMEOUT"

# create a short-lived probe pod to exercise the service cluster-local DNS
PROBE_POD="bg-probe-$$"
kubectl -n "$NAMESPACE" run "$PROBE_POD" --restart=Never --image=bitnami/curl \
  --command -- sh -c "sleep 1d" >/dev/null
trap 'kubectl -n "$NAMESPACE" delete pod "$PROBE_POD" --ignore-not-found' EXIT

# perform 5 health checks; fail fast if any non-2xx or high latency
for i in 1 2 3 4 5; do
  OUT=$(kubectl -n "$NAMESPACE" exec "$PROBE_POD" -- \
    sh -c "curl -sS -w '%{http_code} %{time_total}' http://$SERVICE.$NAMESPACE.svc.cluster.local$HEALTH_PATH -o /dev/null")
  HTTP=$(echo "$OUT" | awk '{print $1}')
  LAT=$(echo "$OUT" | awk '{print $2}')
  # fail if not 2xx or latency exceeds 1.0s
  if [ "$HTTP" -lt 200 ] || [ "$HTTP" -ge 300 ] || awk "BEGIN{exit !($LAT < 1.0)}"; then
    echo "Health check failed: $OUT" >&2
    exit 1
  fi
done

# atomically patch service selector to TARGET
PATCH="{\"spec\":{\"selector\":{\"$LABEL_KEY\":\"$TARGET\"}}}"
kubectl -n "$NAMESPACE" patch service "$SERVICE" --type=merge -p "$PATCH"

# post-switch validation window
sleep 5
# quick verification of endpoints count
kubectl -n "$NAMESPACE" get endpoints "$SERVICE" -o yaml
echo "Promotion to $TARGET complete."