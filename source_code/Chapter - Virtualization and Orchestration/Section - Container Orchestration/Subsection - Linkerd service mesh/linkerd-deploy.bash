# Install Linkerd control plane (HA recommended for production).
# (Linkerd CLI must be installed on the operator machine)
linkerd install --ha | kubectl apply -f -     # apply manifests

# Verify Linkerd health before deploying workloads.
linkerd check

# Deploy example edge service with sidecar injection enabled.
cat <<'YAML' | kubectl apply -f -
apiVersion: apps/v1
kind: Deployment
metadata:
  name: sensor-agent
  namespace: edge
spec:
  replicas: 2
  selector:
    matchLabels:
      app: sensor-agent
  template:
    metadata:
      labels:
        app: sensor-agent
      annotations:
        linkerd.io/inject: enabled           # request Linkerd proxy injection
    spec:
      containers:
      - name: sensor-agent
        image: myregistry.local/edge/sensor-agent:1.2.0-arm64
        imagePullPolicy: IfNotPresent
        ports:
        - containerPort: 8080
        resources:
          requests:
            cpu: "100m"                      # conservative app CPU request
            memory: "128Mi"
          limits:
            cpu: "250m"
            memory: "256Mi"
        readinessProbe:
          httpGet:
            path: /health
            port: 8080
          initialDelaySeconds: 5
          periodSeconds: 5
        livenessProbe:
          httpGet:
            path: /live
            port: 8080
          initialDelaySeconds: 30
          periodSeconds: 10
YAML

# Expose Prometheus scrape endpoint on the edge cluster for metrics aggregation.
# Linkerd emits Prometheus metrics on the proxy; use scraping relays or pushgateway for intermittent connectivity.