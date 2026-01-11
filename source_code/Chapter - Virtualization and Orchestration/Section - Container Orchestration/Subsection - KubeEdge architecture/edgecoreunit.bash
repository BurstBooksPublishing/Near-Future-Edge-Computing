# /etc/systemd/system/kubeedge-edgecore.service
[Unit]
Description=KubeEdge edgecore
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
ExecStart=/usr/local/bin/edgecore -c /etc/kubeedge/config/edgecore.yaml
Restart=on-failure
RestartSec=5
LimitNOFILE=1048576

[Install]
WantedBy=multi-user.target

# /etc/kubeedge/config/edgecore.yaml (essential fields)
# containerRuntime: use containerd for production (CRI socket path)
hostname: "edge-gateway-01"
controller:
  servers:
    - address: "cloud.example.com:10000"   # Kubernetes API / cloudcore endpoint
edgehub:
  enable: true
  websocket:
    url: "wss://cloud.example.com:10002/edge/ws" # cloudhub websocket URL
  # TLS certs and CA for secure websocket connection
  tls:
    ca: "/etc/kubeedge/certs/ca.crt"
    cert: "/etc/kubeedge/certs/edge.crt"
    key: "/etc/kubeedge/certs/edge.key"
cloud:
  protocol: "websocket"
edgecontroller:
  enable: true
# CRI configuration for containerd
containerRuntime:
  type: "containerd"
  endpoint: "/run/containerd/containerd.sock"