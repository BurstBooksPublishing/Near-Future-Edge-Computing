import requests, json, logging, time
from requests.adapters import HTTPAdapter
from urllib3.util import Retry

# Configuration (paths and endpoint should be secured in secret store)
BASE_URL = "https://mec-orchestrator.example.com/mec/v2/applications"
CLIENT_CERT = ("/etc/certs/client.crt", "/etc/certs/client.key")  # mTLS cert/key
CA_BUNDLE = "/etc/certs/ca.pem"
TOKEN = "eyJhbGciOi..."  # short-lived OAuth2 token fetched from identity server

logging.basicConfig(level=logging.INFO)
session = requests.Session()
retries = Retry(total=3, backoff_factor=0.5, status_forcelist=(502,503,504))
session.mount("https://", HTTPAdapter(max_retries=retries))

payload = {
    "appInstanceId": "predictive-maintenance-v1",
    "appName": "predictive-maintenance",
    "appProvider": "acme-industrial",
    "appDescriptor": {"image": "registry.example.com/pm:v1", "cpu": 0.5, "memory": 256},
    "appServiceInfo": [{"serviceName": "anomaly-detector", "transport": "http"}]
}

headers = {"Authorization": f"Bearer {TOKEN}", "Content-Type": "application/json"}

try:
    resp = session.post(BASE_URL, json=payload, headers=headers,
                        cert=CLIENT_CERT, verify=CA_BUNDLE, timeout=10)
    resp.raise_for_status()
    logging.info("Registered MEC application: %s", resp.json())
except requests.RequestException as e:
    logging.error("MEC registration failed: %s", e)
    raise