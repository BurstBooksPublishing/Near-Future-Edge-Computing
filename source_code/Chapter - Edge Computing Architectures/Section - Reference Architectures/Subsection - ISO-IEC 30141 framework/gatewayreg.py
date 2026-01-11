#!/usr/bin/env python3
import requests, json, logging, time
from requests.adapters import HTTPAdapter
from urllib3.util.retry import Retry

logging.basicConfig(level=logging.INFO)
REGISTRY_URL = "https://registry.example.local/api/v1/devices"
TLS_CA = "/etc/ssl/certs/ca.pem"

payload = {
    "@context": "https://www.w3.org/2018/credentials/v1",
    "id": "urn:edge:gateway:cm4-01",
    "type": ["Gateway","EdgeNode"],
    "manufacturer": "AcmeCorp",
    "capabilities": {
        "protocols": ["MQTT","OPC-UA"],
        "compute": {"cpu_cores": 4, "memory_mb": 4096},
        "accelerators": ["none"]
    },
    "endpoints": [{"protocol": "mqtts","uri": "mqtts://cm4-01.local:8883"}]
}

session = requests.Session()
retries = Retry(total=5, backoff_factor=1,
                status_forcelist=(500,502,503,504))
session.mount("https://", HTTPAdapter(max_retries=retries))

def register():
    headers = {"Content-Type": "application/ld+json"}
    try:
        resp = session.post(REGISTRY_URL, json=payload,
                            headers=headers, timeout=10, verify=TLS_CA)
        resp.raise_for_status()
        logging.info("Registered: %s", resp.json().get("device_id"))
    except requests.RequestException as e:
        logging.error("Registration failed: %s", e)
        raise

if __name__ == "__main__":
    for attempt in range(3):
        try:
            register()
            break
        except Exception:
            time.sleep(2**attempt)
    else:
        logging.critical("Could not register device after retries")