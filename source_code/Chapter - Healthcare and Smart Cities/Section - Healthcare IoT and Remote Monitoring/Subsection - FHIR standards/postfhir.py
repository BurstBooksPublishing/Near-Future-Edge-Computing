import time, json, requests, jwt
from requests.adapters import HTTPAdapter
from urllib3.util.retry import Retry

# Config (store secrets securely, e.g., hardware keystore or Vault)
FHIR_BASE = "https://localhost:9443/fhir"
CLIENT_CERT = ("/etc/certs/client.crt", "/etc/certs/client.key")  # mTLS
ROOT_CA = "/etc/certs/ca.pem"
TOKEN_ENDPOINT = "https://auth.example.com/oauth2/token"
CLIENT_ID = "edge-gateway"
CLIENT_SECRET = "REDACTED"  # use secure storage
SCOPE = "user/Observation.write offline_access"
SESSION = requests.Session()
# Retry policy for transient network errors
retries = Retry(total=5, backoff_factor=0.5, status_forcelist=(429,500,502,503,504))
SESSION.mount("https://", HTTPAdapter(max_retries=retries))

_token_cache = {"access_token": None, "expires_at": 0}

def get_bearer_token():
    now = int(time.time())
    if _token_cache["access_token"] and _token_cache["expires_at"] - 30 > now:
        return _token_cache["access_token"]
    # Client credentials grant (or use signed JWT assertion for production)
    r = SESSION.post(TOKEN_ENDPOINT, data={"grant_type":"client_credentials","scope":SCOPE},
                     auth=(CLIENT_ID, CLIENT_SECRET), verify=ROOT_CA)
    r.raise_for_status()
    tok = r.json()
    _token_cache["access_token"] = tok["access_token"]
    _token_cache["expires_at"] = now + tok.get("expires_in", 3600)
    return _token_cache["access_token"]

def make_observation(patient_id, device_id, code, value, unit, timestamp):
    obs = {
      "resourceType": "Observation",
      "status": "final",
      "category": [{"coding":[{"system":"http://terminology.hl7.org/CodeSystem/observation-category","code":"vital-signs"}]}],
      "code": {"coding":[{"system":"http://loinc.org","code": code}]},
      "subject": {"reference": f"Patient/{patient_id}"},
      "device": {"reference": f"Device/{device_id}"},
      "effectiveDateTime": timestamp,
      "valueQuantity": {"value": value, "unit": unit}
    }
    return obs

def post_observation_batch(observations):
    token = get_bearer_token()
    url = f"{FHIR_BASE}/Observation/_batch"
    headers = {"Authorization": f"Bearer {token}", "Content-Type": "application/fhir+json"}
    # Use mTLS and CA verification
    r = SESSION.post(url, json={"resourceType":"Bundle","type":"transaction","entry":[
        {"resource":obs,"request":{"method":"POST","url":"Observation"}} for obs in observations
    ]}, headers=headers, cert=CLIENT_CERT, verify=ROOT_CA)
    r.raise_for_status()
    return r.json()

# Example usage: create and send a single observation
if __name__ == "__main__":
    obs = make_observation("123", "dev-ecg-01", "8867-4", 72, "beats/min", "2025-01-01T12:00:00Z")
    print(post_observation_batch([obs]))