import requests
from requests.adapters import HTTPAdapter
from urllib3.util.retry import Retry

# Configuration (replace with operator/environment values)
NEF_BASE = "https://nef.operator.example.com/nef/v1"
CLIENT_ID = "edge_app_client"
CLIENT_SECRET = "s3cr3t"  # use secure vault in production
OAUTH_TOKEN_URL = "https://auth.operator.example.com/oauth2/token"

# Robust requests session with retries
session = requests.Session()
retries = Retry(total=3, backoff_factor=0.5, status_forcelist=(500,502,503,504))
session.mount("https://", HTTPAdapter(max_retries=retries))

def get_oauth_token():
    # Client credentials grant; secure storage for client_secret required
    resp = session.post(OAUTH_TOKEN_URL, data={"grant_type": "client_credentials"},
                        auth=(CLIENT_ID, CLIENT_SECRET), timeout=5)
    resp.raise_for_status()
    return resp.json()["access_token"]

def create_nef_subscription(token, subscriber_id):
    url = f"{NEF_BASE}/subscriptions"
    payload = {
        "subscriberId": subscriber_id,
        "eventFilter": ["UE_LOCATION_CHANGE","QOS_CHANGE"],
        "callbackReference": {"notifyURL": "https://mec-host.example.com/nef/callback"}
    }
    headers = {"Authorization": f"Bearer {token}", "Content-Type": "application/json"}
    resp = session.post(url, json=payload, headers=headers, timeout=5)
    resp.raise_for_status()
    return resp.json()

if __name__ == "__main__":
    token = get_oauth_token()
    sub = create_nef_subscription(token, subscriber_id="IMSI123456789")
    print("NEF subscription created:", sub)