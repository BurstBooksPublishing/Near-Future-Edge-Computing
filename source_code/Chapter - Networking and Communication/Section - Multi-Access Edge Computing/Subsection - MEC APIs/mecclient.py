#!/usr/bin/env python3
# Production-ready MEC Location/RNIS client with mTLS, OAuth2 refresh, and retries.
import json
import logging
from typing import Dict, Any, Optional
import requests
from requests.adapters import HTTPAdapter
from urllib3.util.retry import Retry

logger = logging.getLogger(__name__)
MEC_API_BASE = "https://mec-host.example:8443"  # configured per-site

class MecClient:
    def __init__(self, client_cert: str, client_key: str, ca_bundle: str,
                 token_endpoint: str, client_id: str, client_secret: str):
        self.session = requests.Session()
        # Configure retries for idempotent calls
        retries = Retry(total=3, backoff_factor=0.5,
                        status_forcelist=(429, 500, 502, 503, 504))
        self.session.mount("https://", HTTPAdapter(max_retries=retries))
        # mTLS certificate tuple
        self.session.cert = (client_cert, client_key)
        self.session.verify = ca_bundle
        self.token_endpoint = token_endpoint
        self.client_id = client_id
        self.client_secret = client_secret
        self._token: Optional[Dict[str, Any]] = None

    def _fetch_token(self) -> None:
        # OAuth2 client_credentials flow
        resp = self.session.post(self.token_endpoint,
                                 data={"grant_type": "client_credentials"},
                                 auth=(self.client_id, self.client_secret),
                                 timeout=5.0)
        resp.raise_for_status()
        self._token = resp.json()

    def _auth_header(self) -> Dict[str, str]:
        if not self._token or self._token.get("expires_in", 0) < 30:
            self._fetch_token()
        return {"Authorization": f'Bearer {self._token["access_token"]}'}

    def get_location(self, ue_id: str, timeout: float = 2.0) -> Dict[str, Any]:
        url = f"{MEC_API_BASE}/location/v1/ues/{ue_id}"
        headers = {"Accept": "application/json"}
        headers.update(self._auth_header())
        resp = self.session.get(url, headers=headers, timeout=timeout)
        resp.raise_for_status()
        return resp.json()

    def get_rnis(self, cell_id: str, timeout: float = 1.0) -> Dict[str, Any]:
        url = f"{MEC_API_BASE}/rnis/v1/cells/{cell_id}/measurements"
        headers = {"Accept": "application/json"}
        headers.update(self._auth_header())
        resp = self.session.get(url, headers=headers, timeout=timeout)
        resp.raise_for_status()
        return resp.json()

# Example instantiation omitted for brevity.