#!/usr/bin/env python3
# Rotate short-lived ES256 JWTs and expose thread-safe getter.
import threading, time, logging
from datetime import datetime, timedelta
import jwt  # PyJWT
from cryptography.hazmat.primitives import serialization

logger = logging.getLogger("jwt_rotator")
logger.setLevel(logging.INFO)

class JWTRotator:
    def __init__(self, privkey_pem_path, kid, audience, ttl_seconds=300, refresh_margin=0.2):
        # Load private key (replace with HSM/PKCS#11 loader)
        with open(privkey_pem_path, "rb") as f:
            self._priv = serialization.load_pem_private_key(f.read(), password=None)
        self._kid = kid
        self._aud = audience
        self._ttl = int(ttl_seconds)
        self._refresh_margin = float(refresh_margin)
        self._lock = threading.Lock()
        self._token = None
        self._expiry = 0
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._rotate_loop, daemon=True)
        self._thread.start()

    def _create_jwt(self):
        iat = int(time.time())
        exp = iat + self._ttl
        payload = {"iat": iat, "exp": exp, "aud": self._aud}
        headers = {"kid": self._kid, "alg": "ES256", "typ": "JWT"}
        token = jwt.encode(payload, self._priv, algorithm="ES256", headers=headers)
        return token, exp

    def _rotate_loop(self):
        while not self._stop.is_set():
            with self._lock:
                if self._token is None or time.time() >= self._expiry - self._ttl * self._refresh_margin:
                    try:
                        token, exp = self._create_jwt()
                        self._token, self._expiry = token, exp
                        logger.info("Rotated JWT exp=%s", datetime.utcfromtimestamp(exp).isoformat())
                    except Exception as e:
                        logger.exception("JWT creation failed: %s", e)
            # Sleep short to react quickly to expiry; adjust for system needs.
            time.sleep(max(1, self._ttl * self._refresh_margin / 4))

    def get_token(self):
        with self._lock:
            return self._token

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=2)

# Example usage:
# rotator = JWTRotator("/persist/device_key.pem", kid="device-001", audience="edge-control", ttl_seconds=300)
# token = rotator.get_token()