#!/usr/bin/env python3
# Verify device possession via certificate signature and verify TOTP factor.
import base64
import time
from cryptography import x509
from cryptography.hazmat.primitives import hashes, serialization
from cryptography.hazmat.primitives.asymmetric import padding
import pyotp

# Load trusted device certificate fingerprint registry (production: use DB)
TRUSTED_FINGERPRINTS = {
    # 'sha256:fingerprint': 'device-id'
}

def fingerprint_sha256(cert_pem: bytes) -> str:
    cert = x509.load_pem_x509_certificate(cert_pem)
    fp = cert.fingerprint(hashes.SHA256())
    return 'sha256:' + fp.hex()

def verify_client_certificate(cert_pem: bytes) -> bool:
    fp = fingerprint_sha256(cert_pem)
    # Minimal check: certificate fingerprint exists in device registry.
    return fp in TRUSTED_FINGERPRINTS

def verify_signature(cert_pem: bytes, signature_b64: str, nonce: bytes) -> bool:
    cert = x509.load_pem_x509_certificate(cert_pem)
    pub = cert.public_key()
    signature = base64.b64decode(signature_b64)
    try:
        pub.verify(
            signature,
            nonce,
            padding.PKCS1v15(),                # aligns with device SE signing policy
            hashes.SHA256()
        )
        return True
    except Exception:
        return False

def verify_totp(token: str, totp_secret: str, skew: int = 1) -> bool:
    totp = pyotp.TOTP(totp_secret)
    # Allow a window to account for clock skew
    return totp.verify(token, valid_window=skew)

def verify_mfa(cert_pem: bytes, signature_b64: str, nonce: bytes,
               totp_token: str, totp_secret: str) -> bool:
    # Step 1: certificate registry check
    if not verify_client_certificate(cert_pem):
        return False
    # Step 2: possession / attestation via signature
    if not verify_signature(cert_pem, signature_b64, nonce):
        return False
    # Step 3: second factor (TOTP)
    if not verify_totp(totp_token, totp_secret):
        return False
    return True

# Example usage (invoked by TLS-handling server)
if __name__ == '__main__':
    # These values would come from the TLS session and device payload
    with open('device_cert.pem','rb') as f: cert = f.read()
    nonce = b'unique-session-nonce-' + str(int(time.time())).encode()
    signature_b64 = 'MEUCIQ...'   # from device
    totp_token = '123456'         # from device
    totp_secret = 'JBSWY3DPEHPK3PXP'  # stored per-device, provisioned securely
    ok = verify_mfa(cert, signature_b64, nonce, totp_token, totp_secret)
    print('MFA success' if ok else 'MFA failed')