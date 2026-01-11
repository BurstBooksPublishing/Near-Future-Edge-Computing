#!/usr/bin/env python3
# Verify attestation: signature over nonce||digest and digest matches PCR list.
import json, base64, hashlib
from cryptography.hazmat.primitives import hashes, serialization
from cryptography.hazmat.primitives.asymmetric import padding, ec
from cryptography.hazmat.primitives.serialization import load_pem_x509_certificate

def sha256_concat_hex_list(hex_list):
    h = hashlib.sha256()
    for hx in hex_list:
        h.update(bytes.fromhex(hx))
    return h.digest()

def verify_attestation(bundle_json: str, expected_nonce: bytes):
    b = json.loads(bundle_json)
    # Fields: 'ak_cert_pem', 'signature_b64', 'pcr_values' (hex list), 'nonce_b64'
    cert = load_pem_x509_certificate(b['ak_cert_pem'].encode())
    pub = cert.public_key()
    signature = base64.b64decode(b['signature_b64'])
    nonce = base64.b64decode(b['nonce_b64'])
    if nonce != expected_nonce:
        raise ValueError("nonce mismatch or replay detected")
    digest = sha256_concat_hex_list(b['pcr_values'])
    # Recreate signed payload = nonce || digest
    payload = nonce + digest
    # Verify signature (support ECDSA and RSA)
    try:
        if isinstance(pub, ec.EllipticCurvePublicKey):
            pub.verify(signature, payload, ec.ECDSA(hashes.SHA256()))
        else:
            pub.verify(signature, payload, padding.PKCS1v15(), hashes.SHA256())
    except Exception as e:
        raise ValueError("signature verification failed") from e
    return {'status':'ok', 'digest_hex':digest.hex(), 'cert_subject':cert.subject.rfc4514_string()}

# Example usage:
# expected_nonce = os.urandom(20)  # generated per-challenge
# result = verify_attestation(received_bundle_json, expected_nonce)