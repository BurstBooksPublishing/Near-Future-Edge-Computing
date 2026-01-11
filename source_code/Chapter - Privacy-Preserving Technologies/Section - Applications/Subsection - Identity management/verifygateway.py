from flask import Flask, request, jsonify
import requests
import json
import base64
from cryptography.hazmat.primitives import serialization, hashes
from cryptography.hazmat.primitives.asymmetric import padding

app = Flask(__name__)
FABRIC_REVOCATION_API = "https://fabric-local:8443/api/revocation"  # ledger revocation endpoint

def load_public_key(pem_bytes):
    # load PEM public key produced from device DID document
    return serialization.load_pem_public_key(pem_bytes)

def verify_jwt_signature(jwt_token, public_key_pem):
    # JWT format: header.payload.signature (base64url)
    header_b64, payload_b64, sig_b64 = jwt_token.split('.')
    signed_part = (header_b64 + '.' + payload_b64).encode('utf-8')
    signature = base64.urlsafe_b64decode(sig_b64 + '==')
    pub = load_public_key(public_key_pem)
    # RSASSA-PKCS1-v1_5 with SHA256 example; adjust for EC keys
    pub.verify(signature, signed_part,
               padding.PKCS1v15(), hashes.SHA256())
    return json.loads(base64.urlsafe_b64decode(payload_b64 + '==').decode('utf-8'))

def ledger_revoked(device_did):
    # Query Fabric chaincode via REST for revocation status
    r = requests.get(f"{FABRIC_REVOCATION_API}?did={device_did}", timeout=2)
    r.raise_for_status()
    resp = r.json()
    return resp.get("revoked", False)

@app.route("/auth", methods=["POST"])
def auth():
    token = request.json.get("jwt")
    device_did = request.json.get("did")
    # Fetch DID document from ledger or local cache (omitted caching logic)
    did_doc = requests.get(f"https://ledger.local/dids/{device_did}", timeout=2).json()
    pubkey_pem = did_doc["verificationMethod"][0]["publicKeyPem"].encode('utf-8')
    try:
        payload = verify_jwt_signature(token, pubkey_pem)
    except Exception as e:
        return jsonify({"ok": False, "error": "signature_verification_failed"}), 401
    if ledger_revoked(device_did):
        return jsonify({"ok": False, "error": "revoked"}), 403
    # Optional: check TPM attestation claims in payload
    return jsonify({"ok": True, "payload": payload}), 200

# Production considerations: TLS mTLS, local caching, rate-limiting, retries, and monitoring.