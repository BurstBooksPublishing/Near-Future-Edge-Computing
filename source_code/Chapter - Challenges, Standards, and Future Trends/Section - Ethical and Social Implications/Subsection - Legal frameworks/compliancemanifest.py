import json, time, hashlib
from cryptography.hazmat.primitives import hashes, serialization
from cryptography.hazmat.primitives.asymmetric import padding
from cryptography.hazmat.backends import default_backend

# Load private key (placeholder; use TPM/HSM in production).
with open("device_signing_key.pem", "rb") as f:
    private_key = serialization.load_pem_private_key(f.read(), password=None, backend=default_backend())

def sha256_hex(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()

manifest = {
    "device_id": "edge-gw-01",                      # unique device identifier
    "timestamp": int(time.time()),
    "firmware_hash": sha256_hex(open("firmware.bin","rb").read()),
    "ai_model": {"name":"pedestrian-detector","version":"v2.1","digest":sha256_hex(open("model.tflite","rb").read())},
    "retention_policy": {"type":"ephemeral","seconds":86400},
    "lawful_basis": "public_safety",                 # policy tag for auditability
    "data_export": {"allowed": False, "exports":"events_only"}
}
payload = json.dumps(manifest, sort_keys=True).encode("utf-8")

# Sign manifest (RSASSA-PSS with SHA256)
signature = private_key.sign(
    payload,
    padding.PSS(mgf=padding.MGF1(hashes.SHA256()), salt_length=padding.PSS.MAX_LENGTH),
    hashes.SHA256()
)

record = {"manifest": manifest, "signature": signature.hex()}
print(json.dumps(record, indent=2))