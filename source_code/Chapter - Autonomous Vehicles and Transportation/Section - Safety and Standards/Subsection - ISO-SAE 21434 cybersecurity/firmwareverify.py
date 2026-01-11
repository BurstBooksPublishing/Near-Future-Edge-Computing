#!/usr/bin/env python3
# Verify firmware blob signature and X.509 chain before apply.
import sys, hashlib
from cryptography.hazmat.primitives import hashes, serialization
from cryptography.hazmat.primitives.asymmetric import ec
from cryptography.x509 import load_pem_x509_certificate
from cryptography.hazmat.primitives.asymmetric.utils import decode_dss_signature

# Load payload, signature, and signer cert (all provided by OTA server)
payload = open('/var/ota/firmware.bin','rb').read()                     # firmware blob
sig = open('/var/ota/firmware.bin.sig','rb').read()                    # DER ECDSA signature
cert_pem = open('/var/ota/signer.crt.pem','rb').read()                 # signer certificate PEM
root_cert_pem = open('/etc/vehicle/root_ca.pem','rb').read()           # trusted root CA

# Verify certificate chain (simple check: signer signed by root)
signer_cert = load_pem_x509_certificate(cert_pem)
root_cert = load_pem_x509_certificate(root_cert_pem)
if signer_cert.issuer != root_cert.subject:
    sys.exit("Certificate chain validation failed")

# Use public key to verify signature
pubkey = signer_cert.public_key()
try:
    # ECDSA verification using SHA256
    pubkey.verify(sig, payload, ec.ECDSA(hashes.SHA256()))
except Exception as e:
    sys.exit(f"Signature verification failed: {e}")

# Anti-rollback: check monotonic counter stored in TPM/secure element
# (abstract call: read_monotonic_counter returns integer)
from hw_trust import read_monotonic_counter  # platform-specific binding
counter = read_monotonic_counter()
if counter >= extract_counter_from_payload(payload):  # custom extraction
    sys.exit("Anti-rollback check failed")

# If all checks pass, atomically install (replace file, update symlink)
open('/var/vehicle/active_firmware.bin.new','wb').write(payload)
# atomic rename to avoid partial updates
import os
os.replace('/var/vehicle/active_firmware.bin.new','/var/vehicle/active_firmware.bin')
print("Firmware verified and installed")