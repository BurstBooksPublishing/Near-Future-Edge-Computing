from nacl.signing import SigningKey, VerifyKey  # libsodium via PyNaCl
from hashlib import blake2s
from typing import List

# Generate or load key material from a secure element in production.
signing_key = SigningKey.generate()          # private key (keep secure)
verify_key = signing_key.verify_key          # public key to distribute

def blake2s_hash(data: bytes) -> bytes:
    h = blake2s(data, digest_size=32)
    return h.digest()

def merkle_root(leaves: List[bytes]) -> bytes:
    if not leaves:
        return blake2s_hash(b'')
    layer = [blake2s_hash(leaf) for leaf in leaves]
    while len(layer) > 1:
        if len(layer) % 2 == 1:
            layer.append(layer[-1])           # duplicate rightmost for even pair
        layer = [blake2s_hash(layer[i] + layer[i+1]) for i in range(0, len(layer), 2)]
    return layer[0]

# Example: sign a block header containing root and timestamp
import time
transactions = [b'tx1:deviceA:payload', b'tx2:deviceB:payload']
root = merkle_root(transactions)
timestamp = int(time.time()).to_bytes(8, 'big')
header = root + timestamp                     # canonical header bytes
signature = signing_key.sign(header).signature  # raw signature bytes

# Export values to network; receivers verify signature with VerifyKey
packet = {'header': header, 'signature': signature, 'pubkey': verify_key.encode()}
# In production serialize with CBOR/Protobuf and protect channel with AEAD.