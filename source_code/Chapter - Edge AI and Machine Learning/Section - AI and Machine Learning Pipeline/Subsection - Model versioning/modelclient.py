#!/usr/bin/env python3
import requests, logging, json, os
from cryptography.hazmat.primitives import hashes, serialization
from cryptography.hazmat.primitives.asymmetric import padding

LOG = logging.getLogger(__name__)

# load device capability once (CPU, TPU, memory in bytes)
def device_profile():
    return {"name":"jetson_agx","mem":32768*1024*1024,"features":["cuda","tensorcores","fp16"]}

# verify signature using PEM public key
def verify_signature(public_pem: bytes, payload: bytes, sig: bytes) -> bool:
    pub = serialization.load_pem_public_key(public_pem)
    try:
        pub.verify(sig, payload, padding.PKCS1v15(), hashes.SHA256())
        return True
    except Exception:
        return False

# score a model per Eq. (1)
def score_model(meta: dict, device: dict, weights=(0.6,0.25,0.15)):
    acc = float(meta["metrics"]["accuracy"])
    lat = float(meta["benchmarks"].get(device["name"], meta["benchmarks"].get("default", 1e3)))
    mem = int(meta["artifact"]["size_bytes"])
    cap = device["mem"]
    w_a,w_l,w_m = weights
    return w_a*acc - w_l*lat - w_m*(mem/cap)

# select best compatible version from registry
def select_and_fetch(registry_url: str, public_key_pem: bytes, out_dir: str):
    dev = device_profile()
    r = requests.get(f"{registry_url}/models").json()  # list of metadata urls
    best=None
    for meta_url in r:
        meta = requests.get(meta_url).json()
        # compatibility check
        if not set(meta["runtime"]["requires"]).issubset(set(dev["features"])):
            continue
        # verify signature
        sig = requests.get(meta["signature_url"]).content
        payload = json.dumps(meta["artifact"]).encode()
        if not verify_signature(public_key_pem, payload, sig):
            LOG.warning("bad signature %s", meta["id"]); continue
        s = score_model(meta, dev)
        if best is None or s>best[0]:
            best=(s,meta)
    if best is None:
        raise SystemExit("no compatible model")
    # download artifact atomically
    aurl = best[1]["artifact"]["url"]
    r = requests.get(aurl, stream=True)
    path = os.path.join(out_dir, best[1]["artifact"]["filename"] + ".tmp")
    with open(path,"wb") as f:
        for chunk in r.iter_content(65536):
            f.write(chunk)
    final = path[:-4]
    os.rename(path, final)
    return final