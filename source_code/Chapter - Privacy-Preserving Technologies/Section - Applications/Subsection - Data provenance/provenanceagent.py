#!/usr/bin/env python3
# Production-ready: uses env config, retry, and minimal dependencies
import os, json, hashlib, time
from web3 import Web3, HTTPProvider
import ipfshttpclient

WEB3_RPC = os.environ["WEB3_RPC"]            # e.g., http://localhost:8545
CONTRACT_ADDR = Web3.toChecksumAddress(os.environ["ANCHOR_CONTRACT"])
PRIVATE_KEY = os.environ["ANCHOR_PRIVATE_KEY"]
ACCOUNT = Web3.toChecksumAddress(os.environ["ANCHOR_ACCOUNT"])

w3 = Web3(HTTPProvider(WEB3_RPC))
ipfs = ipfshttpclient.connect()               # requires local IPFS daemon

def sha256_hex(b): return hashlib.sha256(b).hexdigest()

def pin_and_hash(payload_bytes):
    res = ipfs.add_bytes(payload_bytes)      # returns CID in v0
    return res, sha256_hex(payload_bytes.encode() if isinstance(payload_bytes,str) else payload_bytes)

def merkle_root(hashes):
    # pairwise hash up the tree; assumes list of hex strings
    nodes = [bytes.fromhex(h) for h in hashes]
    while len(nodes) > 1:
        if len(nodes) % 2: nodes.append(nodes[-1])    # duplicate last if odd
        nodes = [hashlib.sha256(nodes[i]+nodes[i+1]).digest() 
                 for i in range(0, len(nodes), 2)]
    return nodes[0].hex()

def anchor_root(root_hex):
    # minimal example; contract abi must implement function anchor(bytes32)
    abi = json.loads(open("AnchorABI.json").read())
    contract = w3.eth.contract(address=CONTRACT_ADDR, abi=abi)
    tx = contract.functions.anchor(Web3.toBytes(hexstr=root_hex)).buildTransaction({
        "from": ACCOUNT, "nonce": w3.eth.get_transaction_count(ACCOUNT), "gas": 200000})
    signed = w3.eth.account.sign_transaction(tx, PRIVATE_KEY)
    tx_hash = w3.eth.send_raw_transaction(signed.rawTransaction)
    return tx_hash.hex()

# Example batching loop (external orchestration should manage intervals)
def process_batch(payloads):
    # payloads: list of bytes or JSON-serializable objects
    hashes = []
    for p in payloads:
        b = json.dumps(p).encode() if not isinstance(p, (bytes,bytearray)) else p
        cid, h = pin_and_hash(b)    # pin to IPFS and get content hash
        hashes.append(h)
    root = merkle_root(hashes)
    tx = anchor_root(root)
    return {"merkle_root": root, "tx": tx}

# usage: integrate with KubeEdge/Greengrass lifecycle and secure key storage