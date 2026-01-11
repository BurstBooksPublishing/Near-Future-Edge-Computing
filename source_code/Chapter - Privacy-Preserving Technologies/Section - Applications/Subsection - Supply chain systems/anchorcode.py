from web3 import Web3
import hashlib, json, time
# Connect to local geth (light) node on gateway
w3 = Web3(Web3.HTTPProvider("http://127.0.0.1:8545", request_kwargs={'timeout':30}))

def merkle_root(leaves):
    # leaves: list of byte strings
    def h(x): return hashlib.sha256(x).digest()
    nodes = [h(x) for x in leaves]
    while len(nodes) > 1:
        if len(nodes) % 2: nodes.append(nodes[-1])  # duplicate odd
        nodes = [h(nodes[i] + nodes[i+1]) for i in range(0, len(nodes), 2)]
    return nodes[0]

# Example: pack sensor readings for batch
batch = [{"id":"sensor-01","ts":int(time.time()),"temp":4.1},
         {"id":"sensor-02","ts":int(time.time()),"temp":3.9}]
leaves = [json.dumps(r, separators=(',',':')).encode() for r in batch]
root = merkle_root(leaves).hex()

# Build transaction payload for smart contract anchor function
contract_address = "0x..."  # consortium contract
abi = [...]  # contract ABI array
contract = w3.eth.contract(address=contract_address, abi=abi)
tx_data = contract.functions.anchorBatch(root, len(batch)).buildTransaction({
    'chainId': 1337, 'gas': 200000, 'gasPrice': w3.toWei('1','gwei'),
    'nonce': w3.eth.get_transaction_count(w3.eth.default_account)
})

# Sign using PKCS#11 HSM or local secure element (ATECC) via PKCS#11 library
# PKCS11_SIGN(tx_hash) must return signature bytes from secure element
# Replace with appropriate PKCS11 wrapper for production
signed = PKCS11_SIGN(w3.eth.account._encode_transaction(tx_data))  # placeholder
raw_tx = w3.eth.account._encode_transaction(tx_data) + signed
tx_hash = w3.eth.send_raw_transaction(raw_tx)
print("Anchor tx sent:", tx_hash.hex())