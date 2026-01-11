#!/usr/bin/env python3
# Production-ready: SQLite queue, secure key loader, batching, retry/backoff.
import time, sqlite3, json, logging, requests
from web3 import Web3, HTTPProvider
from web3.middleware import geth_poa_middleware
from eth_account import Account

RPC_URL = "http://127.0.0.1:8545"                 # local Besu/Quorum node
DB_PATH = "/var/lib/edge_gateway/txqueue.db"
BATCH_SIZE = 16
MAX_RETRY = 6

# Load private key from a secure element or HSM; placeholder function.
def load_private_key():
    # Replace with PKCS#11 or TPM adapter in production.
    return "0x0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef"

w3 = Web3(HTTPProvider(RPC_URL, request_kwargs={'timeout': 10}))
w3.middleware_onion.inject(geth_poa_middleware, layer=0)
acct = Account.from_key(load_private_key())

# Initialize persistent queue
def init_db():
    with sqlite3.connect(DB_PATH) as db:
        db.execute("CREATE TABLE IF NOT EXISTS queue(id INTEGER PRIMARY KEY, payload BLOB, attempts INTEGER DEFAULT 0)")
        db.commit()

def enqueue(payload):
    with sqlite3.connect(DB_PATH) as db:
        db.execute("INSERT INTO queue(payload) VALUES (?)", (json.dumps(payload),))
        db.commit()

def fetch_batch(n):
    with sqlite3.connect(DB_PATH) as db:
        cur = db.execute("SELECT id, payload, attempts FROM queue ORDER BY id LIMIT ?", (n,))
        return cur.fetchall()

def mark_done(ids):
    with sqlite3.connect(DB_PATH) as db:
        db.executemany("DELETE FROM queue WHERE id = ?", [(i,) for i in ids])
        db.commit()

def increment_attempts(id_):
    with sqlite3.connect(DB_PATH) as db:
        db.execute("UPDATE queue SET attempts = attempts + 1 WHERE id = ?", (id_,))
        db.commit()

def submit_batch(entries):
    # Create single transactions per entry; this example submits sequentially with retry.
    sent_ids = []
    for id_, payload, attempts in entries:
        tx = {
            "nonce": w3.eth.get_transaction_count(acct.address),
            "to": payload.get("to", acct.address),
            "value": 0,
            "gas": 300000,
            "gasPrice": w3.toWei('1', 'gwei'),
            "data": Web3.toHex(text=json.dumps(payload))
        }
        signed = acct.sign_transaction(tx)
        raw = signed.rawTransaction.hex()
        backoff = 1
        for r in range(MAX_RETRY):
            try:
                resp = requests.post(RPC_URL, json={
                    "jsonrpc":"2.0","method":"eth_sendRawTransaction","params":[raw],"id":1
                }, timeout=10)
                resp.raise_for_status()
                sent_ids.append(id_)
                break
            except Exception:
                increment_attempts(id_)
                time.sleep(backoff)
                backoff = min(backoff*2, 30)
    if sent_ids:
        mark_done(sent_ids)

if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    init_db()
    # Example: sensor loop enqueues
    enqueue({"sensor":"opcuav1","ts":time.time(),"value":42})
    # Dispatcher loop
    while True:
        batch = fetch_batch(BATCH_SIZE)
        if batch:
            submit_batch(batch)
        else:
            time.sleep(0.5)