from web3 import Web3, HTTPProvider
from requests.adapters import HTTPAdapter
from urllib3.util.retry import Retry
import time, sqlite3, os
from eth_account import Account

# configure resilient HTTP session for provider
session = HTTPProvider.endpoint_uri = os.environ.get("ETH_RPC_URL", "http://127.0.0.1:8545")
provider = HTTPProvider(session, request_kwargs={"timeout": 10})
w3 = Web3(provider)

# attach requests retry to underlying adapter (pseudo; web3 internals may need custom transport)
retry_strategy = Retry(total=5, backoff_factor=0.5, status_forcelist=[502,503,504])
adapter = HTTPAdapter(max_retries=retry_strategy)
# (integration with web3 transport left to platform-specific adapter)

# simple SQLite-backed nonce locker to avoid nonce races across processes
conn = sqlite3.connect("/var/lib/eth_edge/nonce.db", check_same_thread=False)
conn.execute("CREATE TABLE IF NOT EXISTS nonce (account TEXT PRIMARY KEY, n INTEGER)")
def locked_next_nonce(account):
    cur = conn.cursor()
    cur.execute("BEGIN EXCLUSIVE")
    cur.execute("SELECT n FROM nonce WHERE account=?", (account,))
    row = cur.fetchone()
    onchain = w3.eth.get_transaction_count(account, "pending")
    if row is None:
        n = onchain
        conn.execute("INSERT INTO nonce(account,n) VALUES(?,?)", (account,n+1))
    else:
        n = max(row[0], onchain)
        conn.execute("UPDATE nonce SET n=? WHERE account=?", (n+1, account))
    conn.commit()
    return n

# sign and send with retry
def send_tx(to, value_wei, data=b"", gas=200000, gas_price=None):
    acct = Account.from_key(os.environ["EDGE_PRIV_KEY"])  # replace with secure signer
    nonce = locked_next_nonce(acct.address)
    tx = {"to": to, "value": value_wei, "data": data, "nonce": nonce,
          "gas": gas, "chainId": int(os.environ.get("CHAIN_ID", "1"))}
    tx["gasPrice"] = gas_price or w3.eth.gas_price
    signed = acct.sign_transaction(tx)
    for attempt in range(6):
        try:
            txhash = w3.eth.send_raw_transaction(signed.rawTransaction)
            return txhash.hex()
        except Exception as e:
            time.sleep(0.5 * 2**attempt)
    raise RuntimeError("tx submission failed after retries")