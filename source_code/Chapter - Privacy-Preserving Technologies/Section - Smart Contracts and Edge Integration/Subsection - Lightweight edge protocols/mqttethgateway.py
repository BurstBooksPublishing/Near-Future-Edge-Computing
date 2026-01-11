#!/usr/bin/env python3
# MQTT -> local geth light-node bridge. Use HSM for key material in production.
import time, cbor2, logging
from paho.mqtt import client as mqtt
from web3 import Web3, HTTPProvider
from eth_account import Account

MQTT_BROKER = "127.0.0.1"
MQTT_PORT = 1883
MQTT_TOPIC = "factory/sensors/+/telemetry"
GETH_RPC = "http://127.0.0.1:8545"  # local geth in light mode

logging.basicConfig(level=logging.INFO)
w3 = Web3(HTTPProvider(GETH_RPC, request_kwargs={'timeout':10}))

# Replace with secure key retrieval from HSM; Account.privateKeyToAccount used for demo.
PRIVATE_KEY = "0x0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef"
acct = Account.from_key(PRIVATE_KEY)

def build_payload(decoded):
    # Construct compact contract call data; use ABI encode in real code.
    return {
        "device_id": decoded["id"],
        "ts": decoded["t"],
        "m": decoded["m"]  # measurement
    }

def send_tx(payload):
    # Example: contract at ADDRESS with method 'report(bytes)'
    contract_address = "0xdeadbeefdeadbeefdeadbeefdeadbeefdeadbeef"
    # Use ABI encoding libraries; here we use raw data field for compactness.
    tx = {
        "to": contract_address,
        "value": 0,
        "gas": 200000,
        "gasPrice": w3.toWei('2', 'gwei'),
        "nonce": w3.eth.get_transaction_count(acct.address),
        "data": w3.toHex(text=str(payload))
    }
    signed = acct.sign_transaction(tx)
    tx_hash = w3.eth.send_raw_transaction(signed.rawTransaction)
    logging.info("submitted tx %s", tx_hash.hex())
    return tx_hash

def on_message(client, userdata, msg):
    try:
        decoded = cbor2.loads(msg.payload)  # CBOR is compact binary
        # Verify device HMAC or attestation here (omitted, implement in prod).
        payload = build_payload(decoded)
        send_tx(payload)
    except Exception as e:
        logging.exception("processing failed: %s", e)

def main():
    client = mqtt.Client()
    client.on_message = on_message
    client.connect(MQTT_BROKER, MQTT_PORT, keepalive=60)
    client.subscribe(MQTT_TOPIC)
    client.loop_start()
    try:
        while True: time.sleep(1)
    except KeyboardInterrupt:
        client.loop_stop()

if __name__ == "__main__":
    main()