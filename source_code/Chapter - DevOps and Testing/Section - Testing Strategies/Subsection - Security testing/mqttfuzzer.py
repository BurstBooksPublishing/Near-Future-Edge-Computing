#!/usr/bin/env python3
# Production-ready MQTT mutation fuzzer. Use in controlled lab networks only.
import random
import time
import logging
import socket
import ssl
import paho.mqtt.client as mqtt

BROKER = "edge-gw.lab.local"  # rotate via CI secrets
PORT = 8883
TOPIC = "sensors/telemetry"
CLIENT_ID = "fuzzer-01"
CA_FILE = "/etc/ssl/certs/ca.pem"  # use signed CA
LOG = "/var/log/mqtt_fuzz.log"

logging.basicConfig(filename=LOG, level=logging.INFO,
                    format="%(asctime)s %(levelname)s %(message)s")

def random_mutation(base: bytes) -> bytes:
    # Mutate by flipping bytes, inserting length fields, or truncating.
    data = bytearray(base)
    ops = [flip_bytes, insert_garbage, truncate]
    random.choice(ops)(data)
    return bytes(data)

def flip_bytes(data: bytearray):
    for _ in range(max(1, len(data)//10)):
        i = random.randrange(len(data))
        data[i] ^= random.getrandbits(8)

def insert_garbage(data: bytearray):
    pos = random.randrange(len(data))
    data[pos:pos] = bytes(random.getrandbits(8) for _ in range(random.randint(1,8)))

def truncate(data: bytearray):
    if len(data) > 4:
        del data[random.randrange(1,len(data)) : random.randrange(1,len(data))]

def on_connect(client, userdata, flags, rc):
    logging.info("connected rc=%s", rc)

def on_disconnect(client, userdata, rc):
    logging.warning("disconnected rc=%s", rc)

def main():
    base_payload = b'{"device_id":"edge01","temp":22.5,"ts":%d}' % int(time.time())
    client = mqtt.Client(client_id=CLIENT_ID, protocol=mqtt.MQTTv311)
    client.tls_set(ca_certs=CA_FILE, certfile=None, keyfile=None,
                   cert_reqs=ssl.CERT_REQUIRED, tls_version=ssl.PROTOCOL_TLSv1_2)
    client.on_connect = on_connect
    client.on_disconnect = on_disconnect
    client.username_pw_set(username=None, password=None)  # use TLS client auth if required
    client.connect(BROKER, PORT, keepalive=60)
    client.loop_start()
    try:
        for i in range(10000):  # size per campaign; tune with Eq. (1)
            payload = random_mutation(base_payload)
            try:
                client.publish(TOPIC, payload, qos=1)
            except (socket.error, ssl.SSLError) as e:
                logging.exception("network error during publish: %s", e)
            time.sleep(0.01)  # rate control to avoid DoS
    finally:
        client.loop_stop()
        client.disconnect()

if __name__ == "__main__":
    main()