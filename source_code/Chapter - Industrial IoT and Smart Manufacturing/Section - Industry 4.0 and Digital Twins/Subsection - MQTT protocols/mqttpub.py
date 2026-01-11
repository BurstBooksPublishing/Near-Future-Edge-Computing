import json, time, ssl, random
import paho.mqtt.client as mqtt
from paho.mqtt.properties import Properties
from paho.mqtt.packettypes import PacketTypes

BROKER = "edge-broker.local"
PORT = 8883
CLIENT_ID = "edge-gateway-01"
TOPIC = "plant/line1/digital_twin/state"
CA_CERT = "ca.pem"          # path to CA
CLIENT_CERT = "client.crt"  # client certificate
CLIENT_KEY = "client.key"   # client private key

def make_props(expiry=30):
    props = Properties(PacketTypes.PUBLISH)
    props.MessageExpiryInterval = expiry
    return props

def on_connect(client, userdata, flags, rc, properties=None):
    # broker connected; subscribe if needed
    pass

client = mqtt.Client(client_id=CLIENT_ID, protocol=mqtt.MQTTv5)
client.tls_set(ca_certs=CA_CERT, certfile=CLIENT_CERT,
               keyfile=CLIENT_KEY, tls_version=ssl.PROTOCOL_TLSv1_2)
client.tls_insecure_set(False)
client.on_connect = on_connect
client.will_set("plant/line1/availability", payload="offline",
                qos=1, retain=True)

backoff = 1.0
while True:
    try:
        client.connect(BROKER, PORT, keepalive=60)
        client.loop_start()
        backoff = 1.0
        break
    except Exception:
        time.sleep(backoff)
        backoff = min(60.0, backoff * 2)

def publish_state(state):
    payload = json.dumps(state).encode("utf-8")
    props = make_props(expiry=60)  # avoid stale twin updates
    # use QoS 1 for critical updates to reduce state divergence
    client.publish(TOPIC, payload=payload, qos=1, retain=True, properties=props)

# loop: gather sensor/PLC data and publish
try:
    while True:
        state = {"timestamp": time.time(),
                 "motor_rpm": 1500 + random.randint(-5,5),
                 "status": "running"}
        publish_state(state)
        time.sleep(1.0)
finally:
    client.loop_stop()
    client.disconnect()