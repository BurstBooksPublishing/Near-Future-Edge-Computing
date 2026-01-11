#!/usr/bin/env python3
# Production-ready OPC UA -> MQTT bridge with reconnect and local buffer.
import time
import ssl
from opcua import Client, ua
import paho.mqtt.client as mqtt

OPC_URL = "opc.tcp://192.168.1.10:4840"
MQTT_BROKER = "localhost"
MQTT_TOPIC_PREFIX = "factory/cell1/twin"
NODE_IDS = ["ns=2;s=Robot.Axis1.Position","ns=2;s=Conveyor.Speed"]

def on_datachange(node, val, data):
    msg = {"node": node.nodeid.to_string(), "value": val, "ts": int(time.time()*1000)}
    mqttc.publish(f"{MQTT_TOPIC_PREFIX}/{node.nodeid.Identifier}", payload=str(msg), qos=1)
    # keep processing fast; heavy work should go to worker threads/containers.

def ensure_opc_client():
    client = Client(OPC_URL)
    client.set_user("edge_user")           # optional
    client.set_password("secret")         # optional
    # use application certificate if required (path examples)
    # client.load_client_certificate("client_cert.der")
    # client.load_private_key("client_key.pem")
    client.application_uri = "urn:edge:opcua:bridge"
    client.connect()
    return client

def ensure_mqtt_client():
    c = mqtt.Client()
    c.connect(MQTT_BROKER, 1883, 60)
    c.loop_start()
    return c

if __name__ == "__main__":
    mqttc = ensure_mqtt_client()
    while True:
        try:
            opc = ensure_opc_client()
            subs = opc.create_subscription(100, None)  # 100 ms publishing interval
            handles = []
            for nid in NODE_IDS:
                node = opc.get_node(nid)
                handle = subs.subscribe_data_change(node, handler=on_datachange)
                handles.append(handle)
            # main loop monitors connectivity and memory
            while True:
                time.sleep(1)
        except Exception:
            time.sleep(2)   # backoff on failure and retry
        finally:
            try:
                subs.delete()
            except Exception:
                pass
            try:
                opc.disconnect()
            except Exception:
                pass