#!/usr/bin/env python3
# Production-ready edge agent: async read, timestamp, filter, publish over TLS MQTT.
import asyncio, json, ssl, time
from collections import deque
import serial_asyncio
import paho.mqtt.client as mqtt

MQTT_BROKER = "edge-broker.local"
MQTT_PORT = 8883
MQTT_TOPIC = "edge/qsensor/telemetry"
TLS_CERT = "/etc/ssl/certs/ca.pem"

# Exponential smoothing parameter for low-latency filtering.
ALPHA = 0.2
# Use monotonic_ns for elapsed times, time.time_ns() for wall-clock PTP-synced timestamp.
def ptp_timestamp_ns():
    return time.time_ns()  # assumes host PTP or CSAC provides synchronized wall time

class QSensorProtocol(asyncio.Protocol):
    def __init__(self, process_cb):
        self.buffer = bytearray()
        self.process_cb = process_cb
    def data_received(self, data):
        self.buffer.extend(data)
        while b'\n' in self.buffer:
            line, _, self.buffer = self.buffer.partition(b'\n')
            try:
                payload = line.decode('utf-8').strip()
                asyncio.create_task(self.process_cb(payload))
            except Exception:
                pass

async def process_line(line, mqtt_client, state):
    # Example input: "B:12.345" where B is magnetic flux density in nT.
    try:
        if not line: return
        key, val = line.split(':',1)
        meas = float(val)
    except Exception:
        return
    ts = ptp_timestamp_ns()
    # low-complexity exponential filter for edge ML preprocessing
    if state['sm'] is None:
        state['sm'] = meas
    else:
        state['sm'] = ALPHA * meas + (1-ALPHA) * state['sm']
    msg = {
        "ts_ns": ts,
        "sensor": "nv_magnetometer",
        "raw": meas,
        "smoothed": state['sm']
    }
    mqtt_client.publish(MQTT_TOPIC, json.dumps(msg), qos=1)

def mqtt_connect():
    ssl_ctx = ssl.create_default_context(cafile=TLS_CERT)
    client = mqtt.Client(client_id="qsensor-edge-agent")
    client.tls_set_context(ssl_ctx)
    client.username_pw_set("edgeuser","REPLACE_WITH_SECURE_PASSWORD")
    client.connect(MQTT_BROKER, MQTT_PORT, keepalive=60)
    client.loop_start()
    return client

async def main():
    mqtt_client = mqtt_connect()
    state = {'sm': None}
    loop = asyncio.get_running_loop()
    # serial device provided by MCU running Zephyr controlling NV sensor head
    transport, protocol = await serial_asyncio.create_serial_connection(
        loop, lambda: QSensorProtocol(lambda l: process_line(l,mqtt_client,state)),
        '/dev/ttyUSB0', baudrate=115200
    )
    while True:
        await asyncio.sleep(3600)

if __name__ == "__main__":
    asyncio.run(main())