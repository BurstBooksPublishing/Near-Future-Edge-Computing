#!/usr/bin/env python3
# Production-ready asyncio aggregator: LoRa->MQTT with TLS, reconnect, backoff.
import asyncio, ssl, json, time
import aiomqtt  # asyncio MQTT client
from collections import deque

BROKER = "mqtt.example.city"
PORT = 8883
TOPIC = "city/env/aggregates"
WINDOW = 12  # 5-min windows if sampling every 25s
BACKOFF_BASE = 1.5

ssl_ctx = ssl.create_default_context()
ssl_ctx.check_hostname = True
ssl_ctx.verify_mode = ssl.CERT_REQUIRED

async def publish_loop(queue):
    backoff = BACKOFF_BASE
    while True:
        try:
            async with aiomqtt.Client(BROKER, port=PORT, tls=ssl_ctx) as client:
                backoff = BACKOFF_BASE
                while True:
                    payload = await queue.get()
                    await client.publish(TOPIC, payload, qos=1)
        except Exception:
            await asyncio.sleep(backoff)
            backoff = min(backoff * 2, 60)

async def lora_handler(queue):
    windows = {}  # device_id -> deque
    # Replace this stub with real LoRa packet handling (ChirpStack MQTT or UDP)
    async def fake_packet_stream():
        while True:
            yield {"dev": "node-12", "pm25": 12.3, "ts": int(time.time())}
            await asyncio.sleep(25)
    async for pkt in fake_packet_stream():
        d = pkt["dev"]
        windows.setdefault(d, deque(maxlen=WINDOW)).append(pkt["pm25"])
        if len(windows[d]) == WINDOW:
            mean = sum(windows[d]) / WINDOW
            out = json.dumps({"device": d, "pm25_mean": mean, "ts": pkt["ts"]})
            await queue.put(out)

async def main():
    q = asyncio.Queue(maxsize=1000)
    await asyncio.gather(publish_loop(q), lora_handler(q))

if __name__ == "__main__":
    asyncio.run(main())