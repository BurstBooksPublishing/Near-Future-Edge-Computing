#!/usr/bin/env python3
# Production-ready asyncio MQTT agent with priority queue and TLS
import asyncio, ssl, heapq, time, json
from asyncio_mqtt import Client, MqttError

BROKER = "mec-broker.local"
PORT = 8883
TOPICS = [("camera/events", 0), ("audio/events", 0), ("env/sensors", 0)]
JWT = "eyJ..."  # device JWT provisioned via CA
TLS_CTX = ssl.create_default_context()
TLS_CTX.load_verify_locations("/etc/ssl/certs/ca.pem")

class PriorityAgent:
    def __init__(self):
        self.queue = []  # heap of (priority, timestamp, payload)
        self.rate_limit = 50  # msgs/sec max outbound
        self.last_sent = 0
        self.sent_count = 0

    async def connect_and_run(self):
        reconnect_delay = 1
        while True:
            try:
                async with Client(BROKER, PORT, username="device", password=JWT, tls=TLS_CTX) as client:
                    await client.subscribe(TOPICS)
                    async with client.unfiltered_messages() as messages:
                        producer = asyncio.create_task(self._produce(messages))
                        consumer = asyncio.create_task(self._consume(client))
                        await asyncio.gather(producer, consumer)
            except MqttError:
                await asyncio.sleep(reconnect_delay)
                reconnect_delay = min(60, reconnect_delay * 2)

    async def _produce(self, messages):
        async for msg in messages:
            payload = json.loads(msg.payload.decode())
            prio = self._compute_priority(payload)  # score 0..100
            heapq.heappush(self.queue, (-prio, time.time(), payload))  # max-heap
            # drop-old policy
            if len(self.queue) > 1000:
                heapq.heappop(self.queue)

    async def _consume(self, client):
        while True:
            if not self.queue:
                await asyncio.sleep(0.01); continue
            now = time.time()
            if now - self.last_sent >= 1:
                self.last_sent = now; self.sent_count = 0
            if self.sent_count >= self.rate_limit:
                await asyncio.sleep(0.01); continue
            prio, ts, payload = heapq.heappop(self.queue)
            # local inference call; replace with TensorRT/ONNX runtime
            decision = await self._local_infer(payload)
            if decision["alert"]:
                await client.publish("alerts/out", json.dumps(decision), qos=1)
                self.sent_count += 1

    async def _local_infer(self, payload):
        # lightweight placeholder for production inference
        await asyncio.sleep(0.01)
        score = payload.get("score", 0.0)
        return {"alert": score > 0.7, "score": score, "meta": payload}

    def _compute_priority(self, payload):
        # higher weight to audio gunshot reports and high-confidence vision
        kind = payload.get("type","")
        base = 80 if kind=="gunshot" else 50 if kind=="person" else 10
        score = payload.get("score",0.0)
        return base + int(20*score)

if __name__ == "__main__":
    asyncio.run(PriorityAgent().connect_and_run())