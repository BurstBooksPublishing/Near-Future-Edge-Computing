#!/usr/bin/env python3
# asyncio-mqtt based RTT tester: publish, await reply, compute percentiles.
import asyncio
import time
import statistics
from asyncio_mqtt import Client, MqttError

BROKER = "192.168.1.10"        # gateway IP or local broker
PUB_TOPIC = "edge/test/request"
SUB_TOPIC = "edge/test/response"
RATE = 100                     # messages per second
COUNT = 5000                   # total messages to send

async def worker():
    latencies = []
    try:
        async with Client(BROKER) as client:
            messages = asyncio.Queue()
            # subscribe and push incoming timestamps to queue
            async with client.unfiltered_messages() as stream:
                await client.subscribe(SUB_TOPIC)
                async def reader():
                    async for msg in stream:
                        # payload is sent timestamp as bytes
                        sent_ts = float(msg.payload.decode())
                        latencies.append((time.time() - sent_ts)*1000.0)  # ms
                rtask = asyncio.create_task(reader())
                # publish loop at fixed rate
                interval = 1.0 / RATE
                for i in range(COUNT):
                    t0 = time.time()
                    await client.publish(PUB_TOPIC, f"{t0}", qos=1)
                    await asyncio.sleep(interval)
                # allow trailing responses, then cancel reader
                await asyncio.sleep(2.0)
                rtask.cancel()
    except MqttError as e:
        print("MQTT error", e)
    # compute statistics
    if latencies:
        lat_ms = [l for l in latencies]
        print("count:", len(lat_ms))
        print("P50:", statistics.median(lat_ms))
        print("P95:", percentile(lat_ms,95))
        print("P99:", percentile(lat_ms,99))
    else:
        print("no responses recorded")

def percentile(data, p):
    k = max(0, min(len(data)-1, int(round((p/100.0)*(len(data)-1)))))
    return sorted(data)[k]

if __name__ == "__main__":
    asyncio.run(worker())