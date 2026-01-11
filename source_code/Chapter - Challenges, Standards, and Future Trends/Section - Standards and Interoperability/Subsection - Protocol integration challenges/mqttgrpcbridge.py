#!/usr/bin/env python3
import asyncio
import ssl
from asyncio_mqtt import Client, MqttError         # lightweight asyncio MQTT client
import grpc
import telemetry_pb2, telemetry_pb2_grpc          # generated Protobuf/gRPC stubs

MQTT_BROKER = "mqtt-broker.local"
MQTT_TOPIC = "devices/+/telemetry"
GRPC_TARGET = "mec-node.local:50051"
QUEUE_MAX = 1000

async def mqtt_consumer(queue: asyncio.Queue):
    ssl_ctx = ssl.create_default_context()        # configure CA/certs in production
    async with Client(MQTT_BROKER, ssl=ssl_ctx) as client:
        async with client.unfiltered_messages() as messages:
            await client.subscribe(MQTT_TOPIC)
            async for msg in messages:
                payload = msg.payload                # binary payload
                await queue.put((msg.topic, payload)) # backpressure via queue

async def grpc_producer(queue: asyncio.Queue):
    # Keep gRPC channel alive; enable mTLS in production
    async with grpc.aio.insecure_channel(GRPC_TARGET) as channel:
        stub = telemetry_pb2_grpc.TelemetrySinkStub(channel)
        while True:
            topic, payload = await queue.get()
            # deserialize, validate, map fields (type-safety)
            telem = telemetry_pb2.Telemetry()        # Protobuf message
            telem.ParseFromString(payload)          # assume Protobuf payload from device
            try:
                await stub.IngestTelemetry(telem, timeout=2.0)  # RPC with deadline
            except grpc.aio.AioRpcError:
                # simple retry/backoff could be implemented here
                pass
            finally:
                queue.task_done()

async def main():
    queue = asyncio.Queue(maxsize=QUEUE_MAX)
    producers = [mqtt_consumer(queue), grpc_producer(queue)]
    await asyncio.gather(*producers)

if __name__ == "__main__":
    asyncio.run(main())