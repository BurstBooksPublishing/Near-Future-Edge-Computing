#!/usr/bin/env bash
# Start a single-node Redpanda container with resource limits and persistent storage.
docker run -d --name redpanda \
  --restart unless-stopped \
  --memory=1g --cpus=0.75 \
  -v /var/lib/redpanda:/var/lib/redpanda \
  -p 9092:9092 -p 9644:9644 \
  vectorized/redpanda:latest \
  redpanda start --overprovisioned --node-id 0 --memory=512M

# Wait for broker readiness (simple TCP check).
for i in {1..30}; do
  nc -z localhost 9092 && break
  sleep 1
done

# Small, robust Python producer using confluent_kafka.
python3 - <<'PY'
# Production-ready edges: use confluent_kafka for librdkafka performance.
from confluent_kafka import Producer
import socket, json, time

conf = {
    'bootstrap.servers': 'localhost:9092',
    'client.id': f'edge-producer-{socket.gethostname()}',
    'enable.idempotence': True,        # avoid duplicates on retries
    'acks': 'all',                     # wait for ISR (stronger durability)
    'compression.type': 'lz4',         # low-latency compression
    'linger.ms': 50,                   # small batching window
    'batch.num.messages': 200,         # batching control
    'queue.buffering.max.kbytes': 10240,
    'retries': 5,
}

p = Producer(conf)

def delivery(err, msg):
    if err is not None:
        # In production, escalate via local alerting/metrics.
        print(f"Delivery failed: {err}")
    else:
        # Optional: record offsets to local SQLite for reconcilers.
        print(f"Delivered topic={msg.topic()} partition={msg.partition()} offset={msg.offset()}")

topic = 'edge.telemetry'
try:
    for n in range(1000):
        payload = json.dumps({'device':'sensor-01','ts': time.time(), 'value': n})
        p.produce(topic, payload.encode('utf-8'), callback=delivery)
        p.poll(0)  # serve delivery callbacks
        time.sleep(0.01)  # simulate sensor sampling
    p.flush(10)
finally:
    p.flush()
PY