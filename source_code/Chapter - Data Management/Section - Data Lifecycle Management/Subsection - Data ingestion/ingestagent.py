#!/usr/bin/env python3
import asyncio, json, ssl, time, sqlite3
from collections import deque
from confluent_kafka import Producer
from asyncio_mqtt import Client  # pip: asyncio-mqtt

# TLS and broker config (fill with certificates)
MQTT_BROKER = "mqtt.example.local"
MQTT_PORT = 8883
TOPIC = "factory/vibration"
KAFKA_TOPIC = "edge.ingest"
KAFKA_CONF = {"bootstrap.servers":"kafka-broker:9093","security.protocol":"SSL",
              "ssl.ca.location":"/etc/ssl/ca.pem","ssl.certificate.location":"/etc/ssl/cert.pem",
              "ssl.key.location":"/etc/ssl/key.pem"}
DB_PATH = "/var/lib/ingest/queue.db"

# durable queue helper
def init_db():
    conn = sqlite3.connect(DB_PATH, isolation_level=None)
    c = conn.cursor()
    c.execute("CREATE TABLE IF NOT EXISTS batches(id INTEGER PRIMARY KEY AUTOINCREMENT, payload BLOB, sent INT DEFAULT 0)")
    return conn

# adaptive batch controller
class BatchController:
    def __init__(self, target_lat_ms=100, min_B=8, max_B=1024, alpha=0.1):
        self.target_lat = target_lat_ms/1000.0
        self.min_B, self.max_B = min_B, max_B
        self.alpha = alpha
        self.lambda_hat = 1.0  # samples/s smoothed
        self.window = deque(maxlen=1000)
        self.last_ts = None

    def add_sample(self):
        now = time.time()
        if self.last_ts is not None:
            isi = now - self.last_ts
            inst_rate = 1.0 / max(isi, 1e-6)
            self.lambda_hat = (1-self.alpha)*self.lambda_hat + self.alpha*inst_rate
        self.last_ts = now

    def batch_size(self):
        B = int(2 * self.lambda_hat * self.target_lat)
        return max(self.min_B, min(B, self.max_B))

async def mqtt_collector(db_conn, controller):
    producer = Producer(KAFKA_CONF)
    async with Client(MQTT_BROKER, port=MQTT_PORT, tls_context=ssl.create_default_context()) as client:
        async with client.unfiltered_messages() as messages:
            await client.subscribe(TOPIC)
            buffer = []
            while True:
                msg = await messages.get()
                controller.add_sample()
                buffer.append(msg.payload)
                B = controller.batch_size()
                if len(buffer) >= B:
                    payload = json.dumps({"ts":time.time(),"records":[b.decode() for b in buffer]}).encode()
                    # persist batch
                    db_conn.execute("INSERT INTO batches(payload) VALUES(?)",(payload,))
                    buffer.clear()
                    # attempt send pending batches
                    await send_pending(db_conn, producer)

async def send_pending(db_conn, producer):
    # fetch unsent batches, attempt send, mark sent on success
    cur = db_conn.execute("SELECT id,payload FROM batches WHERE sent=0 ORDER BY id LIMIT 16")
    rows = cur.fetchall()
    for id_, payload in rows:
        try:
            producer.produce(KAFKA_TOPIC, payload)
            producer.flush(timeout=1.0)
            db_conn.execute("UPDATE batches SET sent=1 WHERE id=?",(id_,))
        except Exception:
            await asyncio.sleep(1)  # backoff on failure
            break

def main():
    db_conn = init_db()
    controller = BatchController()
    asyncio.run(mqtt_collector(db_conn, controller))

if __name__ == "__main__":
    main()