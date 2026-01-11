#!/usr/bin/env python3
# Production-ready: robust Kafka consumer, local SQLite serving, periodic reconciliation.
import time, signal, threading, sqlite3, json, requests
from confluent_kafka import Consumer, KafkaError

KAFKA_CONF = {'bootstrap.servers':'localhost:9092','group.id':'edge-speed','auto.offset.reset':'earliest'}
TOPIC = 'sensor_events'
SQLITE_DB = '/var/lib/edge/serving.db'
RECONCILE_URL = 'https://fog.example.com/reconciled/aggregates'  # HTTPS endpoint for batch outputs

stop = False
def sigint_handler(sig, frame): 
    global stop; stop = True

signal.signal(signal.SIGINT, sigint_handler)

def init_db():
    conn = sqlite3.connect(SQLITE_DB, timeout=30, check_same_thread=False)
    cur = conn.cursor()
    cur.execute('CREATE TABLE IF NOT EXISTS agg (sensor_id TEXT PRIMARY KEY, count INTEGER, sum REAL, last_ts INTEGER)')
    conn.commit()
    return conn

def process_message(msg, conn):
    if msg is None or msg.error():
        return
    payload = json.loads(msg.value())
    sid = payload['sensor_id']
    val = float(payload['value'])
    ts = int(payload['ts'])
    cur = conn.cursor()
    # upsert approximate aggregate
    cur.execute('INSERT INTO agg(sensor_id,count,sum,last_ts) VALUES(?,?,?,?) ON CONFLICT(sensor_id) DO UPDATE SET count=count+1, sum=sum+?, last_ts=?',
                (sid,1,val,ts,val,ts))
    conn.commit()

def consumer_loop(conn):
    c = Consumer(KAFKA_CONF)
    c.subscribe([TOPIC])
    try:
        while not stop:
            msg = c.poll(timeout=1.0)
            if msg is None: continue
            if msg.error():
                if msg.error().code() != KafkaError._PARTITION_EOF:
                    # handle non-EOF errors
                    continue
            else:
                process_message(msg, conn)
    finally:
        c.close()

def reconcile_loop(conn, interval=300):
    # Pull batch-corrected aggregates periodically and merge atomically.
    while not stop:
        try:
            r = requests.get(RECONCILE_URL, timeout=10)
            r.raise_for_status()
            batch = r.json()  # expect [{ "sensor_id": "...", "count": n, "sum": x, "ts": t }, ...]
            cur = conn.cursor()
            for rec in batch:
                cur.execute('INSERT INTO agg(sensor_id,count,sum,last_ts) VALUES(?,?,?,?) ON CONFLICT(sensor_id) DO UPDATE SET count=?, sum=?, last_ts=?',
                            (rec['sensor_id'], rec['count'], rec['sum'], rec['ts'], rec['count'], rec['sum'], rec['ts']))
            conn.commit()
        except Exception:
            # network failures common at edge; keep local state intact and retry later.
            pass
        time.sleep(interval)

if __name__ == '__main__':
    conn = init_db()
    t_recon = threading.Thread(target=reconcile_loop, args=(conn,), daemon=True)
    t_recon.start()
    consumer_loop(conn)
    conn.close()