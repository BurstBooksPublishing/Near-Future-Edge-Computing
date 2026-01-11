import gzip, queue, threading, time, requests, os

BATCH_MAX   = 2048          # points
FLUSH_INTERVAL = 5.0        # seconds
INFLUX_URL  = "http://192.168.1.10:8086/write?db=edge"

persist_q = queue.Queue()
def persist_enqueue(batch):
    with open("/var/spool/edge/persist.q", "a") as f:
        f.write("\n".join(batch)+"\n")

def batch_sender(gen):
    buffer, last_flush = [], time.time()
    for point in gen:
        buffer.append(point)
        if len(buffer)>=BATCH_MAX or (time.time()-last_flush)>=FLUSH_INTERVAL or point is None):
                persist_enqueue(buffer)           # durable local copy
                # compress batch
                payload = ("\n".join(buffer)).encode("utf-8")
                gz = gzip.compress(payload, compresslevel=3)
                # send with retry/backoff
                for attempt in range(6):
                    try:
                        http_post(gz)
                        backoff = 1.0
                        break
                    except Exception:
                        time.sleep(backoff)
                        backoff = min(backoff*2, 60.0)
                buffer.clear()
                last_flush = time.time()
            if point is None:
                break
        except Exception:
            time.sleep(1)

# Example point generator for industrial sensors
def point_generator():
    while True:
        ts = int(time.time() * 1e9)
        for ch in range(32):
            # line protocol: measurement,tag=val field=val timestamp
            yield f"vibration,ch={ch} value={0.0} {ts}"
        time.sleep(0.001)  # sample interval

# start sender thread
Thread(target=batch_sender, args=(point_generator(),), daemon=True).start()