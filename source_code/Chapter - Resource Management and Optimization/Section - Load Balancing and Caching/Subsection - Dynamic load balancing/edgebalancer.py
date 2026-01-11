import asyncio, aiohttp, time, math, random
# Production parameters
PROM_URL = "http://prometheus:9090/api/v1/query"
SCRAPE_INTERVAL = 2.0
SCORE_DECAY = 0.8  # smooth scores to avoid oscillation
NODES = [                                                           
    {"name":"jetson-1","addr":"http://10.0.0.11:8080","has_gpu":True},
    {"name":"rpi-gw","addr":"http://10.0.0.12:8080","has_gpu":False},
    {"name":"microdc","addr":"http://10.0.0.13:8080","has_gpu":True},
]
# Query Prometheus for queue length and RTT metrics; returns float or None
async def prom_query(session, query):
    async with session.get(PROM_URL, params={"query":query}, timeout=5) as r:
        j = await r.json()
        try:
            return float(j["data"]["result"][0]["value"][1])
        except Exception:
            return None

async def collect_metrics():
    async with aiohttp.ClientSession() as s:
        tasks = []
        for n in NODES:
            q_query = f'node_queue_length{{instance="{n["addr"].split("//")[1]}"}}'
            rtt_query = f'node_rtt_ms{{instance="{n["addr"].split("//")[1]}"}}'
            tasks.append(prom_query(s, q_query))
            tasks.append(prom_query(s, rtt_query))
        vals = await asyncio.gather(*tasks)
        for i,n in enumerate(NODES):
            q = vals[2*i]; rtt = vals[2*i+1]
            # normalized score: lower is better
            norm_q = q if q is not None else 1e6
            norm_rtt = (rtt/1000.0) if rtt is not None else 10.0
            raw = norm_q + 10.0*norm_rtt
            n.setdefault("score", raw if "score" not in n else SCORE_DECAY*n["score"] + (1-SCORE_DECAY)*raw)

# Choose node via softmax over negative scores, optionally filter by capability
def choose_node(require_gpu=False):
    candidates = [n for n in NODES if (not require_gpu) or n["has_gpu"]]
    scores = [math.exp(-n["score"]) for n in candidates]
    s = sum(scores)
    if s == 0: return random.choice(candidates)
    probs = [p/s for p in scores]
    return random.choices(candidates, weights=probs, k=1)[0]

# Route task payload to selected node; returns response or raises
async def route_task(node, payload):
    async with aiohttp.ClientSession() as s:
        async with s.post(node["addr"] + "/v1/process", json=payload, timeout=3) as r:
            r.raise_for_status()
            return await r.json()

# Main loop: collect metrics periodically and handle incoming tasks from a queue
async def main_loop(task_queue):
    async def metrics_loop():
        while True:
            await collect_metrics()
            await asyncio.sleep(SCRAPE_INTERVAL)
    async def worker():
        while True:
            payload = await task_queue.get()
            node = choose_node(require_gpu=payload.get("gpu", False))
            try:
                resp = await route_task(node, payload)
            except Exception as e:
                # simple retry policy on failure
                node2 = choose_node(require_gpu=payload.get("gpu", False))
                resp = await route_task(node2, payload)
            # handle response (ack, SLO logging)
            task_queue.task_done()
    await asyncio.gather(metrics_loop(), worker())

# Entry point
if __name__ == "__main__":
    q = asyncio.Queue()
    # In production, tasks come from MQTT, gRPC, or HTTP webhooks; here we simulate.
    async def producer():
        while True:
            await q.put({"frame_id":int(time.time()*1000), "gpu":random.random()<0.5})
            await asyncio.sleep(0.02)
    asyncio.run(asyncio.gather(producer(), main_loop(q)))