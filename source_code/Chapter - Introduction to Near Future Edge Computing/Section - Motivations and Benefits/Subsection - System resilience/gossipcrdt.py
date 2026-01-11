#!/usr/bin/env python3
# Lightweight gossip-based CRDT sync for edge nodes.
import asyncio, json, os, socket, time
from typing import Dict

GOSSIP_PORT = 50000
GOSSIP_PEERS = os.environ.get("PEERS","").split(",")  # comma list of addr:port
NODE_ID = os.environ.get("NODE_ID","node0")
SNAPSHOT_FILE = f"/var/lib/crdt_{NODE_ID}.json"

# local G-Counter state
state: Dict[str,int] = {}

def load_state():
    try:
        with open(SNAPSHOT_FILE,'r') as f:
            return json.load(f)
    except Exception:
        return {}

def save_state():
    with open(SNAPSHOT_FILE,'w') as f:
        json.dump(state,f)

def merge(remote: Dict[str,int]):
    for k,v in remote.items():
        state[k] = max(state.get(k,0), v)

async def gossip_sender():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    while True:
        payload = json.dumps({"node":NODE_ID, "state":state, "ts":time.time()}).encode()
        for peer in GOSSIP_PEERS:
            if not peer: continue
            host,port = peer.split(":")
            sock.sendto(payload, (host,int(port)))
        await asyncio.sleep(1.0)  # adjustable period

async def gossip_listener():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", GOSSIP_PORT))
    sock.setblocking(False)
    loop = asyncio.get_running_loop()
    while True:
        data, addr = await loop.sock_recvfrom(sock, 65536)
        try:
            msg = json.loads(data.decode())
            merge(msg.get("state",{}))
            save_state()  # fast local persistence
        except Exception:
            continue

async def tick_increment():
    # application increments its own component counter
    while True:
        state.setdefault(NODE_ID,0)
        state[NODE_ID] += 1
        save_state()
        await asyncio.sleep(5.0)  # sampling or event-driven in real apps

async def main():
    global state
    state = load_state()
    await asyncio.gather(gossip_sender(), gossip_listener(), tick_increment())

if __name__=="__main__":
    asyncio.run(main())