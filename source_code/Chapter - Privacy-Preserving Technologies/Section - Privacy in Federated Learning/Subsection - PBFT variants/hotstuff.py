import asyncio
import aiohttp
from aiohttp import web
import json, time
# Configuration (node_id, peer URLs) injected at startup
NODE_ID = int(os.environ.get("NODE_ID", "0"))
PEERS = json.loads(os.environ.get("PEERS_JSON","[]"))
QUORUM = lambda n: (2*(n-1)//3)+1  # compute 2f+1 from n

# Simple in-memory state
pending = {}
committed = set()

async def sign(payload: bytes) -> str:
    # production: use HSM/TEE-backed key (PKCS#11 or TPM)
    return await asyncio.get_running_loop().run_in_executor(None, lambda: "sig"+str(hash(payload)))

async def broadcast(path, body):
    async with aiohttp.ClientSession() as sess:
        tasks = [sess.post(peer+path, json=body, timeout=3) for peer in PEERS]
        await asyncio.gather(*tasks, return_exceptions=True)

async def handle_propose(req):
    data = await req.json()
    slot = data['slot']; proposal = data['proposal']
    sig = await sign(proposal.encode())
    pending.setdefault(slot, {'proposal':proposal, 'prepares':{}, 'commits':{}})
    # send prepare with signature
    body = {'slot':slot, 'proposal':proposal, 'from':NODE_ID, 'sig':sig}
    await broadcast('/prepare', body)
    return web.json_response({'status':'ok'})

async def handle_prepare(req):
    data = await req.json()
    slot = data['slot']; from_id = data['from']; sig = data['sig']
    st = pending.setdefault(slot, {'proposal':data['proposal'], 'prepares':{}, 'commits':{}})
    st['prepares'][from_id] = sig
    n = len(PEERS)+1
    if len(st['prepares'])+1 >= QUORUM(n):  # include self implicitly
        # send commit
        csig = await sign((slot+st['proposal']).encode())
        body = {'slot':slot, 'from':NODE_ID, 'csig':csig}
        await broadcast('/commit', body)
    return web.json_response({'status':'ok'})

async def handle_commit(req):
    data = await req.json()
    slot = data['slot']; from_id = data['from']
    st = pending.setdefault(slot, {'proposal':data.get('proposal',''), 'prepares':{}, 'commits':{}})
    st['commits'][from_id] = data.get('csig')
    n = len(PEERS)+1
    if len(st['commits'])+1 >= QUORUM(n):
        committed.add(slot)
    return web.json_response({'status':'ok'})

app = web.Application()
app.add_routes([web.post('/propose', handle_propose),
                web.post('/prepare', handle_prepare),
                web.post('/commit', handle_commit)])
web.run_app(app, host='0.0.0.0', port=8000)