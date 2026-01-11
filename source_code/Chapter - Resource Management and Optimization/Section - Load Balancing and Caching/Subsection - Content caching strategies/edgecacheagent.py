#!/usr/bin/env python3
"""
edge_cache_agent.py - asyncio-based caching agent.
Assumes Redis for metadata and HTTP backend for origin fetches.
"""

import asyncio
import aiohttp
import aioredis
import hashlib
from bisect import bisect_right

# Rendezvous hashing utility
class RendezvousHash:
    def __init__(self, nodes):
        self.nodes = list(nodes)
    def pick(self, key):
        best = None; best_score = -1
        for n in self.nodes:
            score = int(hashlib.sha256((str(n)+key).encode()).hexdigest(),16)
            if score > best_score:
                best_score, best = score, n
        return best

# Simple LRU with file-backed storage
class LRUCache:
    def __init__(self, capacity_bytes, storage_dir):
        self.capacity = capacity_bytes
        self.storage_dir = storage_dir
        self.meta = {}            # key -> (size, last_access)
        self.total = 0
    async def get(self, key):
        # update access time
        if key in self.meta:
            self.meta[key] = (self.meta[key][0], asyncio.get_event_loop().time())
            path = f"{self.storage_dir}/{hashlib.sha1(key.encode()).hexdigest()}"
            async with aiofiles.open(path,'rb') as f:
                return await f.read()
        return None
    async def put(self, key, data):
        size = len(data)
        while self.total + size > self.capacity:
            # evict oldest
            oldest = min(self.meta.items(), key=lambda kv: kv[1][1])[0]
            await self._evict(oldest)
        path = f"{self.storage_dir}/{hashlib.sha1(key.encode()).hexdigest()}"
        async with aiofiles.open(path,'wb') as f:
            await f.write(data)
        self.meta[key] = (size, asyncio.get_event_loop().time())
        self.total += size
    async def _evict(self, key):
        size, _ = self.meta.pop(key)
        self.total -= size
        # remove file safely
        import os
        try: os.remove(f"{self.storage_dir}/{hashlib.sha1(key.encode()).hexdigest()}")
        except OSError: pass

# Agent main loop (simplified)
async def run_agent(node_id, nodes, redis_url, storage_dir, capacity):
    redis = await aioredis.create_redis_pool(redis_url)
    rhash = RendezvousHash(nodes)
    cache = LRUCache(capacity, storage_dir)
    session = aiohttp.ClientSession()
    async def handle_request(key, origin_url):
        owner = rhash.pick(key)
        if owner != node_id:
            # forward or return redirect; here we fetch owner metadata
            owner_addr = await redis.get(f"node:{owner}:addr")
            return {'redirect': owner_addr.decode()}
        # local attempt
        blob = await cache.get(key)
        if blob: return {'data': blob}
        # fetch from origin
        async with session.get(origin_url) as resp:
            data = await resp.read()
        await cache.put(key, data)
        # announce hot key to Redis with short TTL
        await redis.set(f"hot:{key}", node_id, expire=60)
        return {'data': data}
    return handle_request

# Note: omitted aiofiles import and server wiring for brevity; integrate with asyncio web server.