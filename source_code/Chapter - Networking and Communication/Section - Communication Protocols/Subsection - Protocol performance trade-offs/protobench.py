#!/usr/bin/env python3
# Simple asyncio echo benchmark usable on Raspberry Pi / Jetson.
import asyncio, argparse, time, statistics, socket

async def udp_ping(host, port, size, count, timeout):
    loop = asyncio.get_running_loop()
    addr = (host, port)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setblocking(False)
    rtts = []
    for i in range(count):
        payload = b'x' * size
        t0 = time.perf_counter()
        await loop.sock_sendall(sock, payload)             # send
        try:
            data, _ = await asyncio.wait_for(loop.sock_recvfrom(sock, 65536), timeout)
            rtts.append((time.perf_counter()-t0)*1000)     # ms
        except asyncio.TimeoutError:
            rtts.append(None)
    sock.close()
    return rtts

async def tcp_ping(host, port, size, count, timeout):
    rtts = []
    reader, writer = await asyncio.open_connection(host, port)
    for i in range(count):
        payload = b'x' * size
        t0 = time.perf_counter()
        writer.write(len(payload).to_bytes(2,'big')+payload)  # simple length-prefix
        await writer.drain()
        try:
            header = await asyncio.wait_for(reader.readexactly(2), timeout)
            n = int.from_bytes(header,'big')
            await reader.readexactly(n)
            rtts.append((time.perf_counter()-t0)*1000)
        except asyncio.TimeoutError:
            rtts.append(None)
    writer.close()
    await writer.wait_closed()
    return rtts

# CLI omitted for brevity; invoke either udp_ping or tcp_ping, print p50/p90/p99 and loss.