#!/usr/bin/env python3
# Lightweight async probe: sends CAM-like UDP packets to MEC fusion service.
import asyncio, struct, time, argparse, socket

MSG_FMT = '!Qfff'   # send timestamp (ns), lat, lon, heading
SAMPLING_INTERVAL = 0.05  # 50 ms

async def sender(dst_ip, dst_port, iface=None):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    if iface: sock.setsockopt(socket.SOL_SOCKET, 25, iface.encode())  # bind to interface
    loop = asyncio.get_running_loop()
    seq = 0
    while True:
        t_ns = time.time_ns()
        # example position payload; real system uses CAM/BSM schema
        payload = struct.pack(MSG_FMT, t_ns, 52.0+0.001*seq, 13.4, 1.57)
        await loop.run_in_executor(None, sock.sendto, payload, (dst_ip, dst_port))
        seq += 1
        await asyncio.sleep(SAMPLING_INTERVAL)

async def receiver(bind_ip, bind_port):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((bind_ip, bind_port))
    sock.setblocking(False)
    loop = asyncio.get_running_loop()
    while True:
        data, addr = await loop.run_in_executor(None, sock.recvfrom, 2048)
        recv_ns = time.time_ns()
        t_ns, lat, lon, heading = struct.unpack(MSG_FMT, data)
        rtt_ms = (recv_ns - t_ns) / 1e6
        print(f"{addr} RTT={rtt_ms:.2f} ms lat={lat:.4f}")

if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--dst-ip', required=True)
    p.add_argument('--dst-port', type=int, default=55000)
    p.add_argument('--bind-ip', default='0.0.0.0')
    p.add_argument('--bind-port', type=int, default=55001)
    p.add_argument('--iface', default=None)
    args = p.parse_args()
    # Run both sender and receiver tasks to measure round-trip via MEC
    asyncio.run(asyncio.gather(
        sender(args.dst_ip, args.dst_port, iface=args.iface),
        receiver(args.bind_ip, args.bind_port)
    ))