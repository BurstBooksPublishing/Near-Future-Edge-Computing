#!/usr/bin/env python3
"""
Adaptive controller: monitors bandwidth and adjusts encoder tile bitrates.
Requires GStreamer and a hardware encoder (nvh265enc on Jetson example).
Run in container on edge; keep pipeline restart minimal for production.
"""
import asyncio
import subprocess
import aiohttp
import json
import time

GST_PIPE = (
    "appsrc ! videoconvert ! videoscale ! video/x-raw,width=1920,height=1080,"
    "framerate=90/1 ! nvh265enc max-bframes=0 periodicity-idr=30 bitrate={br} ! "
    "h265parse ! rtph265pay ! udpsink host={host} port={port}"
)

class TranscoderController:
    def __init__(self, host, port, telemetry_url):
        self.host = host; self.port = port; self.telemetry_url = telemetry_url
        self.proc = None; self.current_br = 8000  # kbps initial

    async def fetch_network(self):
        async with aiohttp.ClientSession() as s:
            try:
                async with s.get(self.telemetry_url, timeout=0.5) as r:
                    return await r.json()
            except Exception:
                return {"bw_kbps": 5000, "rtt_ms": 50}

    def start_pipeline(self, br_kbps):
        if self.proc:
            self.proc.terminate(); self.proc.wait()
        cmd = ["gst-launch-1.0", "-e"] + GST_PIPE.format(
            br=int(br_kbps), host=self.host, port=self.port).split()
        self.proc = subprocess.Popen(cmd)

    async def adapt_loop(self):
        self.start_pipeline(self.current_br)
        while True:
            net = await self.fetch_network()
            bw = net.get("bw_kbps", 5000); rtt = net.get("rtt_ms", 50)
            # simple bitrate rule: leave 20% margin, prefer lower br on high rtt
            target = max(2000, int(bw * 0.75 - rtt * 5))
            if abs(target - self.current_br) > 500:
                self.current_br = target
                self.start_pipeline(self.current_br)
            await asyncio.sleep(0.2)

if __name__ == "__main__":
    ctl = TranscoderController(host="192.0.2.10", port=5004, telemetry_url="http://127.0.0.1:8000/net")
    asyncio.run(ctl.adapt_loop())