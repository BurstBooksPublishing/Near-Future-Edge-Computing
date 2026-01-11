#!/usr/bin/env python3
# Lightweight HIL orchestrator: run test vectors, inject CAN faults, push metrics.
import time
import json
import can                            # python-can for CAN bus access
import requests                       # InfluxDB v2 write API
from subprocess import Popen, PIPE

INFLUX_URL = "http://influx.local:8086/api/v2/write?org=org&bucket=hil"
TOKEN = "REPLACE_WITH_TOKEN"

def send_influx(measurement, fields, tags=None, ts=None):
    tags = tags or {}
    tag_str = ",".join(f"{k}={v}" for k,v in tags.items())
    field_str = ",".join(f"{k}={v}" for k,v in fields.items())
    line = f"{measurement},{tag_str} {field_str}"
    if ts: line += f" {ts}"
    headers = {"Authorization": f"Token {TOKEN}"}
    requests.post(INFLUX_URL, data=line, headers=headers, timeout=5)

def run_scenario(scenario_cmd):
    # Run scenario process (e.g., Simulink HIL client), measure start/stop.
    p = Popen(scenario_cmd, shell=True, stdout=PIPE, stderr=PIPE)
    out, err = p.communicate()
    return p.returncode, out.decode(), err.decode()

def inject_can_error(bus, error_frame_id=0x700):
    # Send malformed frame to test CAN error handling.
    msg = can.Message(arbitration_id=error_frame_id, data=[0xFF]*8, is_extended_id=False)
    bus.send(msg)

def main():
    bus = can.interface.Bus(channel="can0", bustype="socketcan")
    scenarios = ["./run_lane_keep.sh", "./run_pedestrian_brake.sh"]
    for s in scenarios:
        t0 = time.time()
        rc, out, err = run_scenario(s)
        dur = time.time() - t0
        # Inject transient CAN fault mid-scenario to validate recovery.
        inject_can_error(bus)
        send_influx("hil_runs", {"duration":dur, "ret":rc}, tags={"scenario":s})
        # log outputs for traceability
        with open(f"logs/{s.replace('./','')}_{int(t0)}.json","w") as f:
            json.dump({"rc":rc,"stdout":out,"stderr":err}, f)

if __name__ == "__main__":
    main()