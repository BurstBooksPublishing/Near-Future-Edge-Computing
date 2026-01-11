#!/usr/bin/env python3
# Production-ready async MLLP sender with mutual TLS for HL7 v2 messages.
import asyncio, ssl, datetime, socket

HOST = "his.example.local"
PORT = 2575
CLIENT_CERT = "/etc/edge/certs/gw.cert.pem"
CLIENT_KEY = "/etc/edge/certs/gw.key.pem"
CA_CERT = "/etc/edge/certs/ca.pem"

MLLP_START = b'\x0b'
MLLP_END = b'\x1c\r'

def hl7_ts(dt: datetime.datetime) -> str:
    return dt.strftime("%Y%m%d%H%M%S")

def build_oru(spo2: float, device_id: str, patient_id: str) -> bytes:
    now = datetime.datetime.utcnow()
    msh = f"MSH|^~\\&|EdgeGW|GatewaySite|HIS|MainHospital|{hl7_ts(now)}||ORU^R01|{int(now.timestamp())}|P|2.5.1"
    pid = f"PID|1||{patient_id}^^^MRN||"
    obx = f"OBX|1|NM|59408-5^Oxygen saturation in Arterial blood^LN|1|{spo2:.1f}|%|||F|||{hl7_ts(now)}"
    # Add custom Z-segment for gateway provenance
    zgw = f"ZGW|{device_id}|edge-fw-1.2.3"
    msg = "\r".join([msh, pid, obx, zgw]) + "\r"
    return msg.encode("utf-8")

async def send_hl7(message: bytes):
    ssl_ctx = ssl.create_default_context(ssl.Purpose.SERVER_AUTH, cafile=CA_CERT)
    ssl_ctx.load_cert_chain(certfile=CLIENT_CERT, keyfile=CLIENT_KEY)
    reader, writer = await asyncio.open_connection(HOST, PORT, ssl=ssl_ctx, server_hostname=HOST)
    # MLLP framing
    writer.write(MLLP_START + message + MLLP_END)
    await writer.drain()
    # Optional: read acknowledgement (ACK)
    try:
        ack = await asyncio.wait_for(reader.readuntil(MLLP_END), timeout=5.0)
    except asyncio.TimeoutError:
        ack = b""
    writer.close()
    await writer.wait_closed()
    return ack

# Example usage (run in gateway app)
if __name__ == "__main__":
    import sys
    spo2 = float(sys.argv[1]) if len(sys.argv) > 1 else 97.2
    device = "edge-gw-01"
    patient = "123456"
    msg = build_oru(spo2, device, patient)
    asyncio.run(send_hl7(msg))