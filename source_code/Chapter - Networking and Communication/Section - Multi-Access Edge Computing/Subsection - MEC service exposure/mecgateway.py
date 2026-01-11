from fastapi import FastAPI, Request, HTTPException, Depends
from pydantic import BaseModel
import httpx, os, jwt

NEF_URL = os.environ["NEF_URL"]      # https://nef.local:8443
INTROSPECT = os.environ["INTROSPECT_URL"]
AUD_REQUIRED = "mec.gateway"

app = FastAPI()

class RnisQuery(BaseModel):
    ue_id: str
    metrics: list[str]

async def introspect(token: str) -> dict:
    async with httpx.AsyncClient(verify="/etc/ssl/certs/ca.pem", timeout=2.0) as c:
        r = await c.post(INTROSPECT, data={"token": token})
    if r.status_code != 200: raise HTTPException(401, "Invalid token")
    return r.json()

async def require_token(req: Request):
    auth = req.headers.get("authorization")
    if not auth or not auth.lower().startswith("bearer "):
        raise HTTPException(401, "Missing bearer token")
    token = auth.split(" ",1)[1]
    meta = await introspect(token)
    if AUD_REQUIRED not in meta.get("aud", []):
        raise HTTPException(403, "Wrong audience")
    return token

@app.post("/mec/v1/rnis/query")
async def rnis_query(q: RnisQuery, token: str = Depends(require_token)):
    # Translate and forward to NEF with mTLS client certs (certs mounted into pod)
    body = {"ueId": q.ue_id, "metrics": q.metrics}
    async with httpx.AsyncClient(
        cert=("/etc/ssl/certs/client.crt","/etc/ssl/private/client.key"),
        verify="/etc/ssl/certs/ca.pem", timeout=1.0
    ) as c:
        r = await c.post(f"{NEF_URL}/rnis", json=body)
    if r.status_code != 200:
        raise HTTPException(502, "NEF proxy error")
    return r.json()
# Run with: uvicorn main:app --host 0.0.0.0 --port 8443 --ssl-keyfile ... --ssl-certfile ...