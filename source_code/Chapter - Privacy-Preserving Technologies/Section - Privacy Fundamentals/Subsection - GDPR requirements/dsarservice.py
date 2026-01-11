from fastapi import FastAPI, HTTPException, Depends
from pydantic import BaseModel
import sqlite3, os, json, base64
from cryptography.hazmat.primitives.ciphers.aead import AESGCM

# Key management should use a secure HSM or TPM; here we load from file for demo.
KEY_PATH = "/var/run/secrets/edge_aes_key.bin"  # provisioned securely
if not os.path.exists(KEY_PATH):
    raise SystemExit("Encryption key missing")
with open(KEY_PATH,'rb') as f:
    AES_KEY = f.read()  # 32 bytes for AES-256-GCM

DB_PATH = "/var/lib/edge_dsar/store.db"
os.makedirs(os.path.dirname(DB_PATH), exist_ok=True)
conn = sqlite3.connect(DB_PATH, check_same_thread=False)
conn.execute("CREATE TABLE IF NOT EXISTS records(id TEXT PRIMARY KEY, data BLOB, iv BLOB)")
app = FastAPI()

class ExportRequest(BaseModel):
    subject_id: str

def encrypt(plaintext: bytes) -> tuple:
    aesgcm = AESGCM(AES_KEY)
    iv = os.urandom(12)
    ct = aesgcm.encrypt(iv, plaintext, None)
    return (ct, iv)

def decrypt(ct: bytes, iv: bytes) -> bytes:
    aesgcm = AESGCM(AES_KEY)
    return aesgcm.decrypt(iv, ct, None)

@app.post("/dsar/store")  # internal API, authenticate in production via mTLS or token
def store_record(subject_id: str, payload: dict):
    pt = json.dumps(payload).encode()
    ct, iv = encrypt(pt)
    conn.execute("REPLACE INTO records(id,data,iv) VALUES(?,?,?)", (subject_id, ct, iv))
    conn.commit()
    return {"status":"stored"}

@app.post("/dsar/export", response_model=dict)
def export_request(req: ExportRequest):
    row = conn.execute("SELECT data, iv FROM records WHERE id=?", (req.subject_id,)).fetchone()
    if not row:
        raise HTTPException(status_code=404, detail="no data")
    ct, iv = row
    try:
        pt = decrypt(ct, iv)
    except Exception:
        raise HTTPException(status_code=500, detail="decryption failed")
    # Return portable JSON; caller must authenticate and log request under audit policy.
    return {"subject_id": req.subject_id, "payload": json.loads(pt)}

@app.post("/dsar/delete")
def delete_request(req: ExportRequest):
    conn.execute("DELETE FROM records WHERE id=?", (req.subject_id,))
    conn.commit()
    # Log deletion with tamper-evident audit trail (send signed event to central log).
    return {"status":"deleted"}