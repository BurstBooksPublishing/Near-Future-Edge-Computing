import sqlite3, threading, time, logging, os

DB_PATH = "/var/lib/mygateway/telemetry.db"
CHECKPOINT_INTERVAL = 30.0  # seconds
WAL_CHECKPOINT_MODE = "PASSIVE"  # use TRUNCATE for strong reclaim

# Open shared connection factory for threads; allow access from multiple threads.
def open_db(path=DB_PATH, timeout=5.0):
    con = sqlite3.connect(path, timeout=timeout, check_same_thread=False)
    cur = con.cursor()
    # WAL, moderate sync for latency; adjust to FULL for critical durability.
    cur.execute("PRAGMA journal_mode=WAL;")
    cur.execute("PRAGMA synchronous=NORMAL;")
    cur.execute("PRAGMA cache_size=-2000;")    # -N means N pages, negative = KB units
    # Enable reasonable mmap on Linux systems with stable filesystems.
    cur.execute("PRAGMA mmap_size=268435456;") # 256MB
    con.commit()
    return con

# Background checkpoint to limit WAL size and control flash behavior.
class WALCheckpointer(threading.Thread):
    def __init__(self, db_path=DB_PATH, interval=CHECKPOINT_INTERVAL):
        super().__init__(daemon=True)
        self.db_path = db_path; self.interval = interval; self.running = True

    def run(self):
        while self.running:
            try:
                c = sqlite3.connect(self.db_path, timeout=2.0)
                # checkpoint may be I/O heavy; PASSIVE avoids blocking writers.
                c.execute(f"PRAGMA wal_checkpoint({WAL_CHECKPOINT_MODE});")
                c.close()
            except Exception as e:
                logging.warning("WAL checkpoint failed: %s", e)
            time.sleep(self.interval)

    def stop(self):
        self.running = False

# Usage: create DB, start checkpointer, use connection pool for app threads.
if __name__ == "__main__":
    os.makedirs(os.path.dirname(DB_PATH), exist_ok=True)
    con = open_db()
    con.execute("CREATE TABLE IF NOT EXISTS telemetry(ts INTEGER PRIMARY KEY, payload BLOB);")
    con.commit()
    cp = WALCheckpointer(); cp.start()
    # application writes and reads here...
    # cp.stop() on shutdown; con.close()