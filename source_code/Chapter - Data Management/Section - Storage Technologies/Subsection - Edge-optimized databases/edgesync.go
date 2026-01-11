package main

import (
    "context"
    "database/sql"
    "encoding/json"
    "net/http"
    "time"
    _ "github.com/mattn/go-sqlite3" // SQLite driver
)

// config constants
const dbPath = "/var/lib/edge/data.db"
const cloudEndpoint = "https://regional-edge.example/api/v1/sync"

// sample represents a telemetry sample row.
type sample struct {
    ID   int64     `json:"id"`
    Ts   time.Time `json:"ts"`
    Val  float64   `json:"val"`
}

// readBatch reads a bounded batch of unsent samples.
func readBatch(db *sql.DB, limit int) ([]sample, error) {
    rows, err := db.Query("SELECT id, ts, val FROM samples WHERE sent=0 ORDER BY ts LIMIT ?", limit)
    if err != nil { return nil, err }
    defer rows.Close()
    var out []sample
    for rows.Next() {
        var s sample
        if err := rows.Scan(&s.ID, &s.Ts, &s.Val); err != nil { return nil, err }
        out = append(out, s)
    }
    return out, rows.Err()
}

// markSent marks rows as sent within a transaction.
func markSent(db *sql.DB, ids []int64) error {
    tx, err := db.Begin()
    if err != nil { return err }
    stmt, _ := tx.Prepare("UPDATE samples SET sent=1 WHERE id=?")
    defer stmt.Close()
    for _, id := range ids {
        if _, err := stmt.Exec(id); err != nil { tx.Rollback(); return err }
    }
    return tx.Commit()
}

func shipBatch(ctx context.Context, batch []sample) error {
    body, _ := json.Marshal(batch)
    req, _ := http.NewRequestWithContext(ctx, "POST", cloudEndpoint, bytes.NewReader(body))
    req.Header.Set("Content-Type", "application/json")
    // simple client with deadline
    client := &http.Client{Timeout: 10 * time.Second}
    resp, err := client.Do(req)
    if err != nil { return err }
    resp.Body.Close()
    if resp.StatusCode != http.StatusOK { return fmt.Errorf("bad status: %d", resp.StatusCode) }
    return nil
}

func main() {
    db, err := sql.Open("sqlite3", dbPath+"?_busy_timeout=5000")
    if err != nil { panic(err) }
    defer db.Close()
    // main loop: read, ship, mark; with backoff on failure.
    for {
        batch, err := readBatch(db, 512)
        if err != nil { time.Sleep(5*time.Second); continue }
        if len(batch) == 0 { time.Sleep(1*time.Second); continue }
        // exponential backoff
        backoff := 1 * time.Second
        for {
            ctx, cancel := context.WithTimeout(context.Background(), 15*time.Second)
            err = shipBatch(ctx, batch)
            cancel()
            if err == nil {
                // mark sent
                ids := make([]int64, len(batch))
                for i, s := range batch { ids[i] = s.ID }
                if err = markSent(db, ids); err != nil { /* log and continue */ }
                break
            }
            time.Sleep(backoff)
            if backoff < 60*time.Second { backoff *= 2 }
        }
    }
}