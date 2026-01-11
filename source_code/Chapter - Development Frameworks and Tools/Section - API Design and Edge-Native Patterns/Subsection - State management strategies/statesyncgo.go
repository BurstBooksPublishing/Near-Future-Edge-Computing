package main

import (
    "context"
    "encoding/json"
    "time"

    mqtt "github.com/eclipse/paho.mqtt.golang"
    bolt "go.etcd.io/bbolt"
)

type Sample struct { Ts int64; Sensor string; Val float64 } // short struct

func main() {
    // open local bolt DB (durable, small footprint)
    db, _ := bolt.Open("state.db", 0600, &bolt.Options{Timeout: 1 * time.Second})
    defer db.Close()

    // MQTT client with reconnect and persistent session
    opts := mqtt.NewClientOptions().AddBroker("tcp://broker:1883").SetClientID("edge-gw-01")
    opts.SetAutoReconnect(true)
    client := mqtt.NewClient(opts)
    if token := client.Connect(); token.Wait() && token.Error() != nil { panic(token.Error()) }
    defer client.Disconnect(250)

    ctx, cancel := context.WithCancel(context.Background())
    go publisher(ctx, db, client) // background delta publisher

    // main ingestion loop: append to WAL bucket
    for i := 0; i < 1000; i++ {
        s := Sample{Ts: time.Now().Unix(), Sensor: "vib", Val: 0.1*float64(i)}
        db.Update(func(tx *bolt.Tx) error {
            b, _ := tx.CreateBucketIfNotExists([]byte("wal"))
            id, _ := b.NextSequence()
            v, _ := json.Marshal(s)
            return b.Put(itob(id), v)
        })
        time.Sleep(10 * time.Millisecond)
    }
    cancel() // graceful shutdown
}

// publisher reads WAL, publishes batches, and checkpoints to 'ckpt' bucket.
func publisher(ctx context.Context, db *bolt.DB, client mqtt.Client) {
    ticker := time.NewTicker(500 * time.Millisecond)
    defer ticker.Stop()
    for {
        select {
        case <-ctx.Done(): return
        case <-ticker.C:
            var batch [][]byte
            var lastID uint64
            // collect up to N records
            db.View(func(tx *bolt.Tx) error {
                b := tx.Bucket([]byte("wal"))
                if b == nil { return nil }
                c := b.Cursor()
                for k, v := c.First(); k != nil && len(batch) < 200; k, v = c.Next() {
                    batch = append(batch, append([]byte(nil), v...)) // copy
                    lastID = btoi(k)
                }
                return nil
            })
            if len(batch) == 0 { continue }
            payload, _ := json.Marshal(batch)
            token := client.Publish("edge/deltas", 1, false, payload) // QoS 1
            token.Wait()
            if token.Error() == nil {
                // checkpoint deletion: truncate WAL up to lastID
                db.Update(func(tx *bolt.Tx) error {
                    b := tx.Bucket([]byte("wal")); if b == nil { return nil }
                    c := b.Cursor()
                    for k, _ := c.First(); k != nil && btoi(k) <= lastID; k, _ = c.Next() {
                        c.Delete()
                    }
                    return tx.Bucket([]byte("ckpt")).Put([]byte("last"), itob(lastID))
                })
            }
        }
    }
}

// helper conversions
func itob(v uint64) []byte { b := make([]byte, 8); for i:=7;i>=0;i--{ b[i]=byte(v); v>>=8 }; return b }
func btoi(b []byte) uint64 { var v uint64; for i:=0;i<8;i++{ v=(v<<8)|uint64(b[i]) }; return v }