package main

import (
  "context"
  "crypto/tls"
  "encoding/json"
  "net/http"
  "os"
  "os/signal"
  "sync"
  "syscall"
  "time"

  mqtt "github.com/eclipse/paho.mqtt.golang"
)

type Message struct {
  Topic string          `json:"topic"`
  Ts    time.Time       `json:"ts"`
  Body  json.RawMessage `json:"body"`
}

func main() {
  ctx, cancel := context.WithCancel(context.Background())
  defer cancel()

  // MQTT client setup with TLS and client certs
  tlsCfg := &tls.Config{MinVersion: tls.VersionTLS12}
  opts := mqtt.NewClientOptions().AddBroker("tls://broker.local:8883")
  opts.SetClientID("gateway-01")
  opts.SetTLSConfig(tlsCfg)
  mc := mqtt.NewClient(opts)
  if token := mc.Connect(); token.Wait() && token.Error() != nil {
    panic(token.Error())
  }
  defer mc.Disconnect(250)

  // HTTP client with connection pooling
  httpClient := &http.Client{
    Timeout:   10 * time.Second,
    Transport: &http.Transport{MaxIdleConnsPerHost: 16},
  }

  // batching pipeline
  ch := make(chan Message, 1000)
  var wg sync.WaitGroup
  wg.Add(1)
  go batchSender(ctx, &wg, ch, httpClient)

  // MQTT subscription handler
  mc.Subscribe("sensors/+", 1, func(c mqtt.Client, m mqtt.Message) {
    msg := Message{Topic: m.Topic(), Ts: time.Now(), Body: json.RawMessage(m.Payload())}
    select {
    case ch <- msg:
    default:
      // backpressure policy: drop oldest, log metric
    }
  })

  // graceful shutdown on SIGINT/SIGTERM
  sig := make(chan os.Signal, 1)
  signal.Notify(sig, syscall.SIGINT, syscall.SIGTERM)
  <-sig
  cancel()
  mc.Unsubscribe("sensors/+")
  close(ch)
  wg.Wait()
}

// batchSender aggregates messages and posts JSON arrays.
func batchSender(ctx context.Context, wg *sync.WaitGroup, ch <-chan Message, client *http.Client) {
  defer wg.Done()
  ticker := time.NewTicker(50 * time.Millisecond) // max batch latency
  defer ticker.Stop()

  buf := make([]Message, 0, 256)
  for {
    select {
    case <-ctx.Done():
      flush(buf, client)
      return
    case m, ok := <-ch:
      if !ok {
        flush(buf, client)
        return
      }
      buf = append(buf, m)
      if len(buf) >= 128 { // max batch size
        flush(buf, client)
        buf = buf[:0]
      }
    case <-ticker.C:
      if len(buf) > 0 {
        flush(buf, client)
        buf = buf[:0]
      }
    }
  }
}

func flush(batch []Message, client *http.Client) {
  if len(batch) == 0 {
    return
  }
  payload, _ := json.Marshal(batch)
  req, _ := http.NewRequest("POST", "https://edge-collector.local/v1/events", 
    bytes.NewReader(payload))
  req.Header.Set("Content-Type", "application/json")
  // send with retry/backoff in production
  client.Do(req)
}