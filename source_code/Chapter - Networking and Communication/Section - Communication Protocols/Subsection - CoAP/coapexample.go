package main

import (
  "context"
  "fmt"
  "log"
  "time"

  coap "github.com/plgd-dev/go-coap/v2/udp/client"
  "github.com/plgd-dev/go-coap/v2/message"
  "github.com/plgd-dev/go-coap/v2/net/blockwise"
)

func main() {
  ctx, cancel := context.WithTimeout(context.Background(), 30*time.Second)
  defer cancel()

  // Dial a CoAP server on UDP (replace with gateway IPv6 or host:port)
  conn, err := coap.Dial("udp", "192.0.2.10:5683")
  if err != nil {
    log.Fatalf("dial error: %v", err)
  }
  defer conn.Close()

  // Enable blockwise transfer with sensible defaults for constrained links.
  conn.EnableBlockwise(blockwise.SZX1024, blockwise.DefaultSZX, true)

  // Simple GET
  resp, err := conn.Get(ctx, "/sensors/temperature")
  if err != nil {
    log.Fatalf("GET error: %v", err)
  }
  fmt.Printf("GET payload: %s\n", resp.Message().Body())

  // Observe: register and process notifications until context done.
  notify, err := conn.Observe(ctx, "/sensors/temperature", func(req *message.Message) {
    buf, _ := req.Body()
    fmt.Printf("Observe notification: %s\n", string(buf))
  })
  if err != nil {
    log.Fatalf("Observe error: %v", err)
  }
  // Wait for notifications or until timeout/cancel.
  <-ctx.Done()
  // Cancel Observe and close.
  notify.Cancel(context.Background())
}