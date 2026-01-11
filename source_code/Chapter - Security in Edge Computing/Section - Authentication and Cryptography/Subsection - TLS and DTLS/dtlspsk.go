package main

import (
        "context"
        "crypto/subtle"
        "log"
        "os"
        "os/signal"
        "time"

        "github.com/pion/dtls/v2"
)

// verifyPSK demonstrates a server-side PSK callback.
// Replace with TPM/HSM-backed lookup in production.
func verifyPSK(identityHint []byte) (psk []byte, err error) {
        // identityHint is a client-provided identity (e.g., "sensor-123")
        if string(identityHint) == "sensor-123" {
                return []byte("supersecretpskvalue"), nil
        }
        return nil, nil
}

func main() {
        // graceful shutdown context
        ctx, cancel := signal.NotifyContext(context.Background(), os.Interrupt)
        defer cancel()

        // DTLS server configuration with PSK and recommended cipher suites
        config := &dtls.Config{
                PSK: func(hint []byte) ([]byte, error) { return verifyPSK(hint) },
                PSKIdentityHint: []byte("edge-gateway"), // optional hint
                CipherSuites: []dtls.CipherSuiteID{
                        dtls.TLS_PSK_WITH_AES_128_GCM_SHA256,
                        dtls.TLS_PSK_WITH_AES_128_CCM_8,
                },
                // Enforce reasonable handshake timeouts for edge environments
                FlightInterval: 250 * time.Millisecond,
                ConnectContextMaker: func() (context.Context, func()) {
                        c, cancel := context.WithTimeout(context.Background(), 5*time.Second)
                        return c, cancel
                },
        }

        // Listen on UDP for DTLS
        conn, err := dtls.Listen("udp", &dtls.ListenConfig{Addr: ":5684"}, config)
        if err != nil {
                log.Fatalf("dtls listen: %v", err)
        }
        defer conn.Close()
        log.Println("DTLS server listening on :5684 (CoAPS)")

        // Accept loop with graceful shutdown
        for {
                select {
                case <-ctx.Done():
                        log.Println("shutting down DTLS server")
                        return
                default:
                }
                conn.SetDeadline(time.Now().Add(500 * time.Millisecond))
                c, err := conn.Accept()
                if err != nil {
                        // transient: continue accept loop
                        if opErr, ok := err.(interface{ Temporary() bool }); ok && opErr.Temporary() {
                                continue
                        }
                        // timeout or closed; re-evaluate context
                        continue
                }
                go handleClient(c)
        }
}

func handleClient(c dtls.Conn) {
        defer c.Close()
        buf := make([]byte, 1500)
        for {
                c.SetReadDeadline(time.Now().Add(30 * time.Second))
                n, err := c.Read(buf)
                if err != nil {
                        return // connection closed or timeout
                }
                // minimal replay-resistant PSK check example (timing-safe)
                if subtle.ConstantTimeCompare(buf[:3], []byte("MSG")) == 1 {
                        // process telemetry payload
                        payload := buf[3:n]
                        log.Printf("recv %d bytes from %s", len(payload), c.RemoteAddr())
                        // persist or forward to local edge pipeline (e.g., MQTT, Kafka)
                }
        }
}