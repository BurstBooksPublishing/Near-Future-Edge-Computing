package main

import (
        "compress/gzip"
        "context"
        "crypto/tls"
        "crypto/x509"
        "encoding/json"
        "fmt"
        "io"
        "net"
        "net/http"
        "os"
        "os/signal"
        "time"

        "github.com/prometheus/client_golang/prometheus/promhttp" // metrics
)

// payload is a compact telemetry structure.
type payload struct {
        DeviceID string  `json:"device_id"`
        Value    float32 `json:"value"`
        TS       int64   `json:"ts"`
}

// writeGzip writes gzipped JSON when client accepts it.
func writeGzip(w http.ResponseWriter, r *http.Request, v interface{}) error {
        w.Header().Set("Content-Type", "application/json")
        if r.Header.Get("Accept-Encoding") == "gzip" {
                w.Header().Set("Content-Encoding", "gzip")
                gw := gzip.NewWriter(w)
                defer gw.Close()
                enc := json.NewEncoder(gw)
                return enc.Encode(v)
        }
        return json.NewEncoder(w).Encode(v)
}

// etagHandler returns resource with ETag conditional semantics.
func etagHandler(w http.ResponseWriter, r *http.Request) {
        ctx := r.Context()
        // short deadline to bound processing on edge
        deadlineCtx, cancel := context.WithTimeout(ctx, 200*time.Millisecond)
        defer cancel()

        select {
        case <-deadlineCtx.Done():
                http.Error(w, "request timeout", http.StatusGatewayTimeout)
                return
        default:
        }

        // Example artifact ID and computed ETag (in production use hash of content)
        artifactID := "model-v2"
        etag := fmt.Sprintf("\"%x\"", len(artifactID)+42) // placeholder deterministic ETag

        // Conditional GET
        if match := r.Header.Get("If-None-Match"); match == etag {
                w.WriteHeader(http.StatusNotModified)
                return
        }
        w.Header().Set("ETag", etag)

        // Provide small telemetry sample
        p := payload{DeviceID: "gw-01", Value: 12.3, TS: time.Now().Unix()}
        if err := writeGzip(w, r, p); err != nil {
                http.Error(w, "encode error", http.StatusInternalServerError)
        }
}

func main() {
        mux := http.NewServeMux()
        mux.Handle("/metrics", promhttp.Handler()) // prometheus metrics endpoint
        mux.HandleFunc("/telemetry", etagHandler)

        // TLS config tuned for edge: session resumption and minimal cipher suites.
        certPEM, err := os.ReadFile("/etc/edge/cert.pem")
        keyPEM, err := os.ReadFile("/etc/edge/key.pem")
        if err != nil {
                // fallback to plaintext in well-controlled networks (log in production)
                server := &http.Server{
                        Addr:         ":8080",
                        Handler:      mux,
                        ReadTimeout:  2 * time.Second,
                        WriteTimeout: 5 * time.Second,
                        IdleTimeout:  60 * time.Second, // keep-alive beneficial on 5G/fog links
                }
                go server.ListenAndServe()
        } else {
                cert, _ := tls.X509KeyPair(certPEM, keyPEM)
                rootCAs := x509.NewCertPool()
                // add CA if needed: rootCAs.AppendCertsFromPEM(caPEM)
                tlsCfg := &tls.Config{
                        Certificates:             []tls.Certificate{cert},
                        MinVersion:               tls.VersionTLS13,
                        PreferServerCipherSuites: true,
                        ClientAuth:               tls.NoClientCert,
                        SessionTicketsDisabled:   false, // enable session resumption
                }
                ln, _ := net.Listen("tcp", ":8443")
                tlsLn := tls.NewListener(ln, tlsCfg)
                server := &http.Server{
                        Handler:      mux,
                        ReadTimeout:  2 * time.Second,
                        WriteTimeout: 5 * time.Second,
                        IdleTimeout:  60 * time.Second,
                }
                go server.Serve(tlsLn)
        }

        // graceful shutdown
        quit := make(chan os.Signal, 1)
        signal.Notify(quit, os.Interrupt)
        <-quit
}