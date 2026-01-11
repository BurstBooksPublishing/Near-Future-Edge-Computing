package main

import (
    "crypto/tls"
    "crypto/x509"
    "io/ioutil"
    "log"
    "net/http"
    "sync/atomic"
    "time"
)

type CertHolder struct {
    cert atomic.Value // stores *tls.Certificate
}

func NewCertHolder() *CertHolder { ch := &CertHolder{}; ch.cert.Store((*tls.Certificate)(nil)); return ch }

func (ch *CertHolder) Load(certFile, keyFile string) error {
    cert, err := tls.LoadX509KeyPair(certFile, keyFile)
    if err != nil { return err }
    ch.cert.Store(&cert)
    return nil
}

func (ch *CertHolder) Get() *tls.Certificate { return ch.cert.Load().(*tls.Certificate) }

func (ch *CertHolder) TLSConfig(caFile string) (*tls.Config, error) {
    caPEM, err := ioutil.ReadFile(caFile)
    if err != nil { return nil, err }
    pool := x509.NewCertPool(); pool.AppendCertsFromPEM(caPEM)
    return &tls.Config{
        RootCAs:            pool,
        GetClientCertificate: func(info *tls.CertificateRequestInfo) (*tls.Certificate, error) {
            cert := ch.Get()
            if cert == nil { return nil, nil }
            return cert, nil
        },
        MinVersion: tls.VersionTLS12,
    }, nil
}

func main() {
    ch := NewCertHolder()
    // initial load; in production, integrate with SPIRE or cert-agent for issuance
    if err := ch.Load("/etc/edge/certs/client.crt", "/etc/edge/certs/client.key"); err != nil {
        log.Fatalf("initial cert load: %v", err)
    }
    tlsCfg, err := ch.TLSConfig("/etc/edge/certs/ca.crt")
    if err != nil { log.Fatalf("tls config: %v", err) }

    transport := &http.Transport{TLSClientConfig: tlsCfg}
    client := &http.Client{Transport: transport, Timeout: 10 * time.Second}

    // background cert refresher polls a local cert agent and atomically replaces cert files
    go func() {
        ticker := time.NewTicker(30 * time.Second); defer ticker.Stop()
        for range ticker.C {
            if err := ch.Load("/etc/edge/certs/client.crt", "/etc/edge/certs/client.key"); err != nil {
                log.Printf("cert reload failed: %v", err)
            }
        }
    }()

    // application loop
    for {
        resp, err := client.Get("https://edge-control-plane.local/api/v1/status")
        if err != nil {
            log.Printf("request error: %v", err); time.Sleep(5 * time.Second); continue
        }
        resp.Body.Close()
        time.Sleep(10 * time.Second)
    }
}