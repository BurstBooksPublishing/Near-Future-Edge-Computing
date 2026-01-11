package main

import (
        "context"
        "crypto/tls"
        "log"
        "time"

        mqtt "github.com/eclipse/paho.mqtt.golang"
        "google.golang.org/grpc"
        "google.golang.org/grpc/credentials"
)

func main() {
        ctx, cancel := context.WithCancel(context.Background())
        defer cancel()

        // MQTT TLS client options (production: validate server certs)
        mqttOpts := mqtt.NewClientOptions().
                AddBroker("tls://broker.example:8883").
                SetClientID("edge-reconciler-01").
                SetTLSConfig(&tls.Config{MinVersion: tls.VersionTLS12})

        mClient := mqtt.NewClient(mqttOpts)
        if token := mClient.Connect(); token.Wait() && token.Error() != nil {
                log.Fatalf("mqtt connect: %v", token.Error())
        }
        defer mClient.Disconnect(250)

        // gRPC secure channel to cloud control-plane API
        creds := credentials.NewClientTLSFromCert(nil, "")
        conn, err := grpc.Dial("control.example:443", grpc.WithTransportCredentials(creds))
        if err != nil {
                log.Fatalf("grpc dial: %v", err)
        }
        defer conn.Close()
        // client := pb.NewControlAPIClient(conn) // generated protobuf client

        // reconciliation loop with backoff
        ticker := time.NewTicker(2 * time.Second)
        defer ticker.Stop()
        backoff := 500 * time.Millisecond

        for {
                select {
                case <-ctx.Done():
                        return
                case <-ticker.C:
                        // fetch desired state (pseudo-call)
                        // desired, err := client.GetDesired(ctx, &pb.Device{Id:"edge-01"})
                        // if err != nil { handle with backoff }
                        // For brevity use a static command payload
                        cmd := []byte(`{"action":"apply","manifest":"v1.2.3"}`)

                        token := mClient.Publish("control/edge-01/cmd", 1, false, cmd)
                        if token.WaitTimeout(5 * time.Second) && token.Error() != nil {
                                log.Printf("publish failed, backing off: %v", token.Error())
                                time.Sleep(backoff)
                                backoff = backoff * 2
                                if backoff > 10*time.Second { backoff = 10 * time.Second }
                                continue
                        }
                        backoff = 500 * time.Millisecond // reset on success
                }
        }
}