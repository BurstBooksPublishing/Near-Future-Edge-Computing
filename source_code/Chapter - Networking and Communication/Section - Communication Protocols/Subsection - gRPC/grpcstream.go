package main

import (
        "context"
        "crypto/tls"
        "log"
        "net"

        "google.golang.org/grpc"
        "google.golang.org/grpc/credentials"
        "google.golang.org/grpc/keepalive"

        telemetrypb "example.com/project/telemetrypb" // generated proto package
)

// server implements TelemetryService with bidirectional stream.
type server struct {
        telemetrypb.UnimplementedTelemetryServiceServer
}

func (s *server) TelemetryStream(stream telemetrypb.TelemetryService_TelemetryStreamServer) error {
        ctx := stream.Context()
        for {
                // receive telemetry message from device
                msg, err := stream.Recv()
                if err != nil {
                        return err // EOF or transport error handled by caller
                }
                // lightweight edge preprocessing before forwarding
                process(msg)
                // optional server-initiated command
                cmd := controlDecision(msg)
                if cmd != nil {
                        if err := stream.Send(cmd); err != nil {
                                return err
                        }
                }
                // respect context cancellation and deadlines
                select {
                case <-ctx.Done():
                        return ctx.Err()
                default:
                }
        }
}

func main() {
        // load TLS certs (production requires cert rotation)
        cert, err := tls.LoadX509KeyPair("/etc/ssl/certs/server.crt", "/etc/ssl/private/server.key")
        if err != nil {
                log.Fatalf("tls load: %v", err)
        }
        creds := credentials.NewServerTLSFromCert(&cert)

        kaParams := keepalive.ServerParameters{
                MaxConnectionIdle: 30000000000, // 30s
                Timeout:           10000000000, // 10s for active ping ack
        }

        grpcServer := grpc.NewServer(
                grpc.Creds(creds),
                grpc.KeepaliveParams(kaParams),
        )
        telemetrypb.RegisterTelemetryServiceServer(grpcServer, &server{})

        lis, err := net.Listen("tcp", ":50051")
        if err != nil {
                log.Fatalf("listen: %v", err)
        }
        log.Println("gRPC telemetry server listening :50051")
        if err := grpcServer.Serve(lis); err != nil {
                log.Fatalf("serve: %v", err)
        }
}

// process and controlDecision are light-weight edge routines.
func process(m *telemetrypb.TelemetryMsg) { /* transform, sample, or filter */ }
func controlDecision(m *telemetrypb.TelemetryMsg) *telemetrypb.ControlCmd { return nil }