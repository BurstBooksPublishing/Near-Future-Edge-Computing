package main

import (
        "context"
        "crypto/tls"
        "log"
        "net"
        "time"

        pb "example.com/telemetry/proto" // generated protobuf package

        "google.golang.org/grpc"
        "google.golang.org/grpc/credentials"
        "google.golang.org/grpc/keepalive"
)

// simpleServer implements TelemetryService defined in proto.
type simpleServer struct{ pb.UnimplementedTelemetryServiceServer }

// StreamTelemetry handles bidirectional streaming of telemetry and control.
func (s *simpleServer) StreamTelemetry(stream pb.TelemetryService_StreamTelemetryServer) error {
        // set a per-stream processing deadline; respect client cancellations
        ctx := stream.Context()
        for {
                msg, err := stream.Recv() // receive telemetry
                if err != nil { return err }
                // process with bounded time budget
                procCtx, cancel := context.WithTimeout(ctx, 30*time.Millisecond)
                _ = procCtx
                cancel()
                // send control update as needed
                if err := stream.Send(&pb.Control{Seq: msg.Seq}); err != nil { return err }
        }
}

func main() {
        certFile := "/etc/ssl/certs/server.crt"
        keyFile := "/etc/ssl/private/server.key"

        creds, err := credentials.NewServerTLSFromFile(certFile, keyFile) // mTLS endpoint handled by Envoy in Kubernetes
        if err != nil { log.Fatalf("failed to load TLS certs: %v", err) }

        // Keepalive tuned for edge: avoid closing idle flows too aggressively.
        kp := keepalive.ServerParameters{
                MaxConnectionIdle:     5 * time.Minute,
                MaxConnectionAge:      0,
                Timeout:               20 * time.Second,
        }

        grpcOpts := []grpc.ServerOption{
                grpc.Creds(creds),
                grpc.KeepaliveParams(kp),
                grpc.MaxConcurrentStreams(128), // bound streams on constrained nodes
        }

        grpcServer := grpc.NewServer(grpcOpts...)
        pb.RegisterTelemetryServiceServer(grpcServer, &simpleServer{})

        lis, err := net.Listen("tcp", ":50051")
        if err != nil { log.Fatalf("listen failed: %v", err) }
        log.Println("gRPC telemetry server listening on :50051")
        if err := grpcServer.Serve(lis); err != nil { log.Fatalf("serve failed: %v", err) }
}