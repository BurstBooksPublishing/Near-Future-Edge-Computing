package main

import (
        "context"
        "time"

        "github.com/sony/gobreaker"           // lightweight circuit breaker
        "github.com/prometheus/client_golang/prometheus"
        "google.golang.org/grpc"
        pb "example.com/protos/model"         // generated proto package
)

var (
        cbState = prometheus.NewGauge(prometheus.GaugeOpts{
                Name: "edge_cb_state", Help: "Circuit breaker state: 0=closed,1=half-open,2=open",
        })
        cbFailures = prometheus.NewCounter(prometheus.CounterOpts{
                Name: "edge_cb_failures_total", Help: "Total downstream failures counted by CB",
        })
)

func init() {
        prometheus.MustRegister(cbState, cbFailures)
}

func main() {
        ctx := context.Background()
        conn, _ := grpc.DialContext(ctx, "cloud:model-service:50051", grpc.WithInsecure(), grpc.WithBlock())
        defer conn.Close()
        client := pb.NewModelServiceClient(conn)

        cb := gobreaker.NewCircuitBreaker(gobreaker.Settings{
                Name:        "ModelService",
                MaxRequests: 2,                      // half-open concurrent probes
                Interval:    30 * time.Second,       // EWMA reset interval (if desired)
                Timeout:     15 * time.Second,       // open -> half-open cooldown
                ReadyToTrip: func(counts gobreaker.Counts) bool {
                        // trip if error ratio > 50% and at least 20 requests
                        if counts.Requests < 20 {
                                return false
                        }
                        return float64(counts.TotalFailures)/float64(counts.Requests) > 0.5
                },
                OnStateChange: func(name string, from, to gobreaker.State) {
                        cbState.Set(float64(to))
                },
        })

        // callRemote wraps model infer calls with CB and per-call timeout
        callRemote := func(input *pb.InferenceRequest) (*pb.InferenceResponse, error) {
                result, err := cb.Execute(func() (interface{}, error) {
                        // per-request timeout to bound latency and energy
                        reqCtx, cancel := context.WithTimeout(ctx, 800*time.Millisecond)
                        defer cancel()
                        resp, err := client.Infer(reqCtx, input)
                        if err != nil {
                                cbFailures.Inc()
                        }
                        return resp, err
                })
                if err != nil {
                        return nil, err
                }
                return result.(*pb.InferenceResponse), nil
        }

        // production loop omitted: call callRemote and fallback to local model when errors occur
}