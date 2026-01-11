package main

import (
  "context"
  "log"
  "net/http"
  "os"
  "os/signal"
  "syscall"
  "time"
)

// quick-start HTTP handler for edge FaaS; keep dependencies minimal.
func handler(w http.ResponseWriter, r *http.Request) {
  // lightweight decode and minimal work to keep cold-start penalty small.
  w.Header().Set("Content-Type", "application/json")
  w.Write([]byte(`{"status":"ok"}`))
}

func main() {
  mux := http.NewServeMux()
  mux.HandleFunc("/", handler)
  mux.HandleFunc("/health", func(w http.ResponseWriter, r *http.Request) {
    w.WriteHeader(http.StatusOK)
    w.Write([]byte("alive"))
  })

  srv := &http.Server{
    Addr:         ":8080",
    Handler:      mux,
    ReadTimeout:  5 * time.Second,
    WriteTimeout: 10 * time.Second,
    IdleTimeout:  60 * time.Second,
  }

  // graceful shutdown to allow fast re-use by platform.
  go func() {
    if err := srv.ListenAndServe(); err != nil && err != http.ErrServerClosed {
      log.Fatalf("server failed: %v", err)
    }
  }()
  stop := make(chan os.Signal, 1)
  signal.Notify(stop, syscall.SIGINT, syscall.SIGTERM)
  <-stop
  ctx, cancel := context.WithTimeout(context.Background(), 5*time.Second)
  defer cancel()
  srv.Shutdown(ctx)
}
--- 
apiVersion: serving.knative.dev/v1
kind: Service
metadata:
  name: fast-edge-fn
spec:
  template:
    metadata:
      annotations:
        autoscaling.knative.dev/maxScale: "10"        # cap for resource-constrained node
        autoscaling.knative.dev/minScale: "1"         # provisioned concurrency for critical path
        autoscaling.knative.dev/target: "50"          # target concurrency per instance
    spec:
      containers:
      - image: registry.example.com/fast-edge-fn:v1    # small distroless image
        ports:
        - containerPort: 8080
        resources:
          limits:
            cpu: "500m"
            memory: "256Mi"