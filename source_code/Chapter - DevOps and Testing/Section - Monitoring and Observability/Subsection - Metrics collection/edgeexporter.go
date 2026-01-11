package main

import (
        "context"
        "log"
        "net/http"
        "os"
        "os/signal"
        "time"

        "github.com/prometheus/client_golang/prometheus"
        "github.com/prometheus/client_golang/prometheus/promhttp"
        "github.com/shirou/gopsutil/cpu"
        "github.com/shirou/gopsutil/mem"
)

var (
        cpuGauge = prometheus.NewGauge(prometheus.GaugeOpts{
                Name: "edge_cpu_usage_percent", Help: "CPU usage percent (0-100).",
        })
        memGauge = prometheus.NewGauge(prometheus.GaugeOpts{
                Name: "edge_memory_used_bytes", Help: "Memory used in bytes.",
        })
)

func init() {
        prometheus.MustRegister(cpuGauge, memGauge)
}

func collectOnce(ctx context.Context) {
        // CPU percent averaged across cores
        percent, err := cpu.PercentWithContext(ctx, time.Second, false)
        if err == nil && len(percent) > 0 {
                cpuGauge.Set(percent[0])
        }
        // Memory stats
        vm, err := mem.VirtualMemoryWithContext(ctx)
        if err == nil {
                memGauge.Set(float64(vm.Used))
        }
}

func main() {
        interval := 10 * time.Second // tune per Eq. (1) and device profile
        if v := os.Getenv("SCRAPE_INTERVAL"); v != "" {
                if d, err := time.ParseDuration(v); err == nil {
                        interval = d
                }
        }
        ctx, stop := signal.NotifyContext(context.Background(), os.Interrupt)
        defer stop()

        // Start HTTP server for Prometheus to scrape
        http.Handle("/metrics", promhttp.Handler())
        srv := &http.Server{Addr: ":9100"}
        go func() {
                log.Println("metrics HTTP server starting on :9100")
                if err := srv.ListenAndServe(); err != nil && err != http.ErrServerClosed {
                        log.Fatalf("listen: %v", err)
                }
        }()

        ticker := time.NewTicker(interval)
        defer ticker.Stop()
        for {
                select {
                case <-ctx.Done():
                        _ = srv.Shutdown(context.Background())
                        return
                case <-ticker.C:
                        collectOnce(ctx)
                }
        }
}