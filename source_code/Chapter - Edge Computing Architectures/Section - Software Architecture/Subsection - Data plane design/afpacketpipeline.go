package main

import (
        "compress/gzip"
        "context"
        "crypto/aes"
        "crypto/cipher"
        "log"
        "os"
        "os/signal"
        "sync"
        "syscall"
        "time"

        "github.com/google/gopacket"
        "github.com/google/gopacket/afpacket"
)

// Fast, bounded batch type for packet payloads.
type Batch struct {
        Payloads [][]byte
}

func captureLoop(ctx context.Context, handle *afpacket.TPacket, out chan<- Batch, batchSize int, batchMaxDelay time.Duration) {
        defer close(out)
        var buf [][]byte
        timer := time.NewTimer(batchMaxDelay)
        timer.Stop()
        for {
                select {
                case <-ctx.Done():
                        return
                default:
                }
                data, _, err := handle.ZeroCopyReadPacketData()
                if err != nil {
                        if err == afpacket.ErrTimeout {
                                // Timeout can be used to flush partial batches.
                                select {
                                case <-timer.C:
                                default:
                                }
                                continue
                        }
                        log.Printf("capture read error: %v", err)
                        continue
                }
                // Minimal proto filter: only UDP payloads to our sensor port.
                packet := gopacket.NewPacket(data, gopacket.LayerTypeEthernet, gopacket.NoCopy)
                if udp := packet.Layer(gopacket.LayerTypeUDP); udp != nil {
                        buf = append(buf, data)
                        if len(buf) == 1 {
                                timer.Reset(batchMaxDelay) // start flush timer
                        }
                        if len(buf) >= batchSize {
                                out <- Batch{Payloads: buf}
                                buf = nil
                                timer.Stop()
                        }
                }
                select {
                case <-timer.C:
                        if len(buf) > 0 {
                                out <- Batch{Payloads: buf}
                                buf = nil
                        }
                default:
                }
        }
}

func worker(ctx context.Context, id int, in <-chan Batch, wg *sync.WaitGroup, key []byte) {
        defer wg.Done()
        block, _ := aes.NewCipher(key)                 // AES-128/192/256 based on key length.
        iv := make([]byte, aes.BlockSize)              // in practice, use unique IV per message.
        stream := cipher.NewCTR(block, iv)             // stream cipher for simplicity.
        for {
                select {
                case <-ctx.Done():
                        return
                case b, ok := <-in:
                        if !ok {
                                return
                        }
                        // Compress then encrypt each batch (streaming gzip).
                        var buf []byte
                        gw := gzip.NewWriter(nil)
                        gw.Reset(nil) // Use bytes.Buffer in real code; omitted for brevity.
                        for _, p := range b.Payloads {
                                // Application-level parsing/aggregation would go here.
                                _, _ = gw.Write(p)
                        }
                        _ = gw.Close()
                        // Encrypt buffer in-place (illustrative; use AEAD in production).
                        stream.XORKeyStream(buf, buf)
                        // Push to sink: e.g., local NATS, MQTT, or gRPC stream to microservice.
                        // pushToSink(buf)
                }
        }
}

func main() {
        // Context and signal handling for clean shutdown.
        ctx, cancel := context.WithCancel(context.Background())
        defer cancel()
        sig := make(chan os.Signal, 1)
        signal.Notify(sig, syscall.SIGINT, syscall.SIGTERM)
        go func() { <-sig; cancel() }()

        // AF_PACKET ring configuration tuned for low-latency.
        handle, err := afpacket.NewTPacket(
                afpacket.OptInterface("eth0"),
                afpacket.OptFrameSize(65536),
                afpacket.OptBlockSize(1<<20), // 1MiB blocks
                afpacket.OptNumBlocks(64),    // large ring for burst tolerance
        )
        if err != nil {
                log.Fatalf("afpacket NewTPacket: %v", err)
        }
        defer handle.Close()

        out := make(chan Batch, 128)
        go captureLoop(ctx, handle, out, 32, 5*time.Millisecond)

        var wg sync.WaitGroup
        key := []byte("example 16 bytekey") // replace with secure key management.
        for i := 0; i < 4; i++ {
                wg.Add(1)
                go worker(ctx, i, out, &wg, key)
        }
        wg.Wait()
}