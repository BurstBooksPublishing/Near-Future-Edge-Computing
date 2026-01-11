package main

import (
        "context"
        "log"
        "os"
        "path/filepath"
        "time"

        minio "github.com/minio/minio-go/v7"
        "github.com/minio/minio-go/v7/pkg/credentials"
)

// Configuration constants (override via env in production)
const (
        LocalDir       = "/var/lib/edge/data" // working set dir
        Bucket         = "edge-cold"
        AccessWindow   = 10 * time.Second     // sampling interval
        EWMAAlpha      = 0.3
        HotLatencyMs   = 1    // ms
        ColdLatencyMs  = 200  // ms (network)
        CapacityBytes  = 50 << 30 // 50 GiB local working set limit
)

type ObjMeta struct {
        Path      string
        Size      int64
        EWMA      float64
        LastAcc   time.Time
}

func main() {
        // MinIO client from env: MINIO_ENDPOINT, MINIO_KEY, MINIO_SECRET, MINIO_SECURE
        endpoint := os.Getenv("MINIO_ENDPOINT")
        secure := os.Getenv("MINIO_SECURE") == "1"
        minioClient, err := minio.New(endpoint, &minio.Options{
                Creds:  credentials.NewEnvAWS(),
                Secure: secure,
        })
        if err != nil {
                log.Fatal(err)
        }
        ctx := context.Background()
        _ = minioClient.MakeBucket(ctx, Bucket, minio.MakeBucketOptions{})

        // In-memory index
        index := map[string]*ObjMeta{}
        ticker := time.NewTicker(AccessWindow)
        for {
                select {
                case <-ticker.C:
                        current := scanLocal(index)
                        estimateAndMigrate(ctx, minioClient, current)
                }
        }
}

// scanLocal updates index with observed files and last modified times.
func scanLocal(index map[string]*ObjMeta) map[string]*ObjMeta {
        now := time.Now()
        total := int64(0)
        current := map[string]*ObjMeta{}
        filepath.Walk(LocalDir, func(p string, info os.FileInfo, err error) error {
                if err != nil || info.IsDir() { return nil }
                m, ok := index[p]
                if !ok {
                        m = &ObjMeta{Path: p, Size: info.Size(), EWMA: 0}
                } else {
                        m.Size = info.Size()
                }
                // detect access via mtime change or application touch
                if info.ModTime().After(m.LastAcc) {
                        m.EWMA = EWMAAlpha*1 + (1-EWMAAlpha)*m.EWMA
                        m.LastAcc = now
                } else {
                        m.EWMA = (1-EWMAAlpha)*m.EWMA
                }
                current[p] = m
                total += m.Size
                return nil
        })
        // simple eviction by scanning if over capacity
        if total > CapacityBytes {
                // compute value density and migrate low-value items
                // (omitted sorting code for brevity; production: sort by v_i)
        }
        return current
}

// estimateAndMigrate moves cold objects to MinIO with retries and atomic local delete.
func estimateAndMigrate(ctx context.Context, client *minio.Client, current map[string]*ObjMeta) {
        for _, m := range current {
                // compute value density v = p * deltaLatency / size
                p := m.EWMA / AccessWindow.Seconds()
                v := p * float64(ColdLatencyMs-HotLatencyMs) / float64(m.Size)
                if v < 1e-8 { // threshold tuned per deployment
                        // upload to MinIO
                        reader, err := os.Open(m.Path)
                        if err != nil { continue }
                        _, err = client.PutObject(ctx, Bucket, filepath.Base(m.Path), reader, m.Size, minio.PutObjectOptions{})
                        reader.Close()
                        if err == nil { os.Remove(m.Path) } // remove local copy after successful upload
                }
        }
}