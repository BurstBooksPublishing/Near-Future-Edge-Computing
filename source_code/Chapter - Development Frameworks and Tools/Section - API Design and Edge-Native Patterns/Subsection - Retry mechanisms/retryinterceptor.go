package retry

import (
        "context"
        "math/rand"
        "time"

        "google.golang.org/grpc"
        "google.golang.org/grpc/codes"
        "google.golang.org/grpc/status"
)

// UnaryClientInterceptor returns a retry-capable gRPC interceptor.
// idempotentFunc checks RPC method idempotency (user-supplied).
func UnaryClientInterceptor(maxAttempts int, base time.Duration, factor float64, cap time.Duration, jitter time.Duration, idempotentFunc func(string) bool) grpc.UnaryClientInterceptor {
        return func(ctx context.Context, method string, req, reply interface{}, cc *grpc.ClientConn, invoker grpc.UnaryInvoker, opts ...grpc.CallOption) error {
                if !idempotentFunc(method) {
                        // Avoid unsafe retries.
                        return invoker(ctx, method, req, reply, cc, opts...)
                }
                attempt := 0
                var lastErr error
                for attempt < maxAttempts {
                        // Respect context deadline/cancellation.
                        if ctx.Err() != nil {
                                return ctx.Err()
                        }
                        attempt++
                        err := invoker(ctx, method, req, reply, cc, opts...)
                        if err == nil {
                                return nil
                        }
                        lastErr = err
                        // Do not retry on non-transient gRPC codes.
                        code := status.Code(err)
                        if code != codes.Unavailable && code != codes.DeadlineExceeded && code != codes.ResourceExhausted {
                                return err
                        }
                        // Compute backoff with cap and jitter.
                        backoff := base * time.Duration(math.Pow(factor, float64(attempt-1)))
                        if backoff > cap {
                                backoff = cap
                        }
                        // Add symmetric jitter.
                        j := time.Duration((rand.Float64()*2-1) * float64(jitter))
                        wait := backoff + j
                        // If waiting would exceed deadline, stop retrying.
                        if dl, ok := ctx.Deadline(); ok {
                                if time.Now().Add(wait).After(dl) {
                                        break
                                }
                        }
                        // Sleep in interruptible way.
                        select {
                        case <-time.After(wait):
                                // continue retry loop
                        case <-ctx.Done():
                                return ctx.Err()
                        }
                }
                // Return last error to caller for logging and metrics.
                return lastErr
        }
}