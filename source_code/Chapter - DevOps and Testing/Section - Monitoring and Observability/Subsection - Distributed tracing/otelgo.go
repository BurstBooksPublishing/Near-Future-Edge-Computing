package main

import (
        "context"
        "log"
        "time"

        "go.opentelemetry.io/contrib/instrumentation/google.golang.org/grpc/otelgrpc" // interceptor
        "go.opentelemetry.io/otel"
        "go.opentelemetry.io/otel/attribute"
        sdktrace "go.opentelemetry.io/otel/sdk/trace"
        "go.opentelemetry.io/otel/exporters/jaeger"

        mqtt "github.com/eclipse/paho.mqtt.golang"
        "google.golang.org/grpc"
)

func initTracer(jaegerEndpoint string) (*sdktrace.TracerProvider, error) {
        exp, err := jaeger.New(jaeger.WithCollectorEndpoint(jaeger.WithEndpoint(jaegerEndpoint)))
        if err != nil { return nil, err }
        tp := sdktrace.NewTracerProvider(sdktrace.WithBatcher(exp))
        otel.SetTracerProvider(tp)
        return tp, nil
}

func main() {
        tp, err := initTracer("https://jaeger-collector.local:14268/api/traces") // mTLS recommended
        if err != nil { log.Fatal(err) }
        defer func(){ _ = tp.Shutdown(context.Background()) }()

        // gRPC server/client set up with OpenTelemetry interceptors
        grpcServer := grpc.NewServer(grpc.UnaryInterceptor(otelgrpc.UnaryServerInterceptor()))
        _ = grpcServer // register services...

        // MQTT client (edge broker) with trace context propagated via user properties (MQTT v5)
        opts := mqtt.NewClientOptions().AddBroker("tls://edge-broker.local:8883")
        opts.SetClientID("edge-service-1")
        mc := mqtt.NewClient(opts)
        if token := mc.Connect(); token.Wait() && token.Error() != nil { log.Fatal(token.Error()) }

        // Create a span and publish with trace context
        tr := otel.Tracer("edge/service")
        ctx, span := tr.Start(context.Background(), "process-and-publish")
        span.SetAttributes(attribute.String("device.id", "gw-01"))
        // Inject trace context into MQTT user properties (example function)
        props := make(map[string]string)
        otel.GetTextMapPropagator().Inject(ctx, mqttTextMapCarrier(props)) // carrier adapter
        payload := []byte("sensor=42")
        // publish with user properties (broker must support v5)
        token := mc.PublishWithProperties("sensors/telemetry", 0, false, payload, props)
        token.WaitTimeout(2*time.Second)
        span.End()
}

// mqttTextMapCarrier adapts map[string]string for OpenTelemetry propagation.
type mqttTextMapCarrier map[string]string
func (c mqttTextMapCarrier) Get(key string) string { return c[key] }
func (c mqttTextMapCarrier) Set(key, val string)  { c[key] = val }
func (c mqttTextMapCarrier) Keys() []string {
        keys := make([]string, 0, len(c))
        for k := range c { keys = append(keys, k) }
        return keys
}