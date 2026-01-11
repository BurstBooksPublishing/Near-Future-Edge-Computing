import com.hivemq.client.mqtt.MqttClient;
import com.hivemq.client.mqtt.MqttGlobalPublishFilter;
import reactor.core.publisher.Flux;
import reactor.core.scheduler.Schedulers;
import java.time.Duration;

public class ReactiveMqttProcessor {
  public static void main(String[] args) {
    var client = MqttClient.builder()
      .useMqttVersion3()
      .identifier("edge-node-01")
      .serverHost("broker.example.local")
      .serverPort(1883)
      .buildBlocking();

    client.connect(); // keep alive and TLS configured externally

    // Create a Flux from MQTT publishes; backpressure via buffer and onBackpressureDrop.
    Flux.fromIterable(client.publishes(MqttGlobalPublishFilter.ALL, true))
      .map(pub -> pub.getTopic().toString() + ":" + new String(pub.getPayloadAsBytes()))
      .onBackpressureBuffer(1024, dropped -> { /* log dropped messages */ })
      .onBackpressureDrop(msg -> { /* metrics increment */ })
      .parallel()                       // parallelize processing
      .runOn(Schedulers.newParallel("proc", 4)) // match to CPU cores
      .flatMap(msg -> Flux.just(msg)
        .delayUntil(m -> processAsync(m)) // non-blocking processing
        .timeout(Duration.ofSeconds(2))   // protect against slow handlers
        .onErrorResume(ex -> handleError(ex, msg)))
      .sequential()
      .subscribe(
        s -> {/* ack metrics */},
        err -> {/* global error handler */});
  }

  // Example: asynchronous I/O-bound processing (e.g., model inference offload).
  static reactor.core.publisher.Mono processAsync(String msg) {
    return reactor.core.publisher.Mono.fromRunnable(() -> {
      // CPU/accelerator work: offload to GPU with vendor API or call local inference runtime.
    }).subscribeOn(Schedulers.boundedElastic()); // allow blocking accelerator calls safely
  }

  static reactor.core.publisher.Mono handleError(Throwable ex, String msg) {
    // resilience: circuit-breaker or retry strategy can be implemented here.
    return reactor.core.publisher.Mono.empty();
  }
}