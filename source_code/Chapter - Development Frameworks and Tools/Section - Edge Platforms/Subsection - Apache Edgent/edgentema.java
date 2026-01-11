import org.apache.edgent.providers.direct.DirectProvider;
import org.apache.edgent.topology.Topology;
import org.apache.edgent.topology.TStream;
import org.eclipse.paho.client.mqttv3.*;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

public class EdgentEmaPipeline {
    // Replace with real sensor read and broker credentials
    static double readSensor() { /* read ADC or I2C sensor; return double */ return 0.0; }

    public static void main(String[] args) throws Exception {
        // MQTT client setup (reuse across sink calls)
        String broker = "ssl://mqtt.example.local:8883";
        String clientId = "edge-node-001";
        MqttClient mqtt = new MqttClient(broker, clientId);
        MqttConnectOptions opts = new MqttConnectOptions();
        opts.setCleanSession(true);
        opts.setSocketFactory(TlsUtils.createSocketFactory("/certs/ca.pem","/certs/client.pem","/certs/client.key")); // helper
        mqtt.connect(opts);

        DirectProvider provider = new DirectProvider();
        Topology topology = provider.newTopology("ema-topology");

        // Poll sensor at 5ms intervals (200 Hz)
        TStream samples = topology.poll(() -> readSensor(), 5, TimeUnit.MILLISECONDS);

        // Exponential moving average with alpha; state captured in AtomicReference
        double alpha = 0.1;
        AtomicReference state = new AtomicReference<>(0.0);
        TStream ema = samples.map(s -> {
            double prev = state.get();
            double next = prev + alpha * (s - prev);
            state.set(next);
            return next;
        });

        // Alerting: publish when EMA exceeds threshold
        double threshold = 1.5; // domain-specific
        ema.filter(v -> v > threshold)
           .sink(v -> {
               // JSON payload compact; avoid heavy allocations
               String payload = String.format("{\"node\":\"%s\",\"ema\":%.3f}", clientId, v);
               MqttMessage msg = new MqttMessage(payload.getBytes("UTF-8"));
               msg.setQos(1);
               mqtt.publish("factory/line1/alerts", msg);
           });

        provider.submit(topology); // starts processing on current JVM
    }
}