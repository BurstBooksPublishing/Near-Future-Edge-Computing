import org.apache.kafka.common.serialization.Serdes;
import org.apache.kafka.streams.*;
import org.apache.kafka.streams.kstream.*;

import java.time.Duration;
import java.util.Properties;

public class EdgeAnomalyStream {
  public static void main(String[] args) {
    Properties props = new Properties();
    props.put(StreamsConfig.APPLICATION_ID_CONFIG, "edge-anomaly-app");
    props.put(StreamsConfig.BOOTSTRAP_SERVERS_CONFIG, "kafka-broker:9092");
    props.put(StreamsConfig.PROCESSING_GUARANTEE_CONFIG, StreamsConfig.EXACTLY_ONCE_V2); // idempotent, transactional
    props.put(StreamsConfig.DEFAULT_KEY_SERDE_CLASS_CONFIG, Serdes.String().getClass());
    props.put(StreamsConfig.DEFAULT_VALUE_SERDE_CLASS_CONFIG, Serdes.Double().getClass());

    StreamsBuilder builder = new StreamsBuilder();
    KStream sensorStream = builder.stream("sensor-readings");

    KTable, Double> rms = sensorStream
      .groupByKey()
      .windowedBy(TimeWindows.ofSizeWithNoGrace(Duration.ofSeconds(10)))
      .aggregate(
        () -> 0.0,
        (key, value, agg) -> Math.sqrt((agg*agg + value*value) / 2.0), // rolling RMS placeholder
        Materialized.with(Serdes.String(), Serdes.Double())
      );

    rms.toStream()
       .map((windowedKey, value) -> new KeyValue<>(windowedKey.key(), value))
       .filter((key, rmsVal) -> rmsVal > 1.5) // anomaly threshold tuned per device type
       .to("anomaly-alerts", Produced.with(Serdes.String(), Serdes.Double()));

    KafkaStreams streams = new KafkaStreams(builder.build(), props);
    streams.start();
    Runtime.getRuntime().addShutdownHook(new Thread(streams::close));
  }
}