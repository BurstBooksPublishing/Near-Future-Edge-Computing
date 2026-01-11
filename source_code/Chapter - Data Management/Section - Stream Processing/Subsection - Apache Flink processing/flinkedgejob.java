import org.apache.flink.api.common.eventtime.WatermarkStrategy;
import org.apache.flink.api.common.serialization.SimpleStringSchema;
import org.apache.flink.api.common.state.ValueState;
import org.apache.flink.api.common.state.ValueStateDescriptor;
import org.apache.flink.contrib.streaming.state.RocksDBStateBackend;
import org.apache.flink.streaming.api.CheckpointingMode;
import org.apache.flink.streaming.api.datastream.SingleOutputStreamOperator;
import org.apache.flink.streaming.api.environment.CheckpointConfig.ExternalizedCheckpointCleanup;
import org.apache.flink.streaming.api.environment.StreamExecutionEnvironment;
import org.apache.flink.streaming.connectors.kafka.FlinkKafkaConsumer;

import java.time.Duration;
import java.util.Properties;

public class EdgeAnomalyJob {
  public static void main(String[] args) throws Exception {
    StreamExecutionEnvironment env = StreamExecutionEnvironment.getExecutionEnvironment();

    // Checkpointing tuned for edge: frequent incremental checkpoints, retained externally
    env.enableCheckpointing(10000L, CheckpointingMode.EXACTLY_ONCE); // every 10s
    env.getCheckpointConfig().setMinPauseBetweenCheckpoints(2000L);
    env.getCheckpointConfig().setCheckpointTimeout(60000L);
    env.getCheckpointConfig().enableExternalizedCheckpoints(ExternalizedCheckpointCleanup.RETAIN_ON_CANCELLATION);

    // RocksDB on local disk for large state; requires flink-statebackend-rocksdb artifact
    env.setStateBackend(new RocksDBStateBackend("file:///var/lib/flink/checkpoints", true));

    Properties kafkaProps = new Properties();
    kafkaProps.setProperty("bootstrap.servers", "kafka-edge:9092");
    kafkaProps.setProperty("group.id", "edge-anomaly-group");
    FlinkKafkaConsumer consumer = new FlinkKafkaConsumer<>("vibration-topic", new SimpleStringSchema(), kafkaProps);
    consumer.assignTimestampsAndWatermarks(
        WatermarkStrategy.forBoundedOutOfOrderness(Duration.ofMillis(500))
            .withTimestampAssigner((event, ts) -> parseTimestamp(event))); // parseTimestamp extracts event time

    SingleOutputStreamOperator processed = env.addSource(consumer)
      .map(raw -> computeFeatures(raw)) // feature extraction; avoid allocations
      .keyBy(record -> record.machineId)
      .process(new KeyedAnomalyScoreProcess()); // maintains ValueState for rolling baseline

    processed.addSink(new EdgeAlertSink()); // send to local MQTT/Kafka/HTTP

    env.execute("Edge Anomaly Scoring Job");
  }

  // Helper methods and process function implemented separately (omitted for brevity).
}