# app_functions_pipeline.py — requires edgex-app-functions-sdk-python
from app_functions_sdk.app import AppFunctions
from app_functions_sdk.models import AppInput
import paho.mqtt.client as mqtt
import ssl

THRESHOLD = 75.0  # degrees, example for industrial temperature
MQTT_BROKER = "mqtt.example.com"
MQTT_TOPIC = "factory/alerts/temp"

# Configure MQTT client with TLS and retries
mqttc = mqtt.Client()
mqttc.tls_set(cert_reqs=ssl.CERT_REQUIRED)  # production cert setup
mqttc.tls_insecure_set(False)
mqttc.connect(MQTT_BROKER, port=8883, keepalive=60)

def filter_temp(data: AppInput):
    # data.readings is a list of sensor readings from Core Data
    try:
        val = float(next(r for r in data.readings if r.name == "temperature").value)
    except StopIteration:
        return None
    return data if val > THRESHOLD else None

def publish_mqtt(data: AppInput):
    # Build compact JSON alert and publish synchronously with QoS=1
    payload = {
        "device": data.origin_device,
        "timestamp": data.origin_timestamp,
        "readings": [{r.name: r.value} for r in data.readings]
    }
    mqttc.publish(MQTT_TOPIC, payload=str(payload), qos=1)
    return data

if __name__ == "__main__":
    app = AppFunctions(config_file="config/configuration.toml")
    # Pipeline: ingress -> filter -> publish
    app.add(filter_temp)
    app.add(publish_mqtt)
    app.start()