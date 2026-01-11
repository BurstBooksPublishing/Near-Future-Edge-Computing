import json, time, numpy as np, paho.mqtt.client as mqtt
import tflite_runtime.interpreter as tflite  # lightweight TFLite runtime

# Config (tune for deployment)
BROKER = "mqtt-broker.local"
IN_TOPIC = "factory/machine01/features"
ALERT_TOPIC = "factory/machine01/alerts"
MODEL_PATH = "/opt/models/ae.tflite"
INV_COV_PATH = "/opt/models/inv_cov.npy"  # precomputed inverse covariance
MEAN_PATH = "/opt/models/mean.npy"
THRESHOLD = 3.0  # combined score threshold

# Load model and statistics
interpreter = tflite.Interpreter(model_path=MODEL_PATH)
interpreter.allocate_tensors()
input_idx = interpreter.get_input_details()[0]['index']
output_idx = interpreter.get_output_details()[0]['index']
inv_cov = np.load(str(INV_COV_PATH))
mu = np.load(str(MEAN_PATH))

def mahalanobis(x):
    d = x - mu
    return float(np.sqrt(d.dot(inv_cov).dot(d)))

def recon_error(x):
    interpreter.set_tensor(input_idx, x.astype(np.float32))
    interpreter.invoke()
    recon = interpreter.get_tensor(output_idx)
    return float(np.linalg.norm(x - recon))

def on_message(client, userdata, msg):
    try:
        vec = np.array(json.loads(msg.payload.decode()), dtype=np.float32)
        dm = mahalanobis(vec)
        sr = recon_error(vec)
        # simple normalization by empirical constants or recent rolling std
        score = 0.5 * (sr / 1.0) + 0.5 * (dm / 1.0)
        if score > THRESHOLD:
            alert = {"ts": time.time(), "score": score, "dm": dm, "sr": sr}
            client.publish(ALERT_TOPIC, json.dumps(alert), qos=1)
    except Exception as e:
        # minimal logging for production; integrate with local syslog/edge telemetry
        pass

client = mqtt.Client()
client.on_message = on_message
client.connect(BROKER)
client.subscribe(IN_TOPIC, qos=1)
client.loop_forever()