import time, json, numpy as np, paho.mqtt.client as mqtt
# Use tflite_runtime on constrained devices; fall back to onnxruntime on others.
try:
    import tflite_runtime.interpreter as tflite
    Interpreter = tflite.Interpreter
except Exception:
    import onnxruntime as ort
    Interpreter = None

MQTT_BROKER = "mqtt.example.local"
DEVICE_ID = "edge-unit-001"  # small identifier
TOPIC = f"telemetry/{DEVICE_ID}/fairness"

client = mqtt.Client()
client.connect(MQTT_BROKER)

# Minimalized model runner (TFLite preferred)
class ModelRunner:
    def __init__(self, model_path):
        if Interpreter:
            self.intp = Interpreter(model_path=model_path)
            self.intp.allocate_tensors()
        else:
            self.sess = ort.InferenceSession(model_path)

    def infer(self, input_tensor):
        if Interpreter:
            inp_idx = self.intp.get_input_details()[0]['index']
            out_idx = self.intp.get_output_details()[0]['index']
            self.intp.set_tensor(inp_idx, input_tensor)
            self.intp.invoke()
            return self.intp.get_tensor(out_idx)
        else:
            name = self.sess.get_inputs()[0].name
            return self.sess.run(None, {name: input_tensor})[0]

# Track per-group counts on device; avoid storing raw data to protect privacy.
class FairnessCounter:
    def __init__(self, groups):
        self.counts = {g:{"tp":0,"fp":0,"tn":0,"fn":0,"n":0} for g in groups}

    def update(self, group, pred, label):
        self.counts[group]["n"] += 1
        if pred==1 and label==1: self.counts[group]["tp"]+=1
        elif pred==1 and label==0: self.counts[group]["fp"]+=1
        elif pred==0 and label==0: self.counts[group]["tn"]+=1
        else: self.counts[group]["fn"]+=1

    def snapshot(self):
        return self.counts.copy()

# Example runtime loop (labels may come from edge sensors or delayed server annotations)
runner = ModelRunner("model.tflite")
fc = FairnessCounter(groups=["day","night"])
while True:
    # acquire input_tensor, group_label, true_label by board-specific code
    input_tensor = np.zeros((1,224,224,3), dtype=np.float32)  # placeholder
    group_label = "night"
    true_label = 0
    out = runner.infer(input_tensor)
    pred = int(out.argmax() == 1)
    fc.update(group_label, pred, true_label)
    if time.time() % 60 < 1:  # publish every ~60s
        payload = {"device": DEVICE_ID, "counts": fc.snapshot()}
        client.publish(TOPIC, json.dumps(payload), qos=1)
    time.sleep(0.1)