# aggregator_client.py
# Requires: paho-mqtt, torch, numpy
import io, json, base64, threading, time
import torch, paho.mqtt.client as mqtt

BROKER = "mqtt-broker.local"  # run Mosquitto on edge gateway
TOPIC_UPDATES = "fed/updates"  # clients publish their updates here
TOPIC_MODEL = "fed/model"      # server publishes new global model here
QOS = 1

def serialize_state(state_dict):
    buf = io.BytesIO()
    torch.save(state_dict, buf)
    return base64.b64encode(buf.getvalue()).decode('ascii')

def deserialize_state(b64):
    buf = io.BytesIO(base64.b64decode(b64))
    return torch.load(buf, map_location='cpu')

class Aggregator:
    def __init__(self, model_template):
        self.model = model_template.state_dict()
        self.lock = threading.Lock()
        self.updates = []  # list of (n_k, state_dict)
        self.client = mqtt.Client("aggregator")
        self.client.tls_set()  # enable TLS in production; configure certs
        self.client.on_message = self._on_message
        self.client.connect(BROKER)
        self.client.subscribe(TOPIC_UPDATES, qos=QOS)
        self.client.loop_start()

    def _on_message(self, client, userdata, msg):
        payload = json.loads(msg.payload.decode('utf-8'))
        n_k = int(payload['n_k'])
        state_b64 = payload['state']
        state = deserialize_state(state_b64)
        with self.lock:
            self.updates.append((n_k, state))

    def aggregate_round(self):
        time.sleep(1)  # wait for clients to publish
        with self.lock:
            if not self.updates:
                return
            total = sum(n for n, _ in self.updates)
            # initialize accumulator
            agg = {k: torch.zeros_like(torch.tensor(v)) for k, v in self.model.items()}
            for n_k, st in self.updates:
                for k in self.model.keys():
                    agg[k] += (n_k / total) * st[k]
            self.model = {k: agg[k] for k in agg}
            self.updates.clear()
        # publish new global model
        msg = json.dumps({'state': serialize_state(self.model)})
        self.client.publish(TOPIC_MODEL, msg, qos=QOS)

class Client:
    def __init__(self, client_id, local_model, data_loader):
        self.id = client_id
        self.model = local_model
        self.loader = data_loader
        self.mqtt = mqtt.Client(f"client-{client_id}")
        self.mqtt.tls_set()
        self.mqtt.on_message = self._on_model
        self.mqtt.connect(BROKER)
        self.mqtt.subscribe(TOPIC_MODEL, qos=QOS)
        self.mqtt.loop_start()
        self.latest_global = None

    def _on_model(self, client, userdata, msg):
        payload = json.loads(msg.payload.decode('utf-8'))
        self.latest_global = deserialize_state(payload['state'])

    def local_train_and_publish(self, epochs=1):
        if self.latest_global:
            self.model.load_state_dict(self.latest_global)
        # perform E epochs of local SGD (omitted: standard PyTorch training loop)
        # ... train here ...
        n_k = len(self.loader.dataset)
        state = serialize_state(self.model.state_dict())
        msg = json.dumps({'client_id': self.id, 'n_k': n_k, 'state': state})
        self.mqtt.publish(TOPIC_UPDATES, msg, qos=QOS)