import os, json, time, shutil
import numpy as np
import paho.mqtt.client as mqtt
import torch
import torch.nn as nn
import torch.optim as optim

# model: simple logistic linear layer
class LogisticModel(nn.Module):
    def __init__(self, dim):
        super().__init__()
        self.linear = nn.Linear(dim, 1)

    def forward(self, x):
        return torch.sigmoid(self.linear(x))

# configuration
MODEL_PATH = "/var/local/models/logistic.pt"
TEMP_PATH = MODEL_PATH + ".tmp"
MQTT_BROKER = "192.0.2.10"  # local broker or fog node
MQTT_TOPIC = "edge/models/deltas"
DEVICE = torch.device("cuda" if torch.cuda.is_available() else "cpu")

# load or init
dim = 128
model = LogisticModel(dim).to(DEVICE)
if os.path.exists(MODEL_PATH):
    model.load_state_dict(torch.load(MODEL_PATH, map_location=DEVICE))

optim = optim.SGD(model.parameters(), lr=1e-3)

client = mqtt.Client()
client.connect(MQTT_BROKER, 1883, 60)

def atomic_checkpoint(model, path=MODEL_PATH, tmp=TEMP_PATH):
    torch.save(model.state_dict(), tmp)
    shutil.move(tmp, path)  # atomic rename on POSIX

def publish_delta(model, topic=MQTT_TOPIC):
    payload = {k: v.cpu().numpy().tolist() for k, v in model.state_dict().items()}
    client.publish(topic, json.dumps(payload), qos=1)

def online_update(x_np, y_label):
    # single-sample update; x_np is numpy array, y_label in {0,1}
    model.train()
    x = torch.from_numpy(x_np.astype(np.float32)).to(DEVICE).unsqueeze(0)
    y = torch.tensor([[float(y_label)]], device=DEVICE)
    optim.zero_grad()
    loss = nn.BCELoss()(model(x), y)
    loss.backward()
    optim.step()
    # checkpoint and publish at controlled intervals
    atomic_checkpoint(model)
    publish_delta(model)

# Example call with sensor-derived feature vector
# online_update(sensor_features, label)