import torch
import torch.nn as nn
import torch.optim as optim
from collections import deque
import random
import paho.mqtt.client as mqtt

# Simple replay buffer using feature-level compression (quantize to int8)
class ReplayBuffer:
    def __init__(self, max_size=1024):
        self.buf = deque(maxlen=max_size)
    def add(self, x, y):
        # store as float16 to save memory on edge
        self.buf.append((x.half(), y))
    def sample(self, k):
        k = min(k, len(self.buf))
        batch = random.sample(self.buf, k)
        xs = torch.stack([b[0].float() for b in batch]).to(device)
        ys = torch.tensor([b[1] for b in batch], device=device)
        return xs, ys

# EWC utility: estimate diagonal Fisher approximately on small minibatches
def estimate_fisher(model, data_loader, device, samples=200):
    model.eval()
    fisher = {n: torch.zeros_like(p) for n, p in model.named_parameters()}
    cnt = 0
    with torch.no_grad():
        for x, y in data_loader:
            if cnt > samples: break
            x, y = x.to(device), y.to(device)
            logits = model(x)
            loss = nn.functional.cross_entropy(logits, y)
            loss.backward()
            for n, p in model.named_parameters():
                if p.grad is not None:
                    fisher[n] += p.grad.data.clone().pow(2)
            model.zero_grad()
            cnt += 1
    for n in fisher: fisher[n] /= max(1, cnt)
    return fisher

# EWC loss addition
def ewc_loss(model, old_params, fisher, lambda_ewc):
    loss = 0.0
    for n, p in model.named_parameters():
        loss += (fisher[n] * (p - old_params[n]).pow(2)).sum()
    return 0.5 * lambda_ewc * loss

# Edge training loop (simplified)
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
model = MyConvNet().to(device)         # use TorchScript for deployment later
optimizer = optim.SGD(model.parameters(), lr=1e-3)
replay = ReplayBuffer(max_size=2048)

def train_on_task(task_loader, old_params=None, fisher=None, lambda_ewc=100):
    model.train()
    for x, y in task_loader:
        x, y = x.to(device), y.to(device)
        # replay mixed batch
        rx, ry = replay.sample(k=32)
        if len(rx) > 0:
            x = torch.cat([x, rx], dim=0); y = torch.cat([y, ry], dim=0)
        logits = model(x)
        loss = nn.functional.cross_entropy(logits, y)
        if old_params is not None:
            loss = loss + ewc_loss(model, old_params, fisher, lambda_ewc)
        optimizer.zero_grad()
        loss.backward()
        optimizer.step()
    # add exemplars (feature extraction optional)
    for x, y in task_loader:
        replay.add(x.mean(dim=[2,3]), int(y))  # compress to feature vector
        break

# Offload/restore model checkpoints via MQTT to cloud orchestrator (small binary blobs)