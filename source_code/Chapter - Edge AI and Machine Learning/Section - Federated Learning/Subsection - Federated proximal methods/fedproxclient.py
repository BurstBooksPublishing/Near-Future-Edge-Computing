import copy
import flwr as fl
import torch
import numpy as np

# Utility: convert model parameters to list of numpy arrays
def params_to_ndarrays(model):
    return [p.detach().cpu().numpy() for p in model.state_dict().values()]

# Utility: load numpy arrays into model state_dict
def ndarrays_to_params(model, ndarrays):
    state = model.state_dict()
    for k, v in zip(state.keys(), ndarrays):
        state[k] = torch.tensor(v, device=next(model.parameters()).device)
    model.load_state_dict(state)

class FedProxClient(fl.client.NumPyClient):
    def __init__(self, model, train_loader, test_loader, device=None, mu=0.1, lr=1e-3):
        self.model = model.to(device or ("cuda" if torch.cuda.is_available() else "cpu"))
        self.train_loader = train_loader
        self.test_loader = test_loader
        self.device = next(self.model.parameters()).device
        self.mu = mu
        self.criterion = torch.nn.CrossEntropyLoss()
        self.optimizer = torch.optim.SGD(self.model.parameters(), lr=lr, momentum=0.9)

    def get_parameters(self):
        return params_to_ndarrays(self.model)

    def fit(self, parameters, config):
        # Load global weights
        ndarrays_to_params(self.model, parameters)
        global_state = copy.deepcopy(list(self.model.parameters()))
        # Local training with proximal term
        self.model.train()
        epochs = int(config.get("local_epochs", 1))
        for _ in range(epochs):
            for xb, yb in self.train_loader:
                xb, yb = xb.to(self.device), yb.to(self.device)
                self.optimizer.zero_grad()
                logits = self.model(xb)
                loss = self.criterion(logits, yb)
                # proximal regularizer: mu/2 * ||w - w_global||^2
                prox = 0.0
                for p, pg in zip(self.model.parameters(), global_state):
                    diff = p - pg.to(self.device)
                    prox += torch.sum(diff * diff)
                loss = loss + 0.5 * self.mu * prox
                loss.backward()
                self.optimizer.step()
        return params_to_ndarrays(self.model), len(self.train_loader.dataset), {}

    def evaluate(self, parameters, config):
        ndarrays_to_params(self.model, parameters)
        self.model.eval()
        correct = 0
        total = 0
        loss_sum = 0.0
        with torch.no_grad():
            for xb, yb in self.test_loader:
                xb, yb = xb.to(self.device), yb.to(self.device)
                logits = self.model(xb)
                loss_sum += float(self.criterion(logits, yb)) * xb.size(0)
                pred = logits.argmax(dim=1)
                correct += (pred == yb).sum().item()
                total += xb.size(0)
        return float(loss_sum / total), total, {"accuracy": correct / total}