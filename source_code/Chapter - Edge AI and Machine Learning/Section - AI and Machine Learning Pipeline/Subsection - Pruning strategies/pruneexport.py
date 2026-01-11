import torch
import torch.nn.utils.prune as prune
from torch.utils.data import DataLoader
# Assume model, train_dataset, val_dataset defined elsewhere and compatible with device
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
model.to(device)

def global_magnitude_prune(model, sparsity):
    # Apply global unstructured pruning (change to structured for filter pruning)
    parameters = [(n, p) for n, p in model.named_parameters() if 'weight' in n and p.dim() > 1]
    prune.global_unstructured(parameters, pruning_method=prune.L1Unstructured, amount=sparsity)

def finetune(model, train_dataset, epochs=3, lr=1e-4):
    model.train()
    opt = torch.optim.SGD(model.parameters(), lr=lr, momentum=0.9)
    loader = DataLoader(train_dataset, batch_size=32, shuffle=True, num_workers=2, pin_memory=True)
    for _ in range(epochs):
        for xb, yb in loader:
            xb, yb = xb.to(device), yb.to(device)
            opt.zero_grad()
            loss = torch.nn.functional.cross_entropy(model(xb), yb)
            loss.backward()
            opt.step()

# Iterative pruning schedule
target_sparsity = 0.8
steps = 4
for step in range(steps):
    s = 1 - (1 - target_sparsity) ** ((step+1)/steps)  # progressive schedule
    global_magnitude_prune(model, sparsity=s)
    finetune(model, train_dataset, epochs=2)

# Remove pruning reparameterizations to make weights permanent
for name, module in model.named_modules():
    for attr in list(module.__dict__):
        if isinstance(module, torch.nn.Module):
            try:
                prune.remove(module, 'weight')
            except Exception:
                pass

# Export to ONNX for TensorRT conversion on Jetson
model.eval()
dummy = torch.randn(1, 3, 224, 224, device=device)
torch.onnx.export(model, dummy, "pruned_model.onnx", opset_version=13, do_constant_folding=True)
# On Jetson: use trtexec or TensorRT Python API to build optimized engine from pruned_model.onnx