import time
import torch
import torch.nn.functional as F

# device selection
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

# model must return list of logits for branches: [logits_b1, ..., logits_full]
model = torch.jit.load("early_exit_model.pt").to(device)
model.eval()

# calibrated temperature and per-branch thresholds (tuned on validation)
temperature = 1.2  # learned scalar
thresholds = [0.9, 0.85, 0.8]  # for K branches; last threshold applies before full model

@torch.no_grad()
def infer_batch(batch_tensor):
    batch_tensor = batch_tensor.to(device)
    start = time.perf_counter()
    logits_list = model(batch_tensor)  # list length K+1
    # iterate branches and check confidence; logits -> softmax with temperature
    for k, logits in enumerate(logits_list[:-1]):  # exclude final full-model logits
        probs = F.softmax(logits / temperature, dim=1)
        maxp, pred = probs.max(dim=1)
        # boolean mask of samples that can exit at this branch
        exit_mask = maxp >= thresholds[k]
        if exit_mask.any():
            # prepare outputs for exited samples; others continue
            exited_indices = exit_mask.nonzero(as_tuple=True)[0]
            results = {"indices": exited_indices.cpu(), "preds": pred[exit_mask].cpu()}
            elapsed = time.perf_counter() - start
            results["latency_s"] = elapsed
            return results
        # else continue to next branch without recomputing shared features
    # no early exit; use final logits
    final_logits = logits_list[-1]
    probs = F.softmax(final_logits / temperature, dim=1)
    maxp, pred = probs.max(dim=1)
    elapsed = time.perf_counter() - start
    return {"indices": torch.arange(batch_tensor.size(0)), "preds": pred.cpu(), "latency_s": elapsed}