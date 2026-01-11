import torch
import torch.nn.functional as F
from torch.utils.data import DataLoader

def distillation_train(teacher, student, train_ds, val_ds, device,
                       epochs=30, lr=1e-3, alpha=0.2, T=4.0):
    # Prepare models
    teacher.eval()
    student.to(device)
    teacher.to(device)
    opt = torch.optim.Adam(student.parameters(), lr=lr)
    sched = torch.optim.lr_scheduler.CosineAnnealingLR(opt, T_max=epochs)
    train_loader = DataLoader(train_ds, batch_size=64, shuffle=True, num_workers=4, pin_memory=True)
    val_loader = DataLoader(val_ds, batch_size=128, shuffle=False, num_workers=4, pin_memory=True)
    kl_loss = torch.nn.KLDivLoss(reduction='batchmean')

    for epoch in range(epochs):
        student.train()
        for images, labels in train_loader:
            images = images.to(device); labels = labels.to(device)
            with torch.no_grad():
                t_logits = teacher(images)                   # teacher logits
            s_logits = student(images)                       # student logits
            loss_ce = F.cross_entropy(s_logits, labels)     # supervised loss
            # KL on softened probabilities (use log_softmax for numerically stable KLDivLoss)
            loss_kd = kl_loss(F.log_softmax(s_logits / T, dim=1),
                              F.softmax(t_logits / T, dim=1)) * (T**2)
            loss = alpha * loss_ce + (1.0 - alpha) * loss_kd
            opt.zero_grad(); loss.backward(); opt.step()
        sched.step()
        # Validation pass (concise): compute top-1 accuracy on val_loader
        student.eval()
        correct = total = 0
        with torch.no_grad():
            for images, labels in val_loader:
                images = images.to(device); labels = labels.to(device)
                preds = student(images).argmax(dim=1)
                correct += (preds == labels).sum().item()
                total += labels.size(0)
        acc = correct / total
        print(f"Epoch {epoch+1}/{epochs}, Val Acc: {acc:.4f}, LR: {sched.get_last_lr()[0]:.2e}")

    # Export: trace and save TorchScript for mobile/runtime use
    example_input = torch.randn(1, 3, 224, 224).to(device)
    student.eval()
    traced = torch.jit.trace(student.cpu(), example_input.cpu())
    traced.save("student_traced.pt")  # deployable on PyTorch Mobile or convert to ONNX/TensorRT
    return student