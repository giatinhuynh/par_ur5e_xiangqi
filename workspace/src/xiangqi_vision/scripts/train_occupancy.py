#!/usr/bin/env python3
"""
Train the ResNet-34 binary occupancy classifier for Xiangqi board cells.

Usage:
    python3 train_occupancy.py --dataset /path/to/dataset --output occupancy.pth

Dataset layout (produced by CellDataCollector):
    dataset/
        empty/       *.jpg
        occupied/    *.jpg

Training follows the two-phase fine-tuning strategy from OpenChessRobot
(Frontiers in Robotics and AI, 2025):
  Phase 1 — freeze all layers except FC; lr=1e-3, 10 epochs
  Phase 2 — unfreeze all layers; lr=1e-4, 40 epochs
Total: 50 epochs, matching the paper.

Recommend ~300+ images (150+ per class) before training.
Achieves ~100% accuracy on the local setup within 50 epochs in the paper.
"""

import argparse
import os

import torch
import torch.nn as nn
from torch.utils.data import DataLoader, random_split
import torchvision.models as models
import torchvision.transforms as T
from torchvision.datasets import ImageFolder


CELL_SIZE   = 64
BATCH_SIZE  = 32
PHASE1_LR   = 1e-3
PHASE2_LR   = 1e-4
PHASE1_EPOCHS = 10
PHASE2_EPOCHS = 40

IMAGENET_MEAN = [0.485, 0.456, 0.406]
IMAGENET_STD  = [0.229, 0.224, 0.225]


def build_model() -> nn.Module:
    net = models.resnet34(weights=models.ResNet34_Weights.IMAGENET1K_V1)
    net.fc = nn.Linear(512, 2)
    return net


def make_loaders(dataset_dir: str, val_split: float = 0.2):
    train_tf = T.Compose([
        T.Resize((CELL_SIZE, CELL_SIZE)),
        T.RandomHorizontalFlip(),
        T.RandomRotation(15),
        T.ColorJitter(brightness=0.4, contrast=0.4, saturation=0.3, hue=0.05),
        T.ToTensor(),
        T.Normalize(IMAGENET_MEAN, IMAGENET_STD),
    ])
    val_tf = T.Compose([
        T.Resize((CELL_SIZE, CELL_SIZE)),
        T.ToTensor(),
        T.Normalize(IMAGENET_MEAN, IMAGENET_STD),
    ])

    full = ImageFolder(dataset_dir, transform=train_tf)
    n_val = max(1, int(len(full) * val_split))
    n_train = len(full) - n_val
    train_ds, val_ds = random_split(full, [n_train, n_val],
                                    generator=torch.Generator().manual_seed(42))
    val_ds.dataset = ImageFolder(dataset_dir, transform=val_tf)

    train_loader = DataLoader(train_ds, batch_size=BATCH_SIZE, shuffle=True,  num_workers=2)
    val_loader   = DataLoader(val_ds,   batch_size=BATCH_SIZE, shuffle=False, num_workers=2)
    print(f'Dataset: {len(full)} images — train {n_train}, val {n_val}')
    print(f'Classes: {full.classes}')
    return train_loader, val_loader


def train_epoch(net, loader, optimiser, criterion, device):
    net.train()
    total_loss = correct = total = 0
    for imgs, labels in loader:
        imgs, labels = imgs.to(device), labels.to(device)
        optimiser.zero_grad()
        out = net(imgs)
        loss = criterion(out, labels)
        loss.backward()
        optimiser.step()
        total_loss += loss.item() * len(imgs)
        correct += (out.argmax(1) == labels).sum().item()
        total += len(imgs)
    return total_loss / total, correct / total


@torch.no_grad()
def eval_epoch(net, loader, criterion, device):
    net.eval()
    total_loss = correct = total = 0
    for imgs, labels in loader:
        imgs, labels = imgs.to(device), labels.to(device)
        out = net(imgs)
        total_loss += criterion(out, labels).item() * len(imgs)
        correct += (out.argmax(1) == labels).sum().item()
        total += len(imgs)
    return total_loss / total, correct / total


def run_phase(net, train_loader, val_loader, n_epochs, lr, device, label):
    optimiser = torch.optim.Adam(
        filter(lambda p: p.requires_grad, net.parameters()), lr=lr
    )
    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(optimiser, T_max=n_epochs)
    criterion = nn.CrossEntropyLoss()
    best_val_acc = 0.0
    best_state   = None

    for epoch in range(1, n_epochs + 1):
        tr_loss, tr_acc = train_epoch(net, train_loader, optimiser, criterion, device)
        vl_loss, vl_acc = eval_epoch(net, val_loader, criterion, device)
        scheduler.step()
        print(f'  [{label}] epoch {epoch:3d}/{n_epochs}  '
              f'train {tr_acc:.3f}  val {vl_acc:.3f}  loss {vl_loss:.4f}')
        if vl_acc >= best_val_acc:
            best_val_acc = vl_acc
            best_state   = {k: v.clone() for k, v in net.state_dict().items()}

    if best_state:
        net.load_state_dict(best_state)
    print(f'  [{label}] best val acc = {best_val_acc:.4f}')
    return best_val_acc


def main():
    parser = argparse.ArgumentParser(description='Train ResNet-34 cell occupancy classifier')
    parser.add_argument('--dataset', required=True, help='Root dir with empty/ and occupied/ subdirs')
    parser.add_argument('--output',  required=True, help='Output .pth model path')
    parser.add_argument('--device',  default='cpu', help='torch device (cpu / cuda / mps)')
    args = parser.parse_args()

    if not os.path.isdir(args.dataset):
        raise SystemExit(f'Dataset directory not found: {args.dataset}')

    device = torch.device(args.device)
    train_loader, val_loader = make_loaders(args.dataset)

    net = build_model().to(device)

    # Phase 1: only FC trains
    for p in net.parameters():
        p.requires_grad = False
    for p in net.fc.parameters():
        p.requires_grad = True
    print(f'\n=== Phase 1: FC only, lr={PHASE1_LR}, {PHASE1_EPOCHS} epochs ===')
    run_phase(net, train_loader, val_loader, PHASE1_EPOCHS, PHASE1_LR, device, 'P1')

    # Phase 2: all layers
    for p in net.parameters():
        p.requires_grad = True
    print(f'\n=== Phase 2: all layers, lr={PHASE2_LR}, {PHASE2_EPOCHS} epochs ===')
    best_acc = run_phase(net, train_loader, val_loader, PHASE2_EPOCHS, PHASE2_LR, device, 'P2')

    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    torch.save(net.state_dict(), args.output)
    print(f'\nModel saved to {args.output}  (val acc={best_acc:.4f})')


if __name__ == '__main__':
    main()
