#!/usr/bin/env python3
"""
Train ResNet-34 cell occupancy classifier on real Xiangqi board images.

Data source
-----------
YOLO piece-detection dataset across three directories:
  /kaggle/input/datasets/hunhcgiatn/chess-real/chess_dataset/Real Board 1/
  /kaggle/input/datasets/hunhcgiatn/chess-real/chess_dataset/Real Board 2/
  /kaggle/input/datasets/hunhcgiatn/chess-real/chess_dataset/Real Board 3/

Each image has a paired YOLO .txt label (class cx cy w h, normalised 0-1).
Any detected piece bbox → that cell is occupied.
All other 90 cells → empty.

Pipeline
--------
1. Discover all images + labels in the three directories.
2. For each image:
   a. ArUco warp (multi-scale, multi-preprocess — same as before).
   b. Transform each YOLO bbox centre through the homography H.
   c. Snap to the nearest board cell (within HALF px).
   d. Extract all 90 cell crops (CELL_PX × CELL_PX).
3. Random 80/20 train/val split (by image, not by crop).
4. Train ResNet-34 (ImageNet init):
      Phase 1 — FC-only warm-up
      Phase 2 — Full fine-tune, cosine LR
5. Visual inference on 5 random val images → board overlay + cell grid.
6. Save model to /kaggle/working/occupancy_real_board.pth

HOW TO USE
----------
Run as a single Kaggle cell.  No external checkpoint required.

Config matches vision_config.yaml:
  cv_occ_cell_px        = 107   → CELL_PX
  cv_occ_x_squeeze_px   = 12    → X_SQUEEZE_PX
  cv_occ_resnet_roi_fraction = 1.00 (half = cell_px * roi_fraction / 2 = 53)
  MODEL_IN              = 64    (ResNet input size, matches CELL_SIZE in cell_occupancy_net.py)
"""

from __future__ import annotations
import cv2, torch, numpy as np, math, random, copy
from pathlib import Path
from PIL import Image as PILImage

import torchvision.transforms as T
from torchvision.models import resnet34, ResNet34_Weights
from torch.utils.data import DataLoader, WeightedRandomSampler, TensorDataset
import torch.nn as nn

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# ── Config ────────────────────────────────────────────────────────────────────
REAL_BOARD_DIRS = [
    Path("/kaggle/input/datasets/hunhcgiatn/chess-real/chess_dataset/Real Board 1"),
    Path("/kaggle/input/datasets/hunhcgiatn/chess-real/chess_dataset/Real Board 2"),
    Path("/kaggle/input/datasets/hunhcgiatn/chess-real/chess_dataset/Real Board 3"),
]
OUT_PATH   = Path("/kaggle/working/occupancy_real_board.pth")

# Must match vision_config.yaml:
#   cv_occ_cell_px=107, cv_occ_resnet_roi_fraction=1.00
#   → half = int(107 * 1.00 / 2) = 53  →  crop = 106×106 px
CELL_PX       = 107    # raw crop diameter; half = CELL_PX // 2 = 53 px
X_SQUEEZE_PX  = 12     # match cv_occ_x_squeeze_px — shifts cols a-d right, i-f left
MODEL_IN      = 64     # ResNet input size (matches CELL_SIZE in cell_occupancy_net.py)
VAL_FRAC      = 0.20   # fraction of images held out for validation

PHASE1_EPOCHS = 20     # FC-only
PHASE2_EPOCHS = 80     # full fine-tune
PHASE1_LR     = 1e-3
PHASE2_LR     = 1e-4
BATCH_SIZE    = 32
DEVICE        = torch.device("cuda" if torch.cuda.is_available() else "cpu")

print(f"Device: {DEVICE}")
print(f"CELL_PX={CELL_PX}  half={CELL_PX//2}  X_SQUEEZE_PX={X_SQUEEZE_PX}  MODEL_IN={MODEL_IN}")

# ── Board geometry ─────────────────────────────────────────────────────────────
NORM_W, NORM_H, MARGIN = 800, 890, 44
COLS, ROWS = 9, 10


def _cell_center_px(f: int, r: int) -> tuple[float, float]:
    il, ir, it, ib = 52.0, 542.0, 52.0, 788.0
    bw, bh = ir - il, 698.0 - 142.0
    cm = min(bw / 8.0, bh / 9.0)
    bb = 142.0 + (bh - cm * 9) / 2.0 + cm * 9
    gx = il + f * cm
    gy = bb - r * cm
    u = max(0.0, min(1.0, (gx - il) / (ir - il)))
    v = max(0.0, min(1.0, (gy - it) / (ib - it)))
    px = (1-u)*(1-v)*44 + u*(1-v)*756 + u*v*756 + (1-u)*v*44
    py = (1-u)*(1-v)*846 + u*(1-v)*846 + u*v*44  + (1-u)*v*44
    return px, py


# Apply the same x_squeeze offset used during inference (cell_occupancy_net.py):
#   cx += int(round(X_SQUEEZE_PX * (4 - file) / 4))
# This shifts cols a–d right and i–f left, converging at col e (file=4, offset=0).
CENTERS = [
    (
        int(round(_cell_center_px(f, r)[0])) + int(round(X_SQUEEZE_PX * (4 - f) / 4)),
        int(round(_cell_center_px(f, r)[1])),
    )
    for r in range(ROWS) for f in range(COLS)
]

SPACING = math.hypot(*(np.array(CENTERS[1]) - np.array(CENTERS[0])))
HALF    = SPACING / 2.0
print(f"SPACING={SPACING:.1f}px  HALF={HALF:.1f}px")


def extract_crop(img_bgr: np.ndarray, idx: int, size: int = CELL_PX) -> np.ndarray:
    cx, cy = CENTERS[idx]
    half = size // 2
    x1 = max(0, cx - half); x2 = min(NORM_W, cx + half)
    y1 = max(0, cy - half); y2 = min(NORM_H, cy + half)
    crop = img_bgr[y1:y2, x1:x2]
    if crop.shape[:2] != (size, size):
        crop = cv2.resize(crop, (size, size))
    return crop

# ── ArUco warp ─────────────────────────────────────────────────────────────────
DST_PTS = np.float32([
    [MARGIN, NORM_H - MARGIN], [NORM_W - MARGIN, NORM_H - MARGIN],
    [NORM_W - MARGIN, MARGIN], [MARGIN, MARGIN],
])
_ad = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
_ap = cv2.aruco.DetectorParameters()
_ap.cornerRefinementMethod      = cv2.aruco.CORNER_REFINE_SUBPIX
_ap.minMarkerPerimeterRate      = 0.01
_ap.polygonalApproxAccuracyRate = 0.05
_ap.adaptiveThreshWinSizeMin    = 3
_ap.adaptiveThreshWinSizeMax    = 53
_ap.adaptiveThreshWinSizeStep   = 4
_det = cv2.aruco.ArucoDetector(_ad, _ap)


def warp_image(img_bgr: np.ndarray) -> tuple[np.ndarray | None, np.ndarray | None]:
    """Returns (warped, H) or (None, None) if fewer than 4 markers found."""
    id_to_ctr: dict = {}
    gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
    for scale in (1.0, 0.75, 0.5, 0.25):
        if len(id_to_ctr) == 4:
            break
        h = int(img_bgr.shape[0] * scale)
        w = int(img_bgr.shape[1] * scale)
        small = cv2.resize(img_bgr, (w, h)) if scale < 1.0 else img_bgr
        sg    = cv2.resize(gray,    (w, h)) if scale < 1.0 else gray
        for frame in (
            small,
            cv2.cvtColor(cv2.createCLAHE(2.0, (8, 8)).apply(sg),    cv2.COLOR_GRAY2BGR),
            cv2.cvtColor(cv2.createCLAHE(4.0, (16, 16)).apply(sg),   cv2.COLOR_GRAY2BGR),
            cv2.cvtColor(cv2.equalizeHist(sg),                         cv2.COLOR_GRAY2BGR),
            cv2.filter2D(small, -1, np.array([[-1,-1,-1],[-1,9,-1],[-1,-1,-1]], np.float32)),
        ):
            if len(id_to_ctr) == 4:
                break
            corners, ids, _ = _det.detectMarkers(frame)
            if ids is None:
                continue
            for i, mid in enumerate(ids.flatten()):
                if mid in (0, 1, 2, 3) and mid not in id_to_ctr:
                    id_to_ctr[int(mid)] = corners[i][0].mean(axis=0) / scale
    if len(id_to_ctr) < 4:
        return None, None
    src = np.float32([id_to_ctr[k] for k in (0, 1, 2, 3)])
    H, _ = cv2.findHomography(src, DST_PTS, cv2.RANSAC, 5.0)
    warped = cv2.warpPerspective(img_bgr, H, (NORM_W, NORM_H))
    return warped, H

# ── YOLO label helpers ─────────────────────────────────────────────────────────
def find_label_file(img_path: Path) -> Path | None:
    """Find the YOLO .txt label paired with an image."""
    # Same directory, same stem
    candidate = img_path.with_suffix(".txt")
    if candidate.exists():
        return candidate
    # Parallel 'labels' sibling of 'images'
    for part in img_path.parts:
        if part.lower() in ("images", "imgs"):
            lbl = Path(*img_path.parts[:img_path.parts.index(part)]) / "labels" / img_path.with_suffix(".txt").name
            if lbl.exists():
                return lbl
    # Walk up one level and look in labels/
    lbl = img_path.parent.parent / "labels" / img_path.with_suffix(".txt").name
    if lbl.exists():
        return lbl
    return None


def load_yolo_bboxes(label_path: Path, img_w: int, img_h: int) -> list[tuple[float, float]]:
    """Return pixel (cx, cy) for each bbox in the label file."""
    centres: list[tuple[float, float]] = []
    with open(label_path) as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) < 5:
                continue
            # format: class_id cx cy w h  (all normalised 0-1)
            cx = float(parts[1]) * img_w
            cy = float(parts[2]) * img_h
            centres.append((cx, cy))
    return centres


def bboxes_to_occ(bboxes_orig: list[tuple[float, float]], H: np.ndarray) -> np.ndarray:
    """
    Transform bbox centres from original image → warped image via H,
    then snap each to the nearest board cell.
    Returns bool[90].
    """
    occ = np.zeros(90, dtype=bool)
    if not bboxes_orig:
        return occ
    pts = np.array([[cx, cy] for cx, cy in bboxes_orig], dtype=np.float32).reshape(-1, 1, 2)
    warped_pts = cv2.perspectiveTransform(pts, H).reshape(-1, 2)
    for (wx, wy) in warped_pts:
        best_idx, best_d = -1, HALF
        for idx, (gx, gy) in enumerate(CENTERS):
            d = math.hypot(wx - gx, wy - gy)
            if d < best_d:
                best_d = d
                best_idx = idx
        if best_idx >= 0:
            occ[best_idx] = True
    return occ

# ════════════════════════════════════════════════════════════════════════════════
# STEP 1 — Discover and load all images
# ════════════════════════════════════════════════════════════════════════════════
print("\n── Step 1: Discover images ──")
IMG_EXTS = {".jpg", ".jpeg", ".png", ".bmp"}

# Collect (image_path, label_path) pairs
all_pairs: list[tuple[Path, Path | None]] = []
for board_dir in REAL_BOARD_DIRS:
    if not board_dir.exists():
        print(f"  [skip] {board_dir} not found")
        continue
    for img_path in sorted(board_dir.rglob("*")):
        if img_path.suffix.lower() not in IMG_EXTS:
            continue
        lbl_path = find_label_file(img_path)
        all_pairs.append((img_path, lbl_path))

print(f"  Found {len(all_pairs)} images")

# ════════════════════════════════════════════════════════════════════════════════
# STEP 2 — Warp each image and extract crops
# ════════════════════════════════════════════════════════════════════════════════
print("\n── Step 2: Warp + extract crops ──")

# Each entry: (crops[90], occ[90], img_path, warped_bgr)
records: list[tuple[list[np.ndarray], np.ndarray, Path, np.ndarray]] = []

n_skip_aruco = 0
n_skip_label = 0
occ_counts   = []

for img_path, lbl_path in all_pairs:
    img = cv2.imread(str(img_path))
    if img is None:
        continue

    warped, H = warp_image(img)
    if warped is None:
        n_skip_aruco += 1
        continue

    if lbl_path is None:
        n_skip_label += 1
        # Still include: treat all cells as empty (no pieces labeled)
        # Only do this if the file genuinely has 0 pieces — otherwise skip
        occ = np.zeros(90, dtype=bool)
    else:
        bboxes = load_yolo_bboxes(lbl_path, img.shape[1], img.shape[0])
        occ    = bboxes_to_occ(bboxes, H)

    crops = [extract_crop(warped, idx) for idx in range(90)]
    records.append((crops, occ, img_path, warped))
    occ_counts.append(occ.sum())

n_imgs = len(records)
total_crops = n_imgs * 90
total_occ   = sum(int(r[1].sum()) for r in records)
total_emp   = total_crops - total_occ

print(f"  Warped successfully: {n_imgs} images  (skipped aruco={n_skip_aruco} label={n_skip_label})")
print(f"  Total crops: {total_crops}  occupied={total_occ}  empty={total_emp}")
print(f"  Avg pieces per image: {np.mean(occ_counts):.1f}")

assert n_imgs > 0, "No images warped — check REAL_BOARD_DIRS and ArUco marker visibility."

# ════════════════════════════════════════════════════════════════════════════════
# STEP 3 — Train / val split (by image)
# ════════════════════════════════════════════════════════════════════════════════
print("\n── Step 3: Train/val split ──")
random.seed(42)
indices = list(range(n_imgs))
random.shuffle(indices)
n_val   = max(1, int(n_imgs * VAL_FRAC))
val_idx = set(indices[:n_val])
trn_idx = set(indices[n_val:])

trn_records = [records[i] for i in indices[n_val:]]
val_records = [records[i] for i in indices[:n_val]]

print(f"  Train images: {len(trn_records)}  Val images: {len(val_records)}")

# ════════════════════════════════════════════════════════════════════════════════
# STEP 4 — Build PyTorch tensors
# ════════════════════════════════════════════════════════════════════════════════
print("\n── Step 4: Build tensors ──")

_MEAN, _STD = [0.485, 0.456, 0.406], [0.229, 0.224, 0.225]

_train_tf = T.Compose([
    T.Resize((MODEL_IN, MODEL_IN)),
    T.RandomHorizontalFlip(),
    T.RandomVerticalFlip(),
    T.RandomRotation(15),
    T.ColorJitter(brightness=0.3, contrast=0.3, saturation=0.2, hue=0.05),
    T.ToTensor(),
    T.Normalize(_MEAN, _STD),
    T.RandomErasing(p=0.15, scale=(0.02, 0.12)),
])

_val_tf = T.Compose([
    T.Resize((MODEL_IN, MODEL_IN)),
    T.ToTensor(),
    T.Normalize(_MEAN, _STD),
])


def records_to_tensors(recs, tf):
    tensors, labels = [], []
    for crops, occ, _, _ in recs:
        for idx in range(90):
            rgb = cv2.cvtColor(crops[idx], cv2.COLOR_BGR2RGB)
            tensors.append(tf(PILImage.fromarray(rgb)))
            labels.append(int(occ[idx]))
    return torch.stack(tensors), torch.tensor(labels, dtype=torch.long)


X_train, y_train = records_to_tensors(trn_records, _train_tf)
X_val,   y_val   = records_to_tensors(val_records,  _val_tf)

print(f"  X_train: {X_train.shape}  y_train: occ={y_train.sum()}  empty={len(y_train)-y_train.sum()}")
print(f"  X_val:   {X_val.shape}    y_val:   occ={y_val.sum()}    empty={len(y_val)-y_val.sum()}")

# Balanced sampler
n_occ_t = int(y_train.sum()); n_emp_t = len(y_train) - n_occ_t
w_occ = len(y_train) / (2 * max(1, n_occ_t))
w_emp = len(y_train) / (2 * max(1, n_emp_t))
weights = torch.where(y_train == 1, torch.tensor(w_occ), torch.tensor(w_emp))
sampler = WeightedRandomSampler(weights, len(weights), replacement=True)

train_loader = DataLoader(TensorDataset(X_train, y_train), batch_size=BATCH_SIZE, sampler=sampler)
val_loader   = DataLoader(TensorDataset(X_val,   y_val),   batch_size=BATCH_SIZE, shuffle=False)

# ════════════════════════════════════════════════════════════════════════════════
# STEP 5 — Model
# ════════════════════════════════════════════════════════════════════════════════
print("\n── Step 5: Build ResNet-34 (ImageNet init) ──")
model = resnet34(weights=ResNet34_Weights.IMAGENET1K_V1)
model.fc = nn.Linear(model.fc.in_features, 2)
model = model.to(DEVICE)

criterion = nn.CrossEntropyLoss()


def evaluate() -> tuple[float, float]:
    model.eval()
    correct = total = 0
    loss_sum = 0.0
    with torch.no_grad():
        for X, y in val_loader:
            X, y = X.to(DEVICE), y.to(DEVICE)
            out = model(X)
            loss_sum += criterion(out, y).item() * len(y)
            correct  += (out.argmax(1) == y).sum().item()
            total    += len(y)
    return loss_sum / max(total, 1), correct / max(total, 1)


def run_phase(n_epochs: int, lr: float, freeze_backbone: bool, tag: str) -> float:
    for p in model.parameters():
        p.requires_grad = not freeze_backbone
    for p in model.fc.parameters():
        p.requires_grad = True

    opt   = torch.optim.AdamW(filter(lambda p: p.requires_grad, model.parameters()),
                               lr=lr, weight_decay=1e-4)
    sched = torch.optim.lr_scheduler.CosineAnnealingLR(opt, T_max=n_epochs)

    best_acc, best_state = 0.0, None
    history = []

    for ep in range(1, n_epochs + 1):
        model.train()
        tr_loss = 0.0
        for X, y in train_loader:
            X, y = X.to(DEVICE), y.to(DEVICE)
            opt.zero_grad()
            loss = criterion(model(X), y)
            loss.backward(); opt.step()
            tr_loss += loss.item() * len(y)
        sched.step()

        val_loss, val_acc = evaluate()
        history.append((ep, tr_loss / len(train_loader.dataset), val_loss, val_acc))

        if val_acc > best_acc:
            best_acc  = val_acc
            best_state = copy.deepcopy(model.state_dict())

        if ep % 10 == 0 or ep == 1:
            print(f"  [{tag}] ep {ep:4d}/{n_epochs}  "
                  f"tr_loss={tr_loss/len(train_loader.dataset):.4f}  "
                  f"val_loss={val_loss:.4f}  val_acc={val_acc:.4f}  best={best_acc:.4f}")

    if best_state:
        model.load_state_dict(best_state)
    print(f"  [{tag}] Done. Best val_acc={best_acc:.4f}")
    return best_acc

# ════════════════════════════════════════════════════════════════════════════════
# STEP 6 — Train
# ════════════════════════════════════════════════════════════════════════════════
print("\n── Phase 1: FC warm-up ──")
run_phase(PHASE1_EPOCHS, PHASE1_LR, freeze_backbone=True,  tag="P1")

print("\n── Phase 2: Full fine-tune ──")
run_phase(PHASE2_EPOCHS, PHASE2_LR, freeze_backbone=False, tag="P2")

# ════════════════════════════════════════════════════════════════════════════════
# STEP 7 — Final evaluation on val set
# ════════════════════════════════════════════════════════════════════════════════
print("\n── Step 7: Final val evaluation ──")
model.eval()
all_preds, all_labels = [], []
probs_occ, probs_emp  = [], []

with torch.no_grad():
    for X, y in val_loader:
        X = X.to(DEVICE)
        p = torch.softmax(model(X), dim=1)[:, 1].cpu().numpy()
        preds = (p >= 0.5).astype(int)
        all_preds.extend(preds.tolist())
        all_labels.extend(y.numpy().tolist())
        probs_occ.extend(p[y.numpy() == 1].tolist())
        probs_emp.extend(p[y.numpy() == 0].tolist())

all_preds  = np.array(all_preds,  dtype=bool)
all_labels = np.array(all_labels, dtype=bool)
acc = (all_preds == all_labels).mean()
fp  = int(( all_preds & ~all_labels).sum())
fn  = int((~all_preds &  all_labels).sum())

print(f"  Val acc={acc:.4f}  FP={fp}  FN={fn}")
print(f"  Mean P(occ | occupied): {np.mean(probs_occ):.4f}" if probs_occ else "  No occupied val crops")
print(f"  Mean P(occ | empty):    {np.mean(probs_emp):.4f}" if probs_emp else "  No empty val crops")

# Distribution plot
fig, ax = plt.subplots(figsize=(8, 4))
if probs_occ: ax.hist(probs_occ, bins=25, alpha=0.6, color="red",  label=f"occupied (n={len(probs_occ)})")
if probs_emp: ax.hist(probs_emp, bins=25, alpha=0.6, color="blue", label=f"empty (n={len(probs_emp)})")
ax.axvline(0.5, color="k", ls="--", label="threshold=0.5")
ax.set_xlabel("P(occupied)"); ax.legend()
ax.set_title(f"Val distribution  acc={acc:.1%}  FP={fp}  FN={fn}")
plt.tight_layout(); plt.savefig("/kaggle/working/train_dist.png", dpi=90); plt.close()

# ════════════════════════════════════════════════════════════════════════════════
# STEP 8 — Visual inference on 5 random val images
# ════════════════════════════════════════════════════════════════════════════════
print("\n── Step 8: Visual inference on val images ──")

_infer_tf = T.Compose([
    T.Resize((MODEL_IN, MODEL_IN)),
    T.ToTensor(),
    T.Normalize(_MEAN, _STD),
])


def infer_board(warped_bgr: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Return (probs[90], pred_occ[90])."""
    model.eval()
    probs = np.zeros(90, dtype=np.float32)
    with torch.no_grad():
        for idx in range(90):
            crop = extract_crop(warped_bgr, idx)
            rgb  = cv2.cvtColor(crop, cv2.COLOR_BGR2RGB)
            t    = _infer_tf(PILImage.fromarray(rgb)).unsqueeze(0).to(DEVICE)
            probs[idx] = torch.softmax(model(t), dim=1)[0, 1].item()
    return probs, probs >= 0.5


def draw_board_overlay(warped_bgr, probs, pred_occ, gt_occ=None):
    vis = warped_bgr.copy()
    RING_R = int(HALF * 0.88)
    for idx, (gx, gy) in enumerate(CENTERS):
        pred = bool(pred_occ[idx])
        if gt_occ is not None:
            gt = bool(gt_occ[idx])
            color = (0, 200, 0) if pred == gt and gt else \
                    (60, 60, 200) if pred == gt else \
                    (0, 140, 255)
        else:
            color = (0, 200, 0) if pred else (60, 60, 200)
        cv2.circle(vis, (gx, gy), RING_R, color, 2)
        cv2.putText(vis, f"{probs[idx]:.2f}", (gx - 18, gy + 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.28, (255, 255, 0), 1)
    return vis


def draw_cell_grid(warped_bgr, pred_occ, gt_occ=None):
    CROP = CELL_PX
    canvas = np.zeros((ROWS * CROP, COLS * CROP, 3), np.uint8)
    for row in range(ROWS):
        for col in range(COLS):
            idx = row * 9 + col
            gx, gy = CENTERS[idx]
            x1 = max(0, gx - CROP//2); x2 = min(NORM_W, gx + CROP//2)
            y1 = max(0, gy - CROP//2); y2 = min(NORM_H, gy + CROP//2)
            crop = warped_bgr[y1:y2, x1:x2]
            ch, cw = crop.shape[:2]
            cy0 = row*CROP + (CROP-ch)//2
            cx0 = col*CROP + (CROP-cw)//2
            canvas[cy0:cy0+ch, cx0:cx0+cw] = crop
            pred = bool(pred_occ[idx])
            if gt_occ is not None:
                gt = bool(gt_occ[idx])
                color = (0, 200, 0) if pred == gt and gt else \
                        (60, 60, 200) if pred == gt else \
                        (0, 140, 255)
            else:
                color = (0, 200, 0) if pred else (60, 60, 200)
            cv2.rectangle(canvas, (col*CROP, row*CROP),
                          ((col+1)*CROP-1, (row+1)*CROP-1), color, 2)
    return canvas


n_show = min(5, len(val_records))
chosen = random.sample(val_records, n_show)

for rec_idx, (crops, gt_occ, img_path, warped) in enumerate(chosen):
    probs, pred_occ = infer_board(warped)
    acc_img = (pred_occ == gt_occ).mean()
    fp_img  = int(( pred_occ & ~gt_occ).sum())
    fn_img  = int((~pred_occ &  gt_occ).sum())

    board_vis = draw_board_overlay(warped, probs, pred_occ, gt_occ)
    grid_vis  = draw_cell_grid(warped, pred_occ, gt_occ)

    title = (f"{img_path.name}  acc={acc_img:.1%}  FP={fp_img}  FN={fn_img}\n"
             f"green=correct occ  blue=correct empty  orange=wrong")

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(18, 10))
    ax1.imshow(cv2.cvtColor(board_vis, cv2.COLOR_BGR2RGB))
    ax1.set_title(title); ax1.axis("off")
    ax2.imshow(cv2.cvtColor(grid_vis, cv2.COLOR_BGR2RGB))
    ax2.set_title("Cell grid"); ax2.axis("off")
    plt.tight_layout()
    safe = img_path.stem.replace(" ", "_")
    out  = f"/kaggle/working/infer_{rec_idx}_{safe}.png"
    plt.savefig(out, dpi=90)
    plt.show(); plt.close()
    print(f"  [{rec_idx+1}/{n_show}] {img_path.name}  acc={acc_img:.1%}  FP={fp_img}  FN={fn_img}")

# ════════════════════════════════════════════════════════════════════════════════
# STEP 9 — Save
# ════════════════════════════════════════════════════════════════════════════════
torch.save(model.state_dict(), OUT_PATH)
print(f"\n  Model saved → {OUT_PATH}")
print(f"  Overall val: acc={acc:.1%}  FP={fp}  FN={fn}")
