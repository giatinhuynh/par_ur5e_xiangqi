# Vision Model Training Guide

## YOLOv8n Fine-tuning for Xiangqi Piece Detection

---

## Overview

The detection pipeline uses **YOLOv8 Nano (YOLOv8n)** to detect and classify all 14 Xiangqi piece classes (7 piece types × 2 colours) on the board. This document covers the complete pipeline from dataset sourcing through model deployment.

**Strategy: Pretrain + Fine-tune**
1. Start from a public Roboflow Xiangqi dataset (hundreds of labelled images)
2. Capture ~50–100 custom images using the lab's actual camera setup
3. Fine-tune the pretrained weights on the combined dataset
4. Validate and export to ONNX for deployment

**Model inference rate target:** ≥ 15 FPS on the lab workstation CPU (real-time during game play).

---

## Class Map

The model detects 14 classes:

| Class ID | Label | Chinese | Colour |
|---|---|---|---|
| 0 | `r_general` | 帅 | Red |
| 1 | `r_advisor` | 仕 | Red |
| 2 | `r_elephant` | 相 | Red |
| 3 | `r_horse` | 马 | Red |
| 4 | `r_chariot` | 车 | Red |
| 5 | `r_cannon` | 炮 | Red |
| 6 | `r_soldier` | 兵 | Red |
| 7 | `b_general` | 将 | Black |
| 8 | `b_advisor` | 士 | Black |
| 9 | `b_elephant` | 象 | Black |
| 10 | `b_horse` | 马 | Black |
| 11 | `b_chariot` | 车 | Black |
| 12 | `b_cannon` | 炮 | Black |
| 13 | `b_soldier` | 卒 | Black |

These map directly to the `CLASS_MAP` in `xiangqi_vision/piece_detector.py`.

---

## Part 1: Environment Setup

### 1.1 Inside the Docker container

All training is done inside the project Docker container which already has `ultralytics` installed.

```bash
# Start the container (from your host machine)
cd /path/to/par_ur5e_xiangqi
docker compose up -d   # or the lab's standard docker run command
docker exec -it ur5e_xiangqi bash

# Verify Ultralytics is available
python3 -c "from ultralytics import YOLO; print('OK')"
```

If running training outside Docker (on a laptop with GPU), install:
```bash
pip install ultralytics roboflow opencv-python-headless numpy
```

### 1.2 Create dataset directory structure

```bash
mkdir -p ~/xiangqi_dataset/{images,labels}/{train,val,test}
mkdir -p ~/xiangqi_dataset/raw_captures
```

---

## Part 2: Acquiring the Base Dataset

### 2.1 Download the Roboflow public Xiangqi dataset

A labelled Xiangqi dataset is available on Roboflow Universe. This gives you a strong starting point before any lab captures.

```bash
pip install roboflow
```

```python
# save as: tools/download_roboflow_dataset.py
from roboflow import Roboflow

rf = Roboflow(api_key="YOUR_ROBOFLOW_API_KEY")  # Free account at roboflow.com

# Search for "xiangqi" or "chinese chess" on Roboflow Universe
# Primary recommended dataset (verify still exists):
project = rf.workspace("xiangqi-detection").project("xiangqi-pieces")
dataset = project.version(1).download("yolov8")

# The download creates: xiangqi-pieces-1/
#   data.yaml
#   train/images/, train/labels/
#   valid/images/, valid/labels/
#   test/images/,  test/labels/
```

> **Alternative:** If the exact workspace/project slug has changed, visit [roboflow.com/universe](https://universe.roboflow.com) and search for "xiangqi". Download the dataset in **YOLOv8 format**. Several datasets have 500–2000 labelled images.

**After downloading, verify the class names** in the `data.yaml` match the 14 classes in the table above. If the class names differ (e.g., `general` vs `r_general`), you will need to remap labels in Step 2.3.

### 2.2 Inspect the downloaded dataset

```bash
python3 - <<'EOF'
import yaml
from pathlib import Path
import glob

with open('xiangqi-pieces-1/data.yaml') as f:
    info = yaml.safe_load(f)

print("Classes:", info['names'])
print("Number of classes:", info['nc'])
train_imgs = glob.glob('xiangqi-pieces-1/train/images/*.jpg')
print(f"Train images: {len(train_imgs)}")
val_imgs   = glob.glob('xiangqi-pieces-1/valid/images/*.jpg')
print(f"Val images:   {len(val_imgs)}")
EOF
```

Expected output: 14 classes, 400–1500 training images.

### 2.3 Remap class IDs (if needed)

If the downloaded dataset uses different class IDs than the table above, run the remapping script:

```python
# save as: tools/remap_labels.py
"""
Remaps YOLO label files to match the project's class ID order.
Edit SRC_CLASSES to match the downloaded dataset's class order.
"""
from pathlib import Path

# Adjust these to match the downloaded data.yaml 'names' list
SRC_CLASSES = [
    'red_king', 'red_advisor', 'red_elephant', 'red_horse',
    'red_rook', 'red_cannon', 'red_pawn',
    'black_king', 'black_advisor', 'black_elephant', 'black_horse',
    'black_rook', 'black_cannon', 'black_pawn',
]

TGT_CLASSES = [
    'r_general', 'r_advisor', 'r_elephant', 'r_horse',
    'r_chariot', 'r_cannon', 'r_soldier',
    'b_general', 'b_advisor', 'b_elephant', 'b_horse',
    'b_chariot', 'b_cannon', 'b_soldier',
]

# Build mapping: src_id -> tgt_id
id_map = {}
for tgt_id, tgt_name in enumerate(TGT_CLASSES):
    for src_id, src_name in enumerate(SRC_CLASSES):
        # Fuzzy match: check if key parts match
        if src_name.split('_')[-1] in tgt_name and src_name.split('_')[0][0] == tgt_name[0]:
            id_map[src_id] = tgt_id
            break

print("ID mapping:", id_map)

label_dirs = list(Path('xiangqi-pieces-1').rglob('labels'))
for ldir in label_dirs:
    for lf in ldir.glob('*.txt'):
        lines = lf.read_text().strip().split('\n')
        new_lines = []
        for line in lines:
            if not line:
                continue
            parts = line.split()
            old_id = int(parts[0])
            new_id = id_map.get(old_id, old_id)
            new_lines.append(f"{new_id} {' '.join(parts[1:])}")
        lf.write_text('\n'.join(new_lines) + '\n')
        
print("Remapping complete.")
```

---

## Part 3: Capturing Lab Images

Lab-specific images are the most valuable data for fine-tuning because they match the exact camera, lighting, and board mat you will use in deployment.

### 3.1 Setup for capture

Before capturing:
- Mount the RealSense camera at its final position (same height and angle as during robot operation)
- Place the board mat in its exact operating position on the table
- Set up lab lighting as it will be during robot use (do not capture under very different lighting)
- Prepare both Red and Black piece sets

### 3.2 Image capture procedure

```bash
# Inside Docker, run the capture utility
ros2 run xiangqi_vision vision_node &   # Start camera node

# Alternatively, use the standalone capture script:
python3 tools/capture_training_images.py
```

Create the capture script:

```python
# tools/capture_training_images.py
"""
Captures raw images from the RealSense camera for labelling.
Saves to ~/xiangqi_dataset/raw_captures/
Press SPACE to capture, Q to quit.
"""
import cv2
import os
import time

SAVE_DIR = os.path.expanduser('~/xiangqi_dataset/raw_captures')
os.makedirs(SAVE_DIR, exist_ok=True)

cap = cv2.VideoCapture(0)  # Adjust index if needed; or use RealSense SDK
cap.set(cv2.CAP_PROP_FRAME_WIDTH,  1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

count = 0
print(f"Saving images to: {SAVE_DIR}")
print("SPACE = capture | Q = quit")

while True:
    ret, frame = cap.read()
    if not ret:
        break
    
    display = frame.copy()
    cv2.putText(display, f"Captured: {count} | SPACE=capture Q=quit",
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    cv2.imshow('Capture', display)
    
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    elif key == ord(' '):
        fname = os.path.join(SAVE_DIR, f'lab_{int(time.time())}_{count:04d}.jpg')
        cv2.imwrite(fname, frame)
        print(f"  Saved: {fname}")
        count += 1

cap.release()
cv2.destroyAllWindows()
print(f"Done. Captured {count} images.")
```

### 3.3 What positions to capture

Aim for **60–100 images** covering the following configurations. Take about 5 images per configuration (with slight camera or piece position variations):

| Configuration | Images | Notes |
|---|---|---|
| Full starting position (all 32 pieces) | 10 | Standard game start |
| Mid-game positions (10–20 pieces remaining) | 15 | Simulate various game stages |
| End-game positions (few pieces left) | 10 | Pieces spread across board |
| Captured pieces in graveyard | 5 | Pieces outside board area |
| Single piece type only | 10 | One row of each piece type for class clarity |
| Under bright overhead light | 5 | Lab lights at maximum |
| Under dimmer / window light | 5 | Realistic variation |
| Slight board rotation (±3°) | 5 | Camera not perfectly centred |
| Human hand partially in frame | 5 | Simulates mid-move detection |
| **Total** | **~70** | |

---

## Part 4: Labelling Lab Images

### 4.1 Install labelling tool

Use **LabelImg** (offline, simple) or **Roboflow Annotate** (online, with team collaboration):

**Option A: LabelImg (local)**
```bash
pip install labelImg
labelImg ~/xiangqi_dataset/raw_captures
```

**Option B: Roboflow Annotate (recommended for teams)**
1. Go to [roboflow.com](https://roboflow.com) and create a free project
2. Upload the images from `~/xiangqi_dataset/raw_captures`
3. Use the annotation interface to draw bounding boxes
4. Export in **YOLOv8 format**

### 4.2 Labelling instructions

This is the most time-consuming step. Work systematically:

1. **Box around each piece**: Draw a tight bounding box around the visible area of each piece disc (not including shadow). The box should cover the full diameter of the piece.

2. **Choose the correct class**: Use the class list from the table in the overview. Red pieces = classes 0–6, Black pieces = classes 7–13.

3. **Edge cases**:
   - Piece partially out of frame → label only if at least 50% of the piece is visible
   - Overlapping pieces (rare in Xiangqi) → label each individually; the top piece gets a slightly smaller box
   - Pieces in the graveyard → label with the same class as normal

4. **Label quality check**: After every 20 images, open a few label files and verify that bounding boxes match the visual. A common mistake is mislabelling Red and Black pieces — double-check by looking at the piece characters (红=Red side: 帅仕相马车炮兵, 黑=Black side: 将士象马车炮卒).

5. **Expected labelling time**: ~1–2 minutes per image × 70 images ≈ **2–3 hours total**. Split this across the team.

### 4.3 Export and merge with Roboflow base dataset

```bash
# After Roboflow export (or LabelImg), you have:
# lab_dataset/
#   train/images/, train/labels/
#   valid/images/, valid/labels/
#   data.yaml

# Merge with the Roboflow base dataset
python3 tools/merge_datasets.py
```

```python
# tools/merge_datasets.py
"""
Merges the Roboflow base dataset and the lab-captured dataset into one.
"""
import shutil
from pathlib import Path

BASE   = Path('xiangqi-pieces-1')     # Downloaded Roboflow dataset
LAB    = Path('lab_dataset')           # Your annotated lab images
OUTPUT = Path(os.path.expanduser('~/xiangqi_dataset'))

for split in ['train', 'val']:
    for kind in ['images', 'labels']:
        out_dir = OUTPUT / split / kind
        out_dir.mkdir(parents=True, exist_ok=True)

        # Base dataset
        src_base = BASE / ('valid' if split == 'val' else split) / kind
        if src_base.exists():
            for f in src_base.iterdir():
                shutil.copy(f, out_dir / f.name)

        # Lab dataset
        src_lab = LAB / split / kind
        if src_lab.exists():
            for f in src_lab.iterdir():
                # Prefix lab files to avoid name collisions
                shutil.copy(f, out_dir / f'lab_{f.name}')

print("Dataset merged.")
for split in ['train', 'val']:
    n = len(list((OUTPUT / split / 'images').glob('*')))
    print(f"  {split}: {n} images")
```

### 4.4 Write the dataset YAML

```python
# tools/write_dataset_yaml.py
import yaml, os

dataset_yaml = {
    'path': os.path.expanduser('~/xiangqi_dataset'),
    'train': 'train/images',
    'val':   'val/images',
    'test':  'test/images',   # optional
    'nc': 14,
    'names': [
        'r_general', 'r_advisor', 'r_elephant', 'r_horse',
        'r_chariot', 'r_cannon', 'r_soldier',
        'b_general', 'b_advisor', 'b_elephant', 'b_horse',
        'b_chariot', 'b_cannon', 'b_soldier',
    ]
}

with open(os.path.expanduser('~/xiangqi_dataset/data.yaml'), 'w') as f:
    yaml.dump(dataset_yaml, f, default_flow_style=False)
    
print("data.yaml written.")
```

```bash
python3 tools/write_dataset_yaml.py
```

---

## Part 5: Training

### 5.1 Training command

```bash
cd ~/xiangqi_dataset

yolo detect train \
  model=yolov8n.pt \
  data=data.yaml \
  epochs=100 \
  imgsz=640 \
  batch=16 \
  patience=20 \
  name=xiangqi_v1 \
  project=runs/detect \
  augment=True \
  degrees=5.0 \
  translate=0.1 \
  scale=0.3 \
  flipud=0.0 \
  fliplr=0.5 \
  mosaic=1.0 \
  hsv_h=0.015 \
  hsv_s=0.7 \
  hsv_v=0.4 \
  save=True \
  save_period=10
```

### 5.2 Parameter explanation

| Parameter | Value | Reasoning |
|---|---|---|
| `model=yolov8n.pt` | YOLOv8n pretrained on COCO | Smallest and fastest YOLOv8 variant; fine-tuning from COCO weights is far better than training from scratch |
| `epochs=100` | 100 | Sufficient for fine-tuning; `patience=20` stops early if no improvement |
| `imgsz=640` | 640 | Standard YOLOv8 input resolution; matches the camera output well |
| `batch=16` | 16 | Reduce to 8 if GPU OOM; increase to 32 if GPU has ≥ 8 GB VRAM |
| `degrees=5.0` | ±5° rotation | Simulates small board tilt variations |
| `translate=0.1` | ±10% shift | Camera position variation |
| `scale=0.3` | ±30% zoom | Distance variation |
| `fliplr=0.5` | 50% horizontal flip | Valid for symmetric board |
| `flipud=0.0` | 0% vertical flip | Do NOT flip vertically (Red/Black sides would swap) |
| `hsv_h/s/v` | colour jitter | Handles different lighting conditions in the lab |
| `mosaic=1.0` | Always on | Mixes 4 images into one — very effective for small datasets |

### 5.3 Expected training time

| Hardware | Estimated time (100 epochs) |
|---|---|
| NVIDIA GTX 1080 / RTX 2060 | ~20–40 minutes |
| NVIDIA RTX 3080 / 4070 | ~10–20 minutes |
| CPU only (no GPU) | ~4–8 hours |

> For CPU training, reduce `epochs` to 50 and use `batch=8`.

### 5.4 Monitor training progress

Training produces live loss curves. In a separate terminal:

```bash
# If tensorboard is installed:
tensorboard --logdir ~/xiangqi_dataset/runs/detect/xiangqi_v1

# Or just tail the training output for key metrics:
# Look for: box_loss, cls_loss, dfl_loss, mAP50, mAP50-95
```

**Target metrics at the end of training:**
- `mAP50` (mean Average Precision at IoU=0.50) > **0.90** (ideally > 0.95)
- `mAP50-95` > **0.65**
- `Precision` > **0.88**
- `Recall` > **0.88**

If mAP50 is below 0.85 after 100 epochs, see Section 6 (Troubleshooting).

---

## Part 6: Validation and Testing

### 6.1 Evaluate on the validation set

```bash
yolo detect val \
  model=runs/detect/xiangqi_v1/weights/best.pt \
  data=~/xiangqi_dataset/data.yaml \
  split=val
```

This prints per-class precision/recall and confusion matrix. Pay attention to:
- Which classes have the lowest recall (often `r_soldier` and `b_soldier` — there are 10 soldiers but they look similar)
- Confusion between Red and Black pieces of the same type

### 6.2 Per-class inspection

```python
# tools/inspect_predictions.py
"""Run inference on validation images and display results for manual inspection."""
from ultralytics import YOLO
import cv2, glob, random

model = YOLO('runs/detect/xiangqi_v1/weights/best.pt')

val_images = glob.glob(os.path.expanduser('~/xiangqi_dataset/val/images/*.jpg'))
random.shuffle(val_images)

for img_path in val_images[:20]:  # Inspect 20 random val images
    results = model(img_path, conf=0.4)[0]
    annotated = results.plot()
    cv2.imshow('Validation Prediction', annotated)
    key = cv2.waitKey(0) & 0xFF
    if key == ord('q'):
        break
        
cv2.destroyAllWindows()
```

For each image, verify:
- [ ] All pieces on the board are detected (no missed pieces)
- [ ] No false positives on the board pattern / river zone labels
- [ ] Red and Black pieces of the same type are correctly distinguished

### 6.3 Live camera test (inside Docker)

```bash
# With the board set up and camera running:
ros2 run xiangqi_vision vision_node --ros-args \
  -p model_path:=runs/detect/xiangqi_v1/weights/best.pt \
  -p confidence_threshold:=0.45

# In a second terminal, view the debug image:
ros2 run rqt_image_view rqt_image_view /xiangqi/debug_image
```

Observe the bounding boxes overlaid on the live camera feed. Look for:
- All 32 pieces detected at the start position
- No phantom detections in the river zone or graveyard
- Boxes remain stable between frames (no flickering classes)

---

## Part 7: Export for Deployment

### 7.1 Export to ONNX (for CPU deployment without CUDA)

```bash
yolo export \
  model=runs/detect/xiangqi_v1/weights/best.pt \
  format=onnx \
  imgsz=640 \
  simplify=True \
  opset=12
```

Output: `runs/detect/xiangqi_v1/weights/best.onnx`

### 7.2 Test the ONNX model

```bash
yolo detect predict \
  model=runs/detect/xiangqi_v1/weights/best.onnx \
  source=~/xiangqi_dataset/val/images/ \
  conf=0.45 \
  save=True
```

Verify that ONNX predictions match PyTorch predictions.

### 7.3 Copy model into the workspace (bind-mounted volume)

Weights are loaded from a path on the **mounted workspace** (not necessarily inside the package install tree). Typical layout:

```bash
MODEL_SRC=runs/detect/xiangqi_v1/weights/best.pt
MODEL_DEST=/home/rosuser/workspace/models/

mkdir -p $MODEL_DEST
cp $MODEL_SRC $MODEL_DEST/xiangqi_kaggle_v1_best.pt

# Optional: ONNX for CPU-only inference elsewhere
cp runs/detect/xiangqi_v1/weights/best.onnx $MODEL_DEST/xiangqi_pieces.onnx
```

On the host this is `UR5e_Env/workspace/models/` (or your compose bind mount). The default `vision_node` / `vision_config.yaml` entry is `xiangqi_kaggle_v1_best.pt` in that folder.

### 7.4 Update vision_config.yaml

```yaml
# workspace/src/xiangqi_bringup/config/vision_config.yaml
vision_node:
  ros__parameters:
    model_path: "/home/rosuser/workspace/models/xiangqi_kaggle_v1_best.pt"
    calibration_file: "/home/rosuser/workspace/config/board_calibration.yaml"
    confidence_threshold: 0.45
    stability_frames: 8
    poll_rate_hz: 4.0
    camera_topic: "/camera/color/image_raw"
```

Rebuild or reinstall the `xiangqi_bringup` package after editing YAML so the install space picks up changes, or override parameters at launch (see root `README.md`).

---

## Part 8: Iterative Improvement

After first deployment, collect images of real game scenarios and retrain periodically.

### 8.1 Hard negative mining

The most common failure mode is false positives in the graveyard or river zone. To fix:
1. Record `/xiangqi/debug_image` during a test game (2–3 games)
2. Extract frames where a false positive occurred (visible in the debug image)
3. Add those frames to the dataset **without labelling the false positive** (an empty label file means "no pieces here")
4. Retrain from the existing `best.pt` weights (use `model=best.pt` to resume fine-tuning)

### 8.2 When to retrain

Retrain if:
- mAP50 on live validation drops below 0.88
- You change the lab lighting setup significantly
- You change the board mat (reprinted version)
- You change the piece set (new diameter or colour)

### 8.3 Version tracking

Save model versions with descriptive names:

```
workspace/models/
  xiangqi_pieces_v1_roboflow_only.pt     # Baseline: Roboflow data only
  xiangqi_pieces_v2_plus_lab.pt          # After adding lab captures
  xiangqi_pieces_v3_hard_negatives.pt    # After hard negative mining
```

Use `vision_config.yaml` to switch which model is loaded without code changes.

---

## Part 9: Troubleshooting

| Symptom | Likely Cause | Fix |
|---|---|---|
| mAP50 < 0.85 after 100 epochs | Dataset too small, wrong class mapping | Add more lab images; verify class IDs in labels |
| Red/Black pieces confused | Similar appearance under lab lighting | Add HSV colour segmentation as a secondary check (see `piece_detector.py`) |
| Pieces at board edges missed | Homography crops too tight | Adjust `warp_margin` in `board_detector.py` |
| Soldier pieces frequently missed | Too many similar pieces with slight occlusion | Add more images focusing on soldier rows; try augmenting with cutmix |
| High false positives in river text | Model memorised text patterns | Add empty-label images showing just the river zone |
| Very slow inference (< 5 FPS) | Large model on CPU | Switch to ONNX export; reduce `imgsz` to 416 |
| `RuntimeError: CUDA out of memory` | Batch too large | Reduce `batch=8` and/or `imgsz=416` |
| ArUco markers not detected | Marker printed too small or blurry | Reprint with sharper borders; try `DICT_5X5_50` which is more robust |

---

## Summary Checklist

- [ ] Docker environment set up with `ultralytics` installed
- [ ] Roboflow base dataset downloaded and class IDs verified/remapped
- [ ] Lab images captured (~70 images across varied configurations)
- [ ] All lab images labelled with bounding boxes (all 14 classes)
- [ ] Datasets merged and `data.yaml` written
- [ ] Training run completed (`epochs=100`, `mAP50 > 0.90`)
- [ ] Validation predictions manually inspected (no systematic errors)
- [ ] ONNX model exported (optional)
- [ ] Model copied to `workspace/models/` (or path referenced by `vision_config.yaml`)
- [ ] `vision_config.yaml` updated with correct `model_path` (and rebuild / `ros2 launch` uses new install)
- [ ] Live camera test passed (all 32 pieces detected at game start)
