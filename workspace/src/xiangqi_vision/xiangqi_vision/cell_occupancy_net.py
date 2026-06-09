"""
ResNet-34 binary occupancy classifier for Xiangqi board cells.

Based on OpenChessRobot (Frontiers in Robotics and AI, 2025):
  - ResNet-34 pretrained on ImageNet, FC replaced with 2-class head (empty / occupied)
  - All 90 cell crops processed in a single batch forward pass (~5–15 ms CPU)
  - Colour (red vs black) uses H+S hue check on occupied cells only — keeps the
    two concerns separate: ResNet answers "is there a piece?", hue answers "which
    colour?".  YOLO continues to provide piece type.

Falls back to a caller-supplied classical CV detector when the model file is not
present (e.g. before first training run).

Data collection:
  CellDataCollector.collect(warped_bgr, fen) saves labeled crops (empty /
  occupied) during live games when YOLO confidence is high enough to be a
  reliable label source.  Aim for ~300+ images before training.

Training:
  python3 scripts/train_occupancy.py --dataset <dir> --output occupancy.pth
"""

from __future__ import annotations

import os
import time
import warnings

import cv2
import numpy as np

from .board_layout import (
    norm_pixel_at_intersection as _blay_intersection,
    _cell_pixel_spacing as _blay_cell_spacing,
)
from .fen_util import fen_to_grid

try:
    import torch
    import torch.nn as nn
    TORCH_AVAILABLE = True
except ImportError:
    TORCH_AVAILABLE = False

try:
    import torchvision.models as _tvm
    import torchvision.transforms as _T
    TORCHVISION_AVAILABLE = True
except ImportError:
    TORCHVISION_AVAILABLE = False

CELL_SIZE = 64  # px — ResNet-34 input after resize


# ---------------------------------------------------------------------------
# Shared NMS utility
# ---------------------------------------------------------------------------

def apply_occupancy_nms(occ: np.ndarray, scores: np.ndarray) -> np.ndarray:
    """Return copy of occ[90] with weaker of two adjacent occupied cells zeroed.

    Prevents a single piece sitting slightly off-centre from claiming two grid
    intersections.  scores[idx] is the signal strength (higher = keep).
    """
    suppress = np.zeros(90, dtype=bool)
    for r in range(10):
        for f in range(9):
            idx = r * 9 + f
            if occ[idx] == 0 or suppress[idx]:
                continue
            if f + 1 < 9:
                nidx = r * 9 + (f + 1)
                if occ[nidx] != 0:
                    if scores[idx] >= scores[nidx]:
                        suppress[nidx] = True
                    else:
                        suppress[idx] = True
            if not suppress[idx] and r + 1 < 10:
                nidx = (r + 1) * 9 + f
                if occ[nidx] != 0:
                    if scores[idx] >= scores[nidx]:
                        suppress[nidx] = True
                    else:
                        suppress[idx] = True
    result = occ.copy()
    result[suppress] = 0
    return result


# ---------------------------------------------------------------------------
# ResNet-34 occupancy detector
# ---------------------------------------------------------------------------

class ResNetOccupancyDetector:
    """ResNet-34 binary occupancy detector (1=occupied, 0=empty).

    Returns binary occupancy only — colour and piece type are handled by YOLO.
    Inference time: ~5–15 ms on CPU for a batch of 90 cell crops.
    """

    def __init__(
        self,
        model_path: str | None = None,
        device: str = 'cpu',
        roi_fraction: float = 1.00,
        cell_px: int = 0,           # if >0: half = int(cell_px * roi_fraction / 2), else use board spacing
        x_squeeze_px: int = 0,      # shift files a-d right and i-f left, linearly from 0 at e
        y_squeeze_px: int = 0,      # shift ranks 0/9 inward toward center (rank 4.5), linearly
        threshold: float = 0.5,
    ):
        self._net: 'nn.Module | None' = None
        self._device_str = device
        self._transform = None
        self._threshold = threshold

        if cell_px > 0:
            self._half = max(8, int(cell_px * roi_fraction / 2))
        else:
            spacing = _blay_cell_spacing()
            self._half = max(8, int(spacing * roi_fraction))

        raw_centers = [
            (int(round(cx)), int(round(cy)))
            for r in range(10)
            for f in range(9)
            for cx, cy in [_blay_intersection(f, r)]
        ]
        centers = raw_centers
        if x_squeeze_px:
            centers = [
                (cx + int(round(x_squeeze_px * (4 - (idx % 9)) / 4)), cy)
                for idx, (cx, cy) in enumerate(centers)
            ]
        if y_squeeze_px:
            centers = [
                (cx, cy + int(round(y_squeeze_px * (4.5 - (idx // 9)) / 4.5)))
                for idx, (cx, cy) in enumerate(centers)
            ]
        self._centers = centers

        self._last_probs: np.ndarray = np.zeros(90, dtype=np.float32)

        if model_path:
            self._try_load(model_path, device)

    # ------------------------------------------------------------------
    def _try_load(self, model_path: str, device: str) -> None:
        if not TORCHVISION_AVAILABLE:
            warnings.warn(
                'torchvision not installed — ResNet occupancy model cannot be loaded; '
                'falling back to classical CV.  Install with: pip install torchvision'
            )
            return
        if not os.path.exists(model_path):
            return
        try:
            net = _tvm.resnet34(weights=None)
            net.fc = nn.Linear(512, 2)
            state = torch.load(model_path, map_location=device)
            net.load_state_dict(state)
            net.eval()
            dev = torch.device(device)
            net.to(dev)
            self._net = net
            self._device_str = device
            self._transform = _T.Compose([
                _T.ToPILImage(),
                _T.Resize((CELL_SIZE, CELL_SIZE)),
                _T.ToTensor(),
                _T.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225]),
            ])
        except Exception as exc:
            warnings.warn(f'ResNet occupancy model failed to load ({exc}); occupancy will return all zeros')

    @property
    def model_loaded(self) -> bool:
        return self._net is not None

    # ------------------------------------------------------------------
    def detect(self, warped_bgr: np.ndarray) -> np.ndarray:
        """Return occ_grid int8[90]: 1=occupied, 0=empty."""
        if self._net is None:
            return np.zeros(90, dtype=np.int8)
        return self._detect_resnet(warped_bgr)

    # ------------------------------------------------------------------
    def _crop_cells(self, warped_bgr: np.ndarray) -> list:
        """Return list of 90 RGB uint8 arrays (cell crops)."""
        half = self._half
        img_h, img_w = warped_bgr.shape[:2]
        crops = []
        for cx, cy in self._centers:
            y1, y2 = max(0, cy - half), min(img_h, cy + half)
            x1, x2 = max(0, cx - half), min(img_w, cx + half)
            crop = warped_bgr[y1:y2, x1:x2]
            if crop.size == 0:
                crop = np.zeros((2 * half, 2 * half, 3), dtype=np.uint8)
            crops.append(cv2.cvtColor(crop, cv2.COLOR_BGR2RGB))
        return crops

    def _detect_resnet(self, warped_bgr: np.ndarray) -> np.ndarray:
        """Return binary occ_grid int8[90]: 1=occupied, 0=empty."""
        crops = self._crop_cells(warped_bgr)

        tensors = [self._transform(c) for c in crops]
        batch = torch.stack(tensors).to(torch.device(self._device_str))
        with torch.no_grad():
            logits = self._net(batch)                                  # [90, 2]
            probs  = torch.softmax(logits, dim=1)[:, 1].cpu().numpy() # P(occupied)

        self._last_probs = probs  # stored for debug overlay
        return (probs >= self._threshold).astype(np.int8)


# ---------------------------------------------------------------------------
# Training data collector
# ---------------------------------------------------------------------------

class CellDataCollector:
    """Saves labeled cell crops (empty / occupied) to a dataset directory.

    Call collect() whenever YOLO has high enough confidence to be a reliable
    label source.  Crops are saved as JPEG under:
        <output_dir>/empty/
        <output_dir>/occupied/

    A simple subdirectory-per-class layout is directly compatible with
    torchvision.datasets.ImageFolder used in the training script.
    """

    def __init__(self, output_dir: str, roi_fraction: float = 1.00, cell_px: int = 85):
        self._empty_dir    = os.path.join(output_dir, 'empty')
        self._occupied_dir = os.path.join(output_dir, 'occupied')
        os.makedirs(self._empty_dir,    exist_ok=True)
        os.makedirs(self._occupied_dir, exist_ok=True)

        if cell_px > 0:
            self._half = max(8, int(cell_px * roi_fraction / 2))
        else:
            self._half = max(8, int(_blay_cell_spacing() * roi_fraction))
        self._centers: list = [
            (int(round(cx)), int(round(cy)))
            for r in range(10)
            for f in range(9)
            for cx, cy in [_blay_intersection(f, r)]
        ]
        self._total = 0

    def collect(self, warped_bgr: np.ndarray, fen: str) -> int:
        """Crop all 90 cells and save with labels derived from FEN.

        Returns the number of image files written.
        """
        grid = fen_to_grid(fen)
        half  = self._half
        img_h, img_w = warped_bgr.shape[:2]
        ts = int(time.time() * 1000)
        saved = 0

        for idx, (cx, cy) in enumerate(self._centers):
            y1, y2 = max(0, cy - half), min(img_h, cy + half)
            x1, x2 = max(0, cx - half), min(img_w, cx + half)
            crop = warped_bgr[y1:y2, x1:x2]
            if crop.shape[0] < 8 or crop.shape[1] < 8:
                continue
            label_dir = self._occupied_dir if grid[idx] != 0 else self._empty_dir
            path = os.path.join(label_dir, f'cell_{ts}_{idx:02d}.jpg')
            cv2.imwrite(path, crop)
            saved += 1

        self._total += saved
        return saved

    @property
    def total_saved(self) -> int:
        return self._total
