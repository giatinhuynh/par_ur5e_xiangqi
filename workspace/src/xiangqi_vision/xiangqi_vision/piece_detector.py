"""
Piece detector: YOLOv8 inference on the warped board image.

Class IDs come from the loaded .pt (data.yaml names). v4 uses black-then-red
alphabetical order; v8 uses red-then-black piece-type order. Mapping to grid
values is built at load time from model.names — do not hard-code one layout.
"""

from __future__ import annotations

import logging
import re
from typing import Dict, List, Tuple, Union

import numpy as np

try:
    import torch
    TORCH_AVAILABLE = True
except ImportError:
    TORCH_AVAILABLE = False

try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False

_log = logging.getLogger(__name__)

BOARD_FILES = 9
BOARD_RANKS = 10

# Piece-type token (after colour prefix) -> BoardState piece_code magnitude
_PIECE_TYPE_CODE: Dict[str, int] = {
    'general': 1, 'king': 1, 'shuai': 1, 'jiang': 1,
    'advisor': 2, 'shi': 2, 'guard': 2,
    'elephant': 3, 'xiang': 3, 'bishop': 3,
    'horse': 4, 'ma': 4, 'knight': 4,
    'chariot': 5, 'ju': 5, 'rook': 5, 'che': 5,
    'cannon': 6, 'pao': 6,
    'soldier': 7, 'bing': 7, 'zu': 7, 'pawn': 7,
}

# Fallback when model.names is missing (v4-style alphabetical black then red)
_LEGACY_CLASS_MAP: Dict[int, Tuple[bool, int]] = {
    0: (False, 2), 1: (False, 6), 2: (False, 5), 3: (False, 3), 4: (False, 1),
    5: (False, 4), 6: (False, 7),
    7: (True, 2), 8: (True, 6), 9: (True, 5), 10: (True, 3), 11: (True, 1),
    12: (True, 4), 13: (True, 7),
}
_LEGACY_CLASS_NAMES: List[str] = [
    'black_advisor', 'black_cannon', 'black_chariot', 'black_elephant',
    'black_general', 'black_horse', 'black_soldier',
    'red_advisor', 'red_cannon', 'red_chariot', 'red_elephant',
    'red_general', 'red_horse', 'red_soldier',
]

from xiangqi_vision.board_layout import pixel_to_grid as _pixel_to_grid


class _nullctx:
    def __enter__(self):
        return self

    def __exit__(self, *_):
        pass


def _normalize_label(name: str) -> str:
    return re.sub(r'[^a-z0-9]+', '_', name.strip().lower()).strip('_')


def _label_to_red_code(name: str) -> Tuple[bool, int]:
    """Parse a YOLO class name into (is_red, piece_code)."""
    norm = _normalize_label(name)
    if not norm:
        raise ValueError(f'empty class name')

    is_red: bool | None = None
    if norm.startswith('red_') or norm.startswith('r_'):
        is_red = True
        piece_token = norm.split('_', 1)[1]
    elif norm.startswith('black_') or norm.startswith('b_'):
        is_red = False
        piece_token = norm.split('_', 1)[1]
    else:
        for prefix, red in (('red', True), ('black', False)):
            if norm.startswith(prefix) and len(norm) > len(prefix):
                is_red = red
                piece_token = norm[len(prefix):].lstrip('_')
                break
        else:
            raise ValueError(f'cannot infer colour from class name: {name!r}')

    code = _PIECE_TYPE_CODE.get(piece_token)
    if code is None:
        raise ValueError(f'unknown piece type in class name: {name!r}')
    return is_red, code


def build_class_map_from_names(
    names: Union[Dict[int, str], List[str], Dict[str, str]],
) -> Tuple[Dict[int, Tuple[bool, int]], List[str]]:
    """Build (class_id -> (is_red, code), ordered names) from Ultralytics names."""
    if isinstance(names, list):
        items = [(i, str(n)) for i, n in enumerate(names)]
    else:
        items = sorted((int(k), str(v)) for k, v in names.items())

    class_map: Dict[int, Tuple[bool, int]] = {}
    class_names: List[str] = []
    for cid, name in items:
        class_map[cid] = _label_to_red_code(name)
        class_names.append(name)
    return class_map, class_names


def _yolo_model_names(model: 'YOLO') -> Union[Dict[int, str], List[str], None]:
    names = getattr(model, 'names', None)
    if names:
        return names
    inner = getattr(getattr(model, 'model', None), 'names', None)
    return inner or None


def _trained_imgsz_from_model(model: 'YOLO') -> int | None:
    """Read imgsz from Ultralytics checkpoint train_args (e.g. v8 @ 1024, v4 @ 640)."""
    ckpt = getattr(model, 'ckpt', None)
    if not isinstance(ckpt, dict):
        return None
    ta = ckpt.get('train_args')
    if ta is None:
        return None
    imgsz = getattr(ta, 'imgsz', None)
    if imgsz is None and isinstance(ta, dict):
        imgsz = ta.get('imgsz')
    if imgsz is None:
        return None
    if isinstance(imgsz, (list, tuple)):
        return int(max(imgsz))
    return int(imgsz)


class Detection:
    __slots__ = (
        'class_id', 'class_name', 'is_red', 'piece_code',
        'file', 'rank', 'pixel_x', 'pixel_y', 'confidence',
    )

    def __init__(
        self,
        class_id: int,
        px: float,
        py: float,
        conf: float,
        *,
        class_map: Dict[int, Tuple[bool, int]],
        class_names: List[str],
    ):
        self.class_id = class_id
        self.class_name = (
            class_names[class_id] if 0 <= class_id < len(class_names) else 'unknown'
        )
        is_red, code = class_map.get(class_id, (True, 0))
        self.is_red = is_red
        self.piece_code = code
        self.pixel_x = px
        self.pixel_y = py
        self.confidence = conf
        self.file, self.rank = _pixel_to_grid(px, py)

    @property
    def grid_value(self) -> int:
        return self.piece_code if self.is_red else -self.piece_code

    @property
    def valid(self) -> bool:
        return self.file >= 0 and self.rank >= 0


class PieceDetector:
    """Runs YOLOv8 inference on the normalised board image and returns grid occupancy."""

    def __init__(
        self,
        model_path: str,
        confidence_threshold: float = 0.5,
        yolo_imgsz: int = 0,
    ):
        if not YOLO_AVAILABLE:
            raise ImportError('ultralytics package not installed')
        self._model = YOLO(model_path)
        self._conf_threshold = confidence_threshold
        if yolo_imgsz > 0:
            self._infer_imgsz = int(yolo_imgsz)
        else:
            self._infer_imgsz = _trained_imgsz_from_model(self._model) or 640
        raw_names = _yolo_model_names(self._model)
        if raw_names:
            self._class_map, self._class_names = build_class_map_from_names(raw_names)
            _log.info(
                'YOLO class layout from weights (%d classes), e.g. 0=%s; infer imgsz=%d',
                len(self._class_names),
                self._class_names[0] if self._class_names else '?',
                self._infer_imgsz,
            )
        else:
            self._class_map = dict(_LEGACY_CLASS_MAP)
            self._class_names = list(_LEGACY_CLASS_NAMES)
            _log.warning(
                'YOLO model has no names metadata; using legacy v4 class layout; infer imgsz=%d',
                self._infer_imgsz,
            )

    def detect(self, board_image: np.ndarray) -> Tuple[List[Detection], np.ndarray]:
        """
        Run inference on the normalised board image.

        Returns:
            (detections, annotated_image)
        """
        # Use training imgsz (e.g. v8x @ 1024). Older code capped at 640 and never upscaled,
        # which heavily hurts models trained at 1024. Scalar imgsz lets Ultralytics letterbox.
        ctx = torch.no_grad() if TORCH_AVAILABLE else _nullctx()
        with ctx:
            results = self._model.predict(
                board_image,
                conf=self._conf_threshold,
                verbose=False,
                imgsz=self._infer_imgsz,
            )

        detections: List[Detection] = []
        if results and results[0].boxes is not None:
            for box in results[0].boxes:
                cls_id = int(box.cls[0].item())
                conf = float(box.conf[0].item())
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
                det = Detection(
                    cls_id, cx, cy, conf,
                    class_map=self._class_map,
                    class_names=self._class_names,
                )
                if det.valid:
                    detections.append(det)

        annotated = results[0].plot() if results else board_image.copy()
        return detections, annotated

    def detections_to_grid(
        self, detections: List[Detection]
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Convert detections to (grid int8[90], cell_confidence float32[90]).

        When multiple detections fall on the same cell, the one with the highest
        confidence wins — both for the piece code and the confidence value.
        """
        grid = np.zeros(BOARD_FILES * BOARD_RANKS, dtype=np.int8)
        cell_conf = np.zeros(BOARD_FILES * BOARD_RANKS, dtype=np.float32)
        for det in detections:
            if det.valid:
                idx = det.rank * BOARD_FILES + det.file
                if det.confidence > cell_conf[idx]:
                    cell_conf[idx] = det.confidence
                    grid[idx] = det.grid_value
        return grid, cell_conf

    def mean_confidence(self, detections: List[Detection]) -> float:
        if not detections:
            return 0.0
        return float(np.mean([d.confidence for d in detections]))
