"""
Piece detector: YOLOv8n inference on the warped board image.

Class mapping (14 piece classes + background):
  0  red_general    1  red_advisor    2  red_elephant   3  red_horse
  4  red_chariot    5  red_cannon     6  red_soldier
  7  black_general  8  black_advisor  9  black_elephant 10 black_horse
  11 black_chariot  12 black_cannon   13 black_soldier

Piece code mapping (for BoardState.grid):
  red:   general=1, advisor=2, elephant=3, horse=4, chariot=5, cannon=6, soldier=7
  black: same values but negative
"""

from __future__ import annotations
import numpy as np
from typing import List, Tuple, Optional
import cv2

try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False

BOARD_FILES = 9
BOARD_RANKS = 10

# (class_id -> (is_red, piece_code))
CLASS_MAP = {
    0:  (True,  1),   # red general
    1:  (True,  2),   # red advisor
    2:  (True,  3),   # red elephant
    3:  (True,  4),   # red horse
    4:  (True,  5),   # red chariot
    5:  (True,  6),   # red cannon
    6:  (True,  7),   # red soldier
    7:  (False, 1),   # black general
    8:  (False, 2),   # black advisor
    9:  (False, 3),   # black elephant
    10: (False, 4),   # black horse
    11: (False, 5),   # black chariot
    12: (False, 6),   # black cannon
    13: (False, 7),   # black soldier
}

CLASS_NAMES = [
    'red_general', 'red_advisor', 'red_elephant', 'red_horse',
    'red_chariot', 'red_cannon', 'red_soldier',
    'black_general', 'black_advisor', 'black_elephant', 'black_horse',
    'black_chariot', 'black_cannon', 'black_soldier',
]

from xiangqi_vision.board_layout import NORM_W, NORM_H, MARGIN, pixel_to_grid as _pixel_to_grid


class Detection:
    __slots__ = ('class_id', 'class_name', 'is_red', 'piece_code',
                 'file', 'rank', 'pixel_x', 'pixel_y', 'confidence')

    def __init__(self, class_id, px, py, conf):
        self.class_id = class_id
        self.class_name = CLASS_NAMES[class_id] if class_id < len(CLASS_NAMES) else 'unknown'
        is_red, code = CLASS_MAP.get(class_id, (True, 0))
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
    """Runs YOLOv8n inference on the normalised board image and returns grid occupancy."""

    def __init__(self, model_path: str, confidence_threshold: float = 0.5):
        if not YOLO_AVAILABLE:
            raise ImportError("ultralytics package not installed")
        self._model = YOLO(model_path)
        self._conf_threshold = confidence_threshold

    def detect(self, board_image: np.ndarray) -> Tuple[List[Detection], np.ndarray]:
        """
        Run inference on the normalised board image.

        Returns:
            (detections, annotated_image)
        """
        results = self._model.predict(
            board_image,
            conf=self._conf_threshold,
            verbose=False,
            imgsz=board_image.shape[:2],
        )

        detections: List[Detection] = []
        if results and results[0].boxes is not None:
            for box in results[0].boxes:
                cls_id = int(box.cls[0].item())
                conf = float(box.conf[0].item())
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
                det = Detection(cls_id, cx, cy, conf)
                if det.valid:
                    detections.append(det)

        annotated = results[0].plot() if results else board_image.copy()
        return detections, annotated

    def detections_to_grid(self, detections: List[Detection]) -> np.ndarray:
        """Convert a list of detections to a flat int8[90] grid array."""
        grid = np.zeros(BOARD_FILES * BOARD_RANKS, dtype=np.int8)
        for det in detections:
            if det.valid:
                idx = det.rank * BOARD_FILES + det.file
                grid[idx] = det.grid_value
        return grid

    def mean_confidence(self, detections: List[Detection]) -> float:
        if not detections:
            return 0.0
        return float(np.mean([d.confidence for d in detections]))
