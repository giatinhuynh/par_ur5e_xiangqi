"""
Board detector: ArUco-based corner detection and homography computation.

ArUco marker IDs (printed at **sheet corners** on the mat; tools/generate_board_svg.py):
  ID 0 = top-left     (file 0, rank 9  -- black side)
  ID 1 = top-right    (file 8, rank 9)
  ID 2 = bottom-right (file 8, rank 0  -- red/robot side)
  ID 3 = bottom-left  (file 0, rank 0)

The grid is inset inside the marker quad — use board_geometry_*.yaml grid_spacing_mm and
good calibration; pixel_to_grid uses separate x/y spacing in the normalised image.
"""

import cv2
import numpy as np
import yaml
import os
from dataclasses import dataclass, field
from typing import Dict, Optional, Tuple


# Xiangqi board: 9 files (columns a-i) x 10 ranks (rows 0-9)
BOARD_FILES = 9
BOARD_RANKS = 10


@dataclass
class BoardCalibration:
    """Stores the computed calibration between camera image and board/robot frames."""
    homography: Optional[np.ndarray] = None          # 3x3 image -> normalised board frame
    board_to_base_tf: Optional[np.ndarray] = None    # 4x4 board frame -> robot base_link
    grid_spacing_mm: float = 61.25                   # 4×A3 mat default (docs/board_geometry_4xA3.yaml)
    board_origin_mm: Tuple[float, float] = (0.0, 0.0)  # Bottom-left corner in robot base XY
    marker_ids: list = field(default_factory=lambda: [0, 1, 2, 3])
    image_width: int = 640
    image_height: int = 480
    # Taught in calibration_tool Step 1 (arm at scan pose, SPACE): base_link TCP, metres / rad.
    scan_pose: Optional[Dict[str, float]] = None
    initial_pose: Optional[Dict[str, float]] = None

    @property
    def is_valid(self) -> bool:
        return self.homography is not None and self.board_to_base_tf is not None

    def save(self, path: str) -> None:
        data = {}
        if os.path.exists(path):
            with open(path, 'r') as f:
                data = yaml.safe_load(f) or {}
        data.update({
            'homography': self.homography.tolist() if self.homography is not None else None,
            'board_to_base_tf': self.board_to_base_tf.tolist() if self.board_to_base_tf is not None else None,
            'grid_spacing_mm': self.grid_spacing_mm,
            'board_origin_mm': list(self.board_origin_mm),
            'image_width': self.image_width,
            'image_height': self.image_height,
        })
        if self.scan_pose is not None:
            data['scan_pose'] = {k: float(v) for k, v in self.scan_pose.items()}
        if self.initial_pose is not None:
            data['initial_pose'] = {k: float(v) for k, v in self.initial_pose.items()}
        parent = os.path.dirname(path)
        if parent:
            os.makedirs(parent, exist_ok=True)
        with open(path, 'w') as f:
            yaml.dump(data, f, default_flow_style=False, sort_keys=False)

    @classmethod
    def load(cls, path: str) -> 'BoardCalibration':
        with open(path, 'r') as f:
            data = yaml.safe_load(f)
        cal = cls()
        if data.get('homography'):
            cal.homography = np.array(data['homography'])
        if data.get('board_to_base_tf'):
            cal.board_to_base_tf = np.array(data['board_to_base_tf'])
        cal.grid_spacing_mm = float(data.get('grid_spacing_mm', 61.25))
        cal.board_origin_mm = tuple(data.get('board_origin_mm', [0.0, 0.0]))
        cal.image_width = data.get('image_width', 640)
        cal.image_height = data.get('image_height', 480)
        if isinstance(data.get('scan_pose'), dict):
            cal.scan_pose = {k: float(data['scan_pose'][k]) for k in ('x', 'y', 'z', 'yaw') if k in data['scan_pose']}
        if isinstance(data.get('initial_pose'), dict):
            cal.initial_pose = {k: float(data['initial_pose'][k]) for k in ('x', 'y', 'z', 'yaw') if k in data['initial_pose']}
        return cal


class BoardDetector:
    """Detects the Xiangqi board position using ArUco markers and computes homography."""

    ARUCO_DICT = cv2.aruco.DICT_4X4_50
    REQUIRED_MARKER_IDS = (0, 1, 2, 3)

    def __init__(self, calibration: Optional[BoardCalibration] = None):
        self.calibration = calibration or BoardCalibration()
        aruco_dict = cv2.aruco.getPredefinedDictionary(self.ARUCO_DICT)
        self._detector_params = cv2.aruco.DetectorParameters()
        # Lab mats: markers can be small in frame, glare on white print — relax defaults.
        self._detector_params.minMarkerPerimeterRate = 0.015
        self._detector_params.maxMarkerPerimeterRate = 4.0
        self._detector_params.adaptiveThreshWinSizeMin = 3
        self._detector_params.adaptiveThreshWinSizeMax = 23
        self._detector_params.adaptiveThreshWinSizeStep = 4
        self._detector_params.minCornerDistanceRate = 0.04
        self._detector_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self._aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, self._detector_params)
        self._last_detect_diag = ''

        # Destination points in a normalised board image (800x890 px). Margins
        # place the warped 9×10 intersections on a uniform grid; file/rank use
        # separate spacing so rank steps match norm_h (square cells on the mat).
        self._norm_w = 800
        self._norm_h = 890
        self._margin = 44
        self._dst_corners = np.float32([
            [self._margin, self._norm_h - self._margin],          # ID 0: top-left  (rank 9)
            [self._norm_w - self._margin, self._norm_h - self._margin],  # ID 1: top-right
            [self._norm_w - self._margin, self._margin],          # ID 2: bottom-right (rank 0)
            [self._margin, self._margin],                         # ID 3: bottom-left
        ])

    def _preprocess(self, image: np.ndarray) -> np.ndarray:
        """Improve ArUco contrast under uneven lab lighting."""
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        enhanced = clahe.apply(gray)
        return cv2.cvtColor(enhanced, cv2.COLOR_GRAY2BGR)

    def _detect_markers(self, image: np.ndarray):
        """Run ArUco on raw and CLAHE images; keep the pass with most corner IDs 0–3."""
        best_corners, best_ids = None, None
        best_count = -1
        for frame in (image, self._preprocess(image)):
            corners, ids, _ = self._aruco_detector.detectMarkers(frame)
            if ids is None:
                continue
            n_required = sum(1 for mid in ids.flatten() if mid in self.REQUIRED_MARKER_IDS)
            if n_required > best_count:
                best_count = n_required
                best_corners, best_ids = corners, ids
            if best_count >= 4:
                break
        return best_corners, best_ids

    @staticmethod
    def _draw_status(debug: np.ndarray, title: str, detail: str, ok: bool) -> None:
        colour = (0, 200, 0) if ok else (0, 0, 255)
        cv2.putText(debug, title, (16, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, colour, 2, cv2.LINE_AA)
        if detail:
            y = 72
            for line in detail.split('\n'):
                cv2.putText(debug, line, (16, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, colour, 1, cv2.LINE_AA)
                y += 26

    def detect(self, image: np.ndarray) -> Tuple[bool, Optional[np.ndarray], np.ndarray]:
        """
        Detect ArUco markers and compute homography.

        Returns:
            (success, homography_3x3, debug_image)
        """
        debug = image.copy()
        corners, ids = self._detect_markers(image)

        all_ids = [] if ids is None else sorted(int(x) for x in ids.flatten())
        id_to_corner = {}
        if ids is not None:
            for i, marker_id in enumerate(ids.flatten()):
                if marker_id in self.REQUIRED_MARKER_IDS:
                    centre = corners[i][0].mean(axis=0)
                    id_to_corner[int(marker_id)] = centre

        missing = [mid for mid in self.REQUIRED_MARKER_IDS if mid not in id_to_corner]
        self._last_detect_diag = (
            f'markers in frame: {len(all_ids)}  ids: {all_ids}\n'
            f'corner IDs 0-3: {sorted(id_to_corner.keys())}  missing: {missing or "none"}'
        )

        if len(id_to_corner) < 4:
            hint = 'Need ALL four ArUco IDs 0,1,2,3 at mat outer corners (4xA3: tape full mat).'
            if missing:
                hint += f' Missing ID(s): {missing}.'
            if len(all_ids) == 0:
                hint += ' None detected — check print scale 100%, DICT_4X4_50, no glare.'
            self._draw_status(debug, 'BOARD NOT FOUND', self._last_detect_diag + '\n' + hint, False)
            if corners is not None and ids is not None:
                cv2.aruco.drawDetectedMarkers(debug, corners, ids)
            return False, None, debug

        cv2.aruco.drawDetectedMarkers(debug, corners, ids)

        src_pts = np.float32([
            id_to_corner[0],
            id_to_corner[1],
            id_to_corner[2],
            id_to_corner[3],
        ])

        H, _ = cv2.findHomography(src_pts, self._dst_corners, cv2.RANSAC, 5.0)
        if H is None:
            self._draw_status(debug, 'BOARD NOT FOUND', self._last_detect_diag + '\nhomography failed', False)
            return False, None, debug

        self._draw_status(debug, 'BOARD DETECTED', 'Press SPACE to capture', True)
        return True, H, debug

    def warp_board(self, image: np.ndarray, H: np.ndarray) -> np.ndarray:
        """Apply homography to get a top-down normalised board image."""
        return cv2.warpPerspective(image, H, (self._norm_w, self._norm_h))

    def pixel_to_grid(self, px: float, py: float, H: np.ndarray) -> Tuple[int, int]:
        """
        Map a raw image pixel to a board grid coordinate (file, rank).
        Returns (-1, -1) if outside board bounds.
        """
        pt = np.array([[[px, py]]], dtype=np.float32)
        warped = cv2.perspectiveTransform(pt, H)[0][0]

        spacing_x = (self._norm_w - 2 * self._margin) / (BOARD_FILES - 1)
        spacing_y = (self._norm_h - 2 * self._margin) / (BOARD_RANKS - 1)
        file_f = (warped[0] - self._margin) / spacing_x
        rank_f = (warped[1] - self._margin) / spacing_y

        file_idx = int(round(file_f))
        rank_idx = int(round(rank_f))

        if 0 <= file_idx < BOARD_FILES and 0 <= rank_idx < BOARD_RANKS:
            return file_idx, rank_idx
        return -1, -1

    def grid_to_world(self, file_idx: int, rank_idx: int) -> np.ndarray:
        """
        Convert grid coordinates to robot world-frame position (metres).
        Requires calibration.board_to_base_tf to be set.
        Returns a 3-element XYZ array in metres.
        """
        if self.calibration.board_to_base_tf is None:
            raise RuntimeError("board_to_base_tf not set -- run calibration first")

        spacing_m = self.calibration.grid_spacing_mm / 1000.0
        ox, oy = self.calibration.board_origin_mm
        ox_m, oy_m = ox / 1000.0, oy / 1000.0

        # Board frame: X = file direction, Y = rank direction, Z = up
        board_pos = np.array([
            ox_m + file_idx * spacing_m,
            oy_m + rank_idx * spacing_m,
            0.0,
            1.0,
        ])
        world_pos = self.calibration.board_to_base_tf @ board_pos
        return world_pos[:3]
