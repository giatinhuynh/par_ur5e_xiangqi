"""
Board detector: ArUco-based corner detection and homography computation.

ArUco marker IDs (printed at **sheet corners** on the mat; tools/generate_board_svg.py):
  ID 0 = top-left     (file 0, rank 9  -- black side)
  ID 1 = top-right    (file 8, rank 9)
  ID 2 = bottom-right (file 8, rank 0  -- red/robot side)
  ID 3 = bottom-left  (file 0, rank 0)

The grid is inset inside the marker quad - use board_geometry_*.yaml grid_spacing_mm and
good calibration; pixel_to_grid uses board_layout (4×A3 mat geometry, not uniform margins).
"""

import cv2
import numpy as np
import yaml
import os
from dataclasses import dataclass, field
from typing import Dict, Optional, Tuple

from xiangqi_vision.board_layout import (
    NORM_W,
    NORM_H,
    MARGIN,
    pixel_to_grid as layout_pixel_to_grid,
)


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
    # Joint positions recorded at Step 1 - used for deterministic joint-space homing.
    scan_joint_positions: Optional[list] = None
    scan_joint_names: Optional[list] = None
    # Raw TCP positions at the 4 calibration corners (base_link, metres):
    # order matches CALIBRATION_CORNERS: (0,0), (8,0), (8,9), (0,9).
    # When present, grid_to_world uses bilinear interpolation (more accurate than rigid transform).
    calibration_corners_base: Optional[list] = None
    # Joint positions recorded at board level for each corner (Step 2 teach-in).
    calibration_corners_joint_names: Optional[list] = None
    calibration_corners_joints: Optional[list] = None  # 4 lists of float
    # Joint configs at approach_height above each calibration corner (Step 3 teach-in).
    cell_approach_joint_names: Optional[list] = None
    cell_approach_joints: Optional[list] = None   # 4 lists of float
    # E-file (file=4) midpoint joint configs for two-patch bilinear interpolation.
    # Order: [e0 (rank=0), e9 (rank=9)]. Absent = fall back to 4-corner mode.
    calibration_midpoints_joint_names: Optional[list] = None
    calibration_midpoints_joints: Optional[list] = None  # 2 lists of float
    cell_approach_midpoints_joints: Optional[list] = None  # 2 lists of float
    # Rank-midpoint joint configs at files a, e, i for 4-patch (2×2) interpolation.
    # Order: [a_mid, e_mid, i_mid]. rank_mid_idx is the rank row that was taught (default 5).
    rank_mid_idx: int = 5
    calibration_rank_mid_joint_names: Optional[list] = None
    calibration_rank_mid_joints: Optional[list] = None  # 3 lists of float
    cell_approach_rank_mid_joints: Optional[list] = None  # 3 lists of float
    # Graveyard joint configs - one fixed centre position per zone.
    graveyard_joint_names: Optional[list] = None
    graveyard_red_y: Optional[float] = None          # reference y for zone detection
    graveyard_red_approach_joints: Optional[list] = None   # single list of joint values
    graveyard_red_grasp_joints: Optional[list] = None
    graveyard_black_y: Optional[float] = None
    graveyard_black_approach_joints: Optional[list] = None
    graveyard_black_grasp_joints: Optional[list] = None

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
        if self.scan_joint_positions is not None:
            data['scan_joint_positions'] = [float(v) for v in self.scan_joint_positions]
        if self.scan_joint_names is not None:
            data['scan_joint_names'] = list(self.scan_joint_names)
        if self.calibration_corners_base is not None:
            data['calibration_corners_base'] = [
                [float(v) for v in corner] for corner in self.calibration_corners_base
            ]
        if self.calibration_corners_joint_names is not None:
            data['calibration_corners_joint_names'] = list(self.calibration_corners_joint_names)
        if self.calibration_corners_joints is not None:
            data['calibration_corners_joints'] = [
                [float(v) for v in row] for row in self.calibration_corners_joints
            ]
        if self.cell_approach_joint_names is not None:
            data['cell_approach_joint_names'] = list(self.cell_approach_joint_names)
        if self.cell_approach_joints is not None:
            data['cell_approach_joints'] = [
                [float(v) for v in row] for row in self.cell_approach_joints
            ]
        if self.calibration_midpoints_joint_names is not None:
            data['calibration_midpoints_joint_names'] = list(self.calibration_midpoints_joint_names)
        if self.calibration_midpoints_joints is not None:
            data['calibration_midpoints_joints'] = [
                [float(v) for v in row] for row in self.calibration_midpoints_joints
            ]
        if self.cell_approach_midpoints_joints is not None:
            data['cell_approach_midpoints_joints'] = [
                [float(v) for v in row] for row in self.cell_approach_midpoints_joints
            ]
        data['rank_mid_idx'] = int(self.rank_mid_idx)
        if self.calibration_rank_mid_joint_names is not None:
            data['calibration_rank_mid_joint_names'] = list(self.calibration_rank_mid_joint_names)
        if self.calibration_rank_mid_joints is not None:
            data['calibration_rank_mid_joints'] = [
                [float(v) for v in row] for row in self.calibration_rank_mid_joints
            ]
        if self.cell_approach_rank_mid_joints is not None:
            data['cell_approach_rank_mid_joints'] = [
                [float(v) for v in row] for row in self.cell_approach_rank_mid_joints
            ]
        if self.graveyard_joint_names is not None:
            data['graveyard_joint_names'] = list(self.graveyard_joint_names)
        for side in ('red', 'black'):
            y_val = getattr(self, f'graveyard_{side}_y')
            if y_val is not None:
                data[f'graveyard_{side}_y'] = float(y_val)
            for jtype in ('approach', 'grasp'):
                key = f'graveyard_{side}_{jtype}_joints'
                val = getattr(self, key)
                if val is not None:
                    data[key] = [float(v) for v in val]
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
        if isinstance(data.get('scan_joint_positions'), list):
            cal.scan_joint_positions = [float(v) for v in data['scan_joint_positions']]
        if isinstance(data.get('scan_joint_names'), list):
            cal.scan_joint_names = list(data['scan_joint_names'])
        if isinstance(data.get('calibration_corners_base'), list):
            cal.calibration_corners_base = [
                [float(v) for v in corner] for corner in data['calibration_corners_base']
            ]
        if isinstance(data.get('calibration_corners_joint_names'), list):
            cal.calibration_corners_joint_names = list(data['calibration_corners_joint_names'])
        if isinstance(data.get('calibration_corners_joints'), list):
            cal.calibration_corners_joints = [
                [float(v) for v in row] for row in data['calibration_corners_joints']
            ]
        if isinstance(data.get('cell_approach_joint_names'), list):
            cal.cell_approach_joint_names = list(data['cell_approach_joint_names'])
        if isinstance(data.get('cell_approach_joints'), list):
            cal.cell_approach_joints = [
                [float(v) for v in row] for row in data['cell_approach_joints']
            ]
        if isinstance(data.get('calibration_midpoints_joint_names'), list):
            cal.calibration_midpoints_joint_names = list(data['calibration_midpoints_joint_names'])
        if isinstance(data.get('calibration_midpoints_joints'), list):
            cal.calibration_midpoints_joints = [
                [float(v) for v in row] for row in data['calibration_midpoints_joints']
            ]
        if isinstance(data.get('cell_approach_midpoints_joints'), list):
            cal.cell_approach_midpoints_joints = [
                [float(v) for v in row] for row in data['cell_approach_midpoints_joints']
            ]
        cal.rank_mid_idx = int(data.get('rank_mid_idx', 5))
        if isinstance(data.get('calibration_rank_mid_joint_names'), list):
            cal.calibration_rank_mid_joint_names = list(data['calibration_rank_mid_joint_names'])
        if isinstance(data.get('calibration_rank_mid_joints'), list):
            cal.calibration_rank_mid_joints = [
                [float(v) for v in row] for row in data['calibration_rank_mid_joints']
            ]
        if isinstance(data.get('cell_approach_rank_mid_joints'), list):
            cal.cell_approach_rank_mid_joints = [
                [float(v) for v in row] for row in data['cell_approach_rank_mid_joints']
            ]
        if isinstance(data.get('graveyard_joint_names'), list):
            cal.graveyard_joint_names = list(data['graveyard_joint_names'])
        for side in ('red', 'black'):
            y_key = f'graveyard_{side}_y'
            if y_key in data:
                setattr(cal, y_key, float(data[y_key]))
            for jtype in ('approach', 'grasp'):
                key = f'graveyard_{side}_{jtype}_joints'
                val = data.get(key)
                if isinstance(val, list) and val:
                    # Accept both flat list and legacy nested list
                    if isinstance(val[0], list):
                        val = val[0]
                    setattr(cal, key, [float(v) for v in val])
        return cal


class BoardDetector:
    """Detects the Xiangqi board position using ArUco markers and computes homography."""

    ARUCO_DICT = cv2.aruco.DICT_4X4_50
    REQUIRED_MARKER_IDS = (0, 1, 2, 3)

    def __init__(self, calibration: Optional[BoardCalibration] = None):
        self.calibration = calibration or BoardCalibration()
        aruco_dict = cv2.aruco.getPredefinedDictionary(self.ARUCO_DICT)
        self._detector_params = cv2.aruco.DetectorParameters()
        # Lab mats: markers can be small in frame, glare on white print - relax defaults.
        self._detector_params.minMarkerPerimeterRate = 0.015
        self._detector_params.maxMarkerPerimeterRate = 4.0
        self._detector_params.adaptiveThreshWinSizeMin = 3
        self._detector_params.adaptiveThreshWinSizeMax = 23
        self._detector_params.adaptiveThreshWinSizeStep = 4
        self._detector_params.minCornerDistanceRate = 0.04
        self._detector_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self._aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, self._detector_params)
        self._last_detect_diag = ''

        # Destination quad for ArUco sheet corners (800×890). Grid intersections
        # are inset on the mat - pixel_to_grid uses board_layout anchors.
        self._norm_w = NORM_W
        self._norm_h = NORM_H
        self._margin = MARGIN
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
                hint += ' None detected - check print scale 100%, DICT_4X4_50, no glare.'
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

    def detect_full(
        self, image: np.ndarray
    ) -> Tuple[bool, Optional[np.ndarray], np.ndarray, Optional[list], Optional[np.ndarray]]:
        """Like detect() but also returns raw corners and ids for 3D pose estimation.

        Returns:
            (success, homography_3x3, debug_image, raw_corners, raw_ids)
        raw_corners / raw_ids are the direct outputs of detectMarkers(); None on failure.
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
                hint += ' None detected - check print scale 100%, DICT_4X4_50, no glare.'
            self._draw_status(debug, 'BOARD NOT FOUND', self._last_detect_diag + '\n' + hint, False)
            if corners is not None and ids is not None:
                cv2.aruco.drawDetectedMarkers(debug, corners, ids)
            return False, None, debug, None, None

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
            return False, None, debug, None, None

        self._draw_status(debug, 'BOARD DETECTED', '', True)
        return True, H, debug, corners, ids

    def estimate_corner_positions_3d(
        self,
        corners_raw: list,
        ids_raw: np.ndarray,
        camera_matrix: np.ndarray,
        dist_coeffs: np.ndarray,
        marker_length_m: float = 0.038,
    ) -> Optional[dict]:
        """Estimate 3D centre positions of the 4 ArUco corner markers in camera frame.

        Uses cv2.solvePnP with IPPE_SQUARE for each marker independently.
        Returns {id: np.ndarray([x, y, z])} for IDs 0–3, or None on failure.
        """
        if corners_raw is None or ids_raw is None:
            return None

        half = marker_length_m / 2.0
        obj_pts = np.array([
            [-half,  half, 0.0],
            [ half,  half, 0.0],
            [ half, -half, 0.0],
            [-half, -half, 0.0],
        ], dtype=np.float32)

        result = {}
        for i, marker_id in enumerate(ids_raw.flatten()):
            mid = int(marker_id)
            if mid not in self.REQUIRED_MARKER_IDS:
                continue
            img_pts = corners_raw[i][0].astype(np.float32)
            try:
                ok, rvec, tvec = cv2.solvePnP(
                    obj_pts, img_pts, camera_matrix, dist_coeffs,
                    flags=cv2.SOLVEPNP_IPPE_SQUARE,
                )
            except cv2.error:
                ok, rvec, tvec = cv2.solvePnP(
                    obj_pts, img_pts, camera_matrix, dist_coeffs,
                )
            if ok:
                result[mid] = tvec.flatten().astype(float)

        if not all(mid in result for mid in self.REQUIRED_MARKER_IDS):
            return None
        return result

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
        return layout_pixel_to_grid(float(warped[0]), float(warped[1]))

    def grid_to_world(self, file_idx: int, rank_idx: int) -> np.ndarray:
        """
        Convert grid coordinates to robot world-frame position (metres).

        Uses bilinear interpolation from the 4 measured corner TCP positions when available
        (calibration_corners_base present in YAML). This is more accurate than the rigid-body
        transform because it passes through all 4 measured corners exactly - no residual error.

        Falls back to board_to_base_tf rigid transform when corners not stored (old calibrations).
        """
        cal = self.calibration
        if cal.calibration_corners_base is not None and len(cal.calibration_corners_base) == 4:
            # Bilinear interpolation: corners in order (0,0),(8,0),(8,9),(0,9)
            C00, C80, C89, C09 = [np.array(c) for c in cal.calibration_corners_base]
            u = file_idx / 8.0
            v = rank_idx / 9.0
            return (1 - u) * (1 - v) * C00 + u * (1 - v) * C80 + u * v * C89 + (1 - u) * v * C09

        # Fallback: rigid-body transform (old calibration without corner data)
        if cal.board_to_base_tf is None:
            raise RuntimeError("board_to_base_tf not set -- run calibration first")
        spacing_m = cal.grid_spacing_mm / 1000.0
        ox_m = cal.board_origin_mm[0] / 1000.0
        oy_m = cal.board_origin_mm[1] / 1000.0
        board_pos = np.array([ox_m + file_idx * spacing_m, oy_m + rank_idx * spacing_m, 0.0, 1.0])
        return (cal.board_to_base_tf @ board_pos)[:3]
