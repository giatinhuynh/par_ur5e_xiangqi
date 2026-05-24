"""
move_translator.py: Converts Xiangqi algebraic moves to robot world-frame poses.

Xiangqi coordinate notation (coordinate-style):
  - Source: file letter + rank digit, e.g. 'h0' = file h (index 7), rank 0 (red home row)
  - Move: 4 chars, e.g. 'h0g2' = from h0 to g2
  - Files: a(0) b(1) c(2) d(3) e(4) f(5) g(6) h(7) i(8)
  - Ranks: 0 (red/robot side) to 9 (black/human side)
"""

from __future__ import annotations
from typing import Dict, Optional, Tuple

import numpy as np
from geometry_msgs.msg import Pose, Point, Quaternion
from dataclasses import dataclass
import yaml


@dataclass
class BoardCalibration:
    """Calibration data for mapping grid indices to base_link (same YAML as vision)."""

    homography: Optional[object] = None
    board_to_base_tf: Optional[np.ndarray] = None
    grid_spacing_mm: float = 61.25
    board_origin_mm: Tuple[float, float] = (0.0, 0.0)
    piece_diameter_mm: float = 20.0
    grasp_height_mm: float = 15.0
    approach_height_mm: float = 120.0
    transit_height_mm: float = 200.0
    scan_pose: Optional[Dict[str, float]] = None
    initial_pose: Optional[Dict[str, float]] = None
    scan_joint_positions: Optional[list] = None
    scan_joint_names: Optional[list] = None
    # Raw TCP positions at the 4 calibration corners (base_link, metres):
    # order: (0,0), (8,0), (8,9), (0,9). Used for bilinear grid_to_world interpolation.
    calibration_corners_base: Optional[list] = None
    # Joint positions recorded at board level for each corner (Step 2 teach-in).
    calibration_corners_joint_names: Optional[list] = None
    calibration_corners_joints: Optional[list] = None  # 4 lists of float
    # Joint configs at approach_height above each calibration corner (Step 3 teach-in).
    # Same corner order. Used for bilinear joint interpolation for board approach moves.
    cell_approach_joint_names: Optional[list] = None
    cell_approach_joints: Optional[list] = None   # 4 lists of float (one per corner)
    # Graveyard joint configs — one fixed centre position per zone.
    graveyard_joint_names: Optional[list] = None
    graveyard_red_y: Optional[float] = None          # reference y for zone detection
    graveyard_red_approach_joints: Optional[list] = None   # single list of joint values
    graveyard_red_grasp_joints: Optional[list] = None
    graveyard_black_y: Optional[float] = None
    graveyard_black_approach_joints: Optional[list] = None
    graveyard_black_grasp_joints: Optional[list] = None

    @classmethod
    def load(cls, path: str) -> 'BoardCalibration':
        """Load from the same `board_calibration.yaml` written by calibration_tool / vision."""
        with open(path, 'r') as f:
            data = yaml.safe_load(f) or {}
        cal = cls()
        if data.get('homography'):
            cal.homography = np.array(data['homography'])
        if data.get('board_to_base_tf'):
            cal.board_to_base_tf = np.array(data['board_to_base_tf'])
        cal.grid_spacing_mm = float(data.get('grid_spacing_mm', 61.25))
        bo = data.get('board_origin_mm', [0.0, 0.0])
        cal.board_origin_mm = (float(bo[0]), float(bo[1]))
        cal.piece_diameter_mm = float(data.get('piece_diameter_mm', 20.0))
        cal.grasp_height_mm = float(data.get('grasp_height_mm', cal.piece_diameter_mm / 2.0))
        cal.approach_height_mm = float(data.get('approach_height_mm', 120.0))
        cal.transit_height_mm = float(data.get('transit_height_mm', 200.0))
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
                    # Accept both flat list [j0, j1, ...] and legacy nested [[j0, j1, ...]]
                    if isinstance(val[0], list):
                        val = val[0]
                    setattr(cal, key, [float(v) for v in val])
        return cal

    def interpolate_approach_joints(
        self, file_f: float, rank_f: float
    ) -> Optional[Tuple[list, list]]:
        """Bilinear interpolation of joint angles for any board cell approach position.

        file_f / rank_f are continuous floats (0.0–8.0 / 0.0–9.0).
        Returns (joint_names, joint_positions) or None if approach joints not calibrated.
        """
        if (
            self.cell_approach_joints is None
            or len(self.cell_approach_joints) != 4
            or self.cell_approach_joint_names is None
        ):
            return None
        J00 = np.array(self.cell_approach_joints[0])
        J80 = np.array(self.cell_approach_joints[1])
        J89 = np.array(self.cell_approach_joints[2])
        J09 = np.array(self.cell_approach_joints[3])
        u = file_f / 8.0
        v = rank_f / 9.0
        joints = (1-u)*(1-v)*J00 + u*(1-v)*J80 + u*v*J89 + (1-u)*v*J09
        return self.cell_approach_joint_names, joints.tolist()

    def interpolate_board_joints(
        self, file_f: float, rank_f: float
    ) -> Optional[Tuple[list, list]]:
        """Bilinear interpolation of joint angles for any board cell grasp position.

        file_f / rank_f are continuous floats (0.0–8.0 / 0.0–9.0).
        Returns (joint_names, joint_positions) or None if board joints not calibrated.
        """
        if (
            self.calibration_corners_joints is None
            or len(self.calibration_corners_joints) != 4
            or self.calibration_corners_joint_names is None
        ):
            return None
        J00 = np.array(self.calibration_corners_joints[0])
        J80 = np.array(self.calibration_corners_joints[1])
        J89 = np.array(self.calibration_corners_joints[2])
        J09 = np.array(self.calibration_corners_joints[3])
        u = file_f / 8.0
        v = rank_f / 9.0
        joints = (1-u)*(1-v)*J00 + u*(1-v)*J80 + u*v*J89 + (1-u)*v*J09
        return self.calibration_corners_joint_names, joints.tolist()

    def get_graveyard_joints(
        self, is_red: bool
    ) -> Optional[Tuple[Tuple[list, list], Tuple[list, list]]]:
        """Return fixed graveyard joint configs for the given zone.

        Returns ((approach_names, approach_positions), (grasp_names, grasp_positions))
        or None if graveyard joints for this side are not calibrated.
        """
        if is_red:
            approach = self.graveyard_red_approach_joints
            grasp    = self.graveyard_red_grasp_joints
        else:
            approach = self.graveyard_black_approach_joints
            grasp    = self.graveyard_black_grasp_joints

        if (
            approach is None or grasp is None
            or self.graveyard_joint_names is None
        ):
            return None

        names = self.graveyard_joint_names
        return (names, approach), (names, grasp)

    def grid_to_world(self, file_idx: int, rank_idx: int) -> np.ndarray:
        """
        Convert grid coordinates to robot world-frame position (metres).

        Uses bilinear interpolation from the 4 measured corner TCP positions when available.
        This passes through all 4 corners exactly (zero error at corners, minimal error
        at interior points for a flat board). Falls back to rigid transform for old calibrations.
        """
        if self.calibration_corners_base is not None and len(self.calibration_corners_base) == 4:
            # Corners in order: (0,0), (8,0), (8,9), (0,9)
            C00, C80, C89, C09 = [np.array(c) for c in self.calibration_corners_base]
            u = file_idx / 8.0
            v = rank_idx / 9.0
            return (1 - u) * (1 - v) * C00 + u * (1 - v) * C80 + u * v * C89 + (1 - u) * v * C09

        # Fallback: rigid-body transform (calibrations without corner data)
        if self.board_to_base_tf is None:
            raise RuntimeError('board_to_base_tf not set -- run calibration first')
        spacing_m = self.grid_spacing_mm / 1000.0
        ox_m = self.board_origin_mm[0] / 1000.0
        oy_m = self.board_origin_mm[1] / 1000.0
        board_pos = np.array([ox_m + file_idx * spacing_m, oy_m + rank_idx * spacing_m, 0.0, 1.0])
        return (np.array(self.board_to_base_tf) @ board_pos)[:3]


# Default height offsets (metres); overridden per-calibration from board_calibration.yaml
APPROACH_HEIGHT   = 0.12   # Above board plane at intersection
GRASP_HEIGHT      = 0.010  # Side-grip at mid-piece (~10 mm for 20 mm disc)
TRANSIT_HEIGHT    = 0.20   # Clearance during transit
GRAVEYARD_OFFSET  = 0.10   # Z-offset above graveyard zone


# Graveyard zones: flat positions off the board where captured pieces go
# Defined in robot base_link frame (metres). Configured via launch parameters.
DEFAULT_RED_GRAVEYARD   = [(0.60 + i * 0.05, -0.30, 0.02) for i in range(16)]
DEFAULT_BLACK_GRAVEYARD = [(0.60 + i * 0.05,  0.30, 0.02) for i in range(16)]


def _file_index(file_char: str) -> int:
    return ord(file_char.lower()) - ord('a')


def _parse_move(move: str) -> Tuple[int, int, int, int]:
    """Return (from_file, from_rank, to_file, to_rank).

    Handles both 4-char (e.g. 'b3b9') and 5-char (e.g. 'b3b10') moves
    where the destination rank can be two digits.
    """
    from_file = _file_index(move[0])
    from_rank = int(move[1])
    to_file   = _file_index(move[2])
    to_rank   = int(move[3:])   # 1 or 2 digits
    return from_file, from_rank, to_file, to_rank


def _make_pose(x: float, y: float, z: float) -> Pose:
    """Create a Pose with fixed downward orientation (gripper pointing down)."""
    pose = Pose()
    pose.position = Point(x=x, y=y, z=z)
    # Quaternion for downward-pointing end-effector (Z-down, gripper facing down)
    pose.orientation = Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)
    return pose


class MoveTranslator:
    """Converts Xiangqi board coordinates to robot workspace Cartesian poses."""

    def __init__(self, calibration: BoardCalibration):
        self._cal = calibration
        self._red_graveyard = list(DEFAULT_RED_GRAVEYARD)
        self._black_graveyard = list(DEFAULT_BLACK_GRAVEYARD)
        self._red_graveyard_idx = 0
        self._black_graveyard_idx = 0

    def reset_graveyards(self) -> None:
        self._red_graveyard_idx = 0
        self._black_graveyard_idx = 0

    def _approach_height_m(self) -> float:
        return self._cal.approach_height_mm / 1000.0

    def _grasp_height_m(self) -> float:
        return self._cal.grasp_height_mm / 1000.0

    def _transit_height_m(self) -> float:
        return self._cal.transit_height_mm / 1000.0

    def move_to_poses(
        self,
        move: str,
        approach_height: float | None = None,
        grasp_height: float | None = None,
        transit_height: float | None = None,
    ) -> Tuple[Pose, Pose, Pose, Pose, Pose]:
        if approach_height is None:
            approach_height = self._approach_height_m()
        if grasp_height is None:
            grasp_height = self._grasp_height_m()
        if transit_height is None:
            transit_height = self._transit_height_m()
        """
        Convert a 4-char move to the 5 key waypoint poses:
          (approach_pick, grasp, lift, approach_place, place)
        """
        from_file, from_rank, to_file, to_rank = _parse_move(move)

        pick_xyz = self._cal.grid_to_world(from_file, from_rank)
        place_xyz = self._cal.grid_to_world(to_file, to_rank)

        approach_pick  = _make_pose(pick_xyz[0],  pick_xyz[1],  pick_xyz[2]  + approach_height)
        grasp_pose     = _make_pose(pick_xyz[0],  pick_xyz[1],  pick_xyz[2]  + grasp_height)
        lift_pose      = _make_pose(pick_xyz[0],  pick_xyz[1],  pick_xyz[2]  + transit_height)
        approach_place = _make_pose(place_xyz[0], place_xyz[1], place_xyz[2] + approach_height)
        place_pose     = _make_pose(place_xyz[0], place_xyz[1], place_xyz[2] + grasp_height)

        return approach_pick, grasp_pose, lift_pose, approach_place, place_pose

    def graveyard_pose(self, is_red_piece: bool) -> Pose:
        """Return the next available graveyard position for a captured piece."""
        if is_red_piece:
            slots = self._red_graveyard
            idx = self._red_graveyard_idx % len(slots)
            self._red_graveyard_idx += 1
        else:
            slots = self._black_graveyard
            idx = self._black_graveyard_idx % len(slots)
            self._black_graveyard_idx += 1

        x, y, z = slots[idx]
        return _make_pose(x, y, z + GRASP_HEIGHT)

    def square_to_world(self, file_idx: int, rank_idx: int) -> Pose:
        """Get the robot pose directly above a board square."""
        xyz = self._cal.grid_to_world(file_idx, rank_idx)
        return _make_pose(xyz[0], xyz[1], xyz[2] + APPROACH_HEIGHT)
