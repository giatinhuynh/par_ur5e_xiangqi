"""
move_translator.py: Converts Xiangqi algebraic moves to robot world-frame poses.

Xiangqi coordinate notation (coordinate-style):
  - Source: file letter + rank digit, e.g. 'h0' = file h (index 7), rank 0 (red home row)
  - Move: 4 chars, e.g. 'h0g2' = from h0 to g2
  - Files: a(0) b(1) c(2) d(3) e(4) f(5) g(6) h(7) i(8)
  - Ranks: 0 (red/robot side) to 9 (black/human side)
"""

from __future__ import annotations
import numpy as np
from geometry_msgs.msg import Pose, Point, Quaternion
from typing import Tuple, Optional


class BoardCalibration:
    """Lightweight local reference to calibration data needed by MoveTranslator.
    The full implementation lives in xiangqi_vision.board_detector.BoardCalibration;
    this mirrors only what the manipulation layer needs."""
    def __init__(self):
        self.homography = None
        self.board_to_base_tf = None
        self.grid_spacing_mm: float = 45.0
        self.board_origin_mm = (0.0, 0.0)

    def grid_to_world(self, file_idx: int, rank_idx: int):
        import numpy as np
        if self.board_to_base_tf is None:
            raise RuntimeError('board_to_base_tf not set -- run calibration first')
        spacing_m = self.grid_spacing_mm / 1000.0
        ox_m = self.board_origin_mm[0] / 1000.0
        oy_m = self.board_origin_mm[1] / 1000.0
        board_pos = np.array([ox_m + file_idx * spacing_m, oy_m + rank_idx * spacing_m, 0.0, 1.0])
        world_pos = np.array(self.board_to_base_tf) @ board_pos
        return world_pos[:3]


# Height offsets (metres) for different motion phases
APPROACH_HEIGHT   = 0.12   # Above piece surface for approach
GRASP_HEIGHT      = 0.005  # Touch piece surface (slight positive for suction)
TRANSIT_HEIGHT    = 0.20   # Safe clearance height during transit
GRAVEYARD_OFFSET  = 0.10   # Z-offset above graveyard zone


# Graveyard zones: flat positions off the board where captured pieces go
# Defined in robot base_link frame (metres). Configured via launch parameters.
DEFAULT_RED_GRAVEYARD   = [(0.60 + i * 0.05, -0.30, 0.02) for i in range(16)]
DEFAULT_BLACK_GRAVEYARD = [(0.60 + i * 0.05,  0.30, 0.02) for i in range(16)]


def _file_index(file_char: str) -> int:
    return ord(file_char.lower()) - ord('a')


def _parse_move(move: str) -> Tuple[int, int, int, int]:
    """Return (from_file, from_rank, to_file, to_rank) from a 4-char coordinate move."""
    return (
        _file_index(move[0]), int(move[1]),
        _file_index(move[2]), int(move[3]),
    )


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

    def move_to_poses(
        self,
        move: str,
        approach_height: float = APPROACH_HEIGHT,
        grasp_height: float = GRASP_HEIGHT,
        transit_height: float = TRANSIT_HEIGHT,
    ) -> Tuple[Pose, Pose, Pose, Pose, Pose]:
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
