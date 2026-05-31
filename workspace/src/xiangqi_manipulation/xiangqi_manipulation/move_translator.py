"""
move_translator.py: Converts Xiangqi algebraic moves to robot world-frame poses.

Xiangqi coordinate notation (coordinate-style):
  - Source: file letter + rank digit, e.g. 'h0' = file h (index 7), rank 0 (red home row)
  - Move: 4 chars, e.g. 'h0g2' = from h0 to g2
  - Files: a(0) b(1) c(2) d(3) e(4) f(5) g(6) h(7) i(8)
  - Ranks: 0 (red/robot side) to 9 (black/human side)
"""

from __future__ import annotations
from typing import Dict, List, Optional, Tuple

import numpy as np
from geometry_msgs.msg import Pose, Point, Quaternion
from dataclasses import dataclass
import yaml


# ---------------------------------------------------------------------------
# 4×A3 mat geometry — mirrors board_layout._compute_4xa3_grid_mm() exactly
# ---------------------------------------------------------------------------

def _compute_4xa3_fractions() -> Tuple[float, float, float, float, float]:
    """Return (u_file0, u_file8, v_rank0, v_rank9, grid_spacing_m) for the 4×A3 Xiangqi mat.

    u / v are bilinear fractions within the ArUco marker-centre quadrilateral:
      u=0,v=0 → M3 centre (file0-side, rank0-side marker)
      u=1,v=0 → M2 centre (file8-side, rank0-side marker)
      u=1,v=1 → M1 centre (file8-side, rank9-side marker)
      u=0,v=1 → M0 centre (file0-side, rank9-side marker)

    Grid intersections a0/i0/a9/i9 are INSIDE the marker quad (the grid starts at the
    inner band boundary, 25 mm past each marker centre). This is NOT the same as the
    v values used by board_layout.norm_pixel_at_intersection which uses the inner-band
    origin as u=0,v=0 and is suitable only for pixel snapping, not 3D positioning.
    """
    page_h = 840.0
    page_w = 594.0
    aruco_page_inset = 8.0
    aruco_size = 38.0
    band = 6.0
    graveyard_w = 85.0
    graveyard_gap = 5.0

    marker_inset = aruco_page_inset + aruco_size / 2.0  # 27.0 mm from sheet corner to marker centre

    inner = aruco_page_inset + aruco_size + band          # 52.0 mm (inner band boundary)
    inner_top = inner
    inner_bottom = page_h - inner                          # 788.0 mm
    board_top = inner_top + graveyard_w + graveyard_gap   # 142.0 mm
    board_bottom = inner_bottom - graveyard_w - graveyard_gap  # 698.0 mm
    bh = board_bottom - board_top                          # 556.0 mm
    bw = page_w - 2.0 * inner                             # 490.0 mm
    cell = min(bw / 8.0, bh / 9.0)                       # 61.25 mm
    v_lo = board_top + (bh - cell * 9) / 2.0             # 144.375 mm from page top (rank-9 grid row)
    bb_grid = v_lo + cell * 9                             # 695.625 mm from page top (rank-0 grid row)

    # Physical offsets from the rank-0/file-0 marker centre (M3) to the a0 grid intersection.
    # M3 is at (marker_inset, page_h - marker_inset) from the sheet BL corner (Y-up sheet frame).
    # a0 is at (inner, page_h - bb_grid) in the same frame.
    delta_x = inner - marker_inset               # 52 - 27 = 25 mm
    delta_y = (page_h - bb_grid) - marker_inset  # 144.375 - 27 = 117.375 mm

    marker_span_x = page_w - 2.0 * marker_inset  # 540 mm centre-to-centre
    marker_span_y = page_h - 2.0 * marker_inset  # 786 mm centre-to-centre

    u_file0 = delta_x / marker_span_x                      # ≈ 0.04630
    u_file8 = (delta_x + 8.0 * cell) / marker_span_x      # ≈ 0.95370
    v_rank0 = delta_y / marker_span_y                      # ≈ 0.14932
    v_rank9 = (delta_y + 9.0 * cell) / marker_span_y      # ≈ 0.85068

    return u_file0, u_file8, v_rank0, v_rank9, cell / 1000.0


_MAT_U_FILE0, _MAT_U_FILE8, _MAT_V_RANK0, _MAT_V_RANK9, _MAT_GRID_SPACING_M = (
    _compute_4xa3_fractions()
)


class FlatBoardLocator:
    """Compute board cell XYZ analytically from 4 ArUco marker-centre 3D positions.

    The board is assumed flat (constant Z). XY for any cell is computed by
    bilinear interpolation between the 4 ArUco marker centres using exact physical
    offsets from the 4×A3 mat geometry.

    Marker ordering (same as board_detector.py REQUIRED_MARKER_IDS):
      ID 0 = file 0, rank 9  (black/far side, top-left on mat)
      ID 1 = file 8, rank 9  (black/far side, top-right)
      ID 2 = file 8, rank 0  (robot/near side, bottom-right)
      ID 3 = file 0, rank 0  (robot/near side, bottom-left)

    The marker centres define a quadrilateral (u=0,v=0 at M3; u=1,v=1 at M1).
    Grid intersections are offset from the markers because the grid starts at the
    inner-band boundary (25 mm past each marker centre in the file direction).
    """

    U_FILE0 = _MAT_U_FILE0  # ≈ 0.04630 — bilinear u for file=0 (a-file)
    U_FILE8 = _MAT_U_FILE8  # ≈ 0.95370 — bilinear u for file=8 (i-file)
    V_RANK0 = _MAT_V_RANK0  # ≈ 0.14932 — bilinear v for rank=0 (robot side)
    V_RANK9 = _MAT_V_RANK9  # ≈ 0.85068 — bilinear v for rank=9 (black side)

    def __init__(self, marker_centres_base: List[List[float]]):
        """Initialise from a flat 12-element list: [M0.x,M0.y,M0.z, M1.x,..., M3.z]."""
        flat = list(marker_centres_base)
        if len(flat) != 12:
            raise ValueError(f'Expected 12 values (4×xyz), got {len(flat)}')
        self._M = [np.array(flat[i*3:(i+1)*3]) for i in range(4)]
        self._board_z = float(np.mean([m[2] for m in self._M]))

    def cell_xyz(self, file: int, rank: int) -> np.ndarray:
        """Return 3D board-surface position for grid intersection (base_link, metres)."""
        M0, M1, M2, M3 = self._M
        u = self.U_FILE0 + file * (self.U_FILE8 - self.U_FILE0) / 8.0
        v = self.V_RANK0 + rank * (self.V_RANK9 - self.V_RANK0) / 9.0
        return (1.0 - u) * (1.0 - v) * M3 + u * (1.0 - v) * M2 + u * v * M1 + (1.0 - u) * v * M0

    @property
    def board_z(self) -> float:
        """Mean Z of the 4 ArUco marker centres (board surface, base_link metres)."""
        return self._board_z


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
                    # Accept both flat list [j0, j1, ...] and legacy nested [[j0, j1, ...]]
                    if isinstance(val[0], list):
                        val = val[0]
                    setattr(cal, key, [float(v) for v in val])
        return cal

    def _interp_joints_4patch(
        self,
        file_f: float, rank_f: float,
        J_a0, J_e0, J_i0,
        J_a5, J_e5, J_i5,
        J_a9, J_e9, J_i9,
    ) -> np.ndarray:
        """4-patch (2×2) bilinear interpolation over the 3×3 reference grid.

        File split at 4, rank split at rank_mid_idx.  Each patch is bilinear
        in its local (u, v) coordinates so all 9 anchor points are hit exactly.
        """
        rm = float(self.rank_mid_idx)
        if rank_f <= rm:
            v = rank_f / rm
            if file_f <= 4.0:
                u = file_f / 4.0
                return (1-u)*(1-v)*J_a0 + u*(1-v)*J_e0 + u*v*J_e5 + (1-u)*v*J_a5
            else:
                u = (file_f - 4.0) / 4.0
                return (1-u)*(1-v)*J_e0 + u*(1-v)*J_i0 + u*v*J_i5 + (1-u)*v*J_e5
        else:
            v = (rank_f - rm) / (9.0 - rm)
            if file_f <= 4.0:
                u = file_f / 4.0
                return (1-u)*(1-v)*J_a5 + u*(1-v)*J_e5 + u*v*J_e9 + (1-u)*v*J_a9
            else:
                u = (file_f - 4.0) / 4.0
                return (1-u)*(1-v)*J_e5 + u*(1-v)*J_i5 + u*v*J_i9 + (1-u)*v*J_e9

    def interpolate_approach_joints(
        self, file_f: float, rank_f: float
    ) -> Optional[Tuple[list, list]]:
        """Bilinear interpolation of joint angles for any board cell approach position.

        Uses 4-patch (2×2) mode when both file and rank midpoints are available.
        Falls back to 2-patch (file only) or 4-corner bilinear as midpoints allow.

        file_f / rank_f are continuous floats (0.0–8.0 / 0.0–9.0).
        Returns (joint_names, joint_positions) or None if approach joints not calibrated.
        """
        if (
            self.cell_approach_joints is None
            or len(self.cell_approach_joints) != 4
            or self.cell_approach_joint_names is None
        ):
            return None
        J_a0 = np.array(self.cell_approach_joints[0])
        J_i0 = np.array(self.cell_approach_joints[1])
        J_i9 = np.array(self.cell_approach_joints[2])
        J_a9 = np.array(self.cell_approach_joints[3])
        has_file_mid = (
            self.cell_approach_midpoints_joints is not None
            and len(self.cell_approach_midpoints_joints) == 2
        )
        has_rank_mid = (
            self.cell_approach_rank_mid_joints is not None
            and len(self.cell_approach_rank_mid_joints) == 3
        )
        if has_file_mid and has_rank_mid:
            J_e0 = np.array(self.cell_approach_midpoints_joints[0])
            J_e9 = np.array(self.cell_approach_midpoints_joints[1])
            J_a5 = np.array(self.cell_approach_rank_mid_joints[0])
            J_e5 = np.array(self.cell_approach_rank_mid_joints[1])
            J_i5 = np.array(self.cell_approach_rank_mid_joints[2])
            joints = self._interp_joints_4patch(
                file_f, rank_f,
                J_a0, J_e0, J_i0,
                J_a5, J_e5, J_i5,
                J_a9, J_e9, J_i9,
            )
        elif has_file_mid:
            J_e0 = np.array(self.cell_approach_midpoints_joints[0])
            J_e9 = np.array(self.cell_approach_midpoints_joints[1])
            v = rank_f / 9.0
            if file_f <= 4.0:
                u = file_f / 4.0
                joints = (1-u)*(1-v)*J_a0 + u*(1-v)*J_e0 + u*v*J_e9 + (1-u)*v*J_a9
            else:
                u = (file_f - 4.0) / 4.0
                joints = (1-u)*(1-v)*J_e0 + u*(1-v)*J_i0 + u*v*J_i9 + (1-u)*v*J_e9
        else:
            u = file_f / 8.0
            v = rank_f / 9.0
            joints = (1-u)*(1-v)*J_a0 + u*(1-v)*J_i0 + u*v*J_i9 + (1-u)*v*J_a9
        return self.cell_approach_joint_names, joints.tolist()

    def interpolate_board_joints(
        self, file_f: float, rank_f: float
    ) -> Optional[Tuple[list, list]]:
        """Bilinear interpolation of joint angles for any board cell grasp position.

        Uses 4-patch (2×2) mode when both file and rank midpoints are available.
        Falls back to 2-patch (file only) or 4-corner bilinear as midpoints allow.

        file_f / rank_f are continuous floats (0.0–8.0 / 0.0–9.0).
        Returns (joint_names, joint_positions) or None if board joints not calibrated.
        """
        if (
            self.calibration_corners_joints is None
            or len(self.calibration_corners_joints) != 4
            or self.calibration_corners_joint_names is None
        ):
            return None
        J_a0 = np.array(self.calibration_corners_joints[0])
        J_i0 = np.array(self.calibration_corners_joints[1])
        J_i9 = np.array(self.calibration_corners_joints[2])
        J_a9 = np.array(self.calibration_corners_joints[3])
        has_file_mid = (
            self.calibration_midpoints_joints is not None
            and len(self.calibration_midpoints_joints) == 2
        )
        has_rank_mid = (
            self.calibration_rank_mid_joints is not None
            and len(self.calibration_rank_mid_joints) == 3
        )
        if has_file_mid and has_rank_mid:
            J_e0 = np.array(self.calibration_midpoints_joints[0])
            J_e9 = np.array(self.calibration_midpoints_joints[1])
            J_a5 = np.array(self.calibration_rank_mid_joints[0])
            J_e5 = np.array(self.calibration_rank_mid_joints[1])
            J_i5 = np.array(self.calibration_rank_mid_joints[2])
            joints = self._interp_joints_4patch(
                file_f, rank_f,
                J_a0, J_e0, J_i0,
                J_a5, J_e5, J_i5,
                J_a9, J_e9, J_i9,
            )
        elif has_file_mid:
            J_e0 = np.array(self.calibration_midpoints_joints[0])
            J_e9 = np.array(self.calibration_midpoints_joints[1])
            v = rank_f / 9.0
            if file_f <= 4.0:
                u = file_f / 4.0
                joints = (1-u)*(1-v)*J_a0 + u*(1-v)*J_e0 + u*v*J_e9 + (1-u)*v*J_a9
            else:
                u = (file_f - 4.0) / 4.0
                joints = (1-u)*(1-v)*J_e0 + u*(1-v)*J_i0 + u*v*J_i9 + (1-u)*v*J_e9
        else:
            u = file_f / 8.0
            v = rank_f / 9.0
            joints = (1-u)*(1-v)*J_a0 + u*(1-v)*J_i0 + u*v*J_i9 + (1-u)*v*J_a9
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

    def grid_fraction_to_world(self, file_f: float, rank_f: float) -> np.ndarray:
        """Bilinear TCP position for fractional grid coords (0–8 file, 0–9 rank).

        Corners: a0 (0,0), i0 (8,0), i9 (8,9), a9 (0,9) — same order as calibration_corners_*.
        """
        if self.calibration_corners_base is None or len(self.calibration_corners_base) != 4:
            raise RuntimeError(
                'calibration_corners_base required for grid_fraction_to_world'
            )
        C00, C80, C89, C09 = [np.array(c) for c in self.calibration_corners_base]
        u = max(0.0, min(8.0, float(file_f))) / 8.0
        v = max(0.0, min(9.0, float(rank_f))) / 9.0
        return (1 - u) * (1 - v) * C00 + u * (1 - v) * C80 + u * v * C89 + (1 - u) * v * C09

    def world_xy_to_grid_fraction(
        self, x: float, y: float, max_xy_error_m: float = 0.05
    ) -> Optional[Tuple[float, float]]:
        """Inverse map base_link XY to fractional (file, rank) on the bilinear board grid."""
        if self.calibration_corners_base is None or len(self.calibration_corners_base) != 4:
            return None

        def _search(
            file_min: float, file_max: float, rank_min: float, rank_max: float, step: float
        ) -> tuple[float, float, float]:
            best_f, best_r, best_d2 = 0.0, 0.0, float('inf')
            f = file_min
            while f <= file_max + 1e-9:
                r = rank_min
                while r <= rank_max + 1e-9:
                    xyz = self.grid_fraction_to_world(f, r)
                    d2 = (float(xyz[0]) - x) ** 2 + (float(xyz[1]) - y) ** 2
                    if d2 < best_d2:
                        best_d2 = d2
                        best_f, best_r = f, r
                    r += step
                f += step
            return best_f, best_r, best_d2

        file_f, rank_f, best_d2 = _search(0.0, 8.0, 0.0, 9.0, 0.5)
        file_f, rank_f, best_d2 = _search(
            max(0.0, file_f - 0.5), min(8.0, file_f + 0.5),
            max(0.0, rank_f - 0.5), min(9.0, rank_f + 0.5),
            0.05,
        )
        if best_d2 > max_xy_error_m ** 2:
            return None
        return file_f, rank_f

    def ik_seed_joints(
        self, x: float, y: float, for_approach: bool
    ) -> Optional[Tuple[list, list]]:
        """IK seed from 4-corner (plus midpoint) taught joints at the board cell nearest (x, y)."""
        frac = self.world_xy_to_grid_fraction(x, y)
        if frac is None:
            return None
        file_f, rank_f = frac
        if for_approach:
            return self.interpolate_approach_joints(file_f, rank_f)
        return self.interpolate_board_joints(file_f, rank_f)

    def grid_to_world(self, file_idx: int, rank_idx: int) -> np.ndarray:
        """
        Convert grid coordinates to robot world-frame position (metres).

        Uses bilinear interpolation from the 4 measured corner TCP positions when available.
        This passes through all 4 corners exactly (zero error at corners, minimal error
        at interior points for a flat board). Falls back to rigid transform for old calibrations.
        """
        if self.calibration_corners_base is not None and len(self.calibration_corners_base) == 4:
            return self.grid_fraction_to_world(float(file_idx), float(rank_idx))

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
    """Return (from_file, from_rank, to_file, to_rank) as 0-indexed board coordinates.

    Accepts pyffish/UCI moves where ranks are 1–10 (1 = red/robot home row).
    Handles both 4-char (e.g. 'b2b8') and 5-char (e.g. 'b1b10', 'a10b9') moves.
    """
    def _sq(s: str, i: int) -> Tuple[int, int, int]:
        f = ord(s[i].lower()) - ord('a')
        # Rank 10 (black home) is encoded as two digits '10'
        if i + 2 < len(s) and s[i + 1] == '1' and s[i + 2] == '0':
            return f, 9, i + 3        # UCI rank 10 → 0-indexed rank 9
        return f, int(s[i + 1]) - 1, i + 2  # UCI rank 1–9 → 0-indexed 0–8

    ff, fr, ni = _sq(move, 0)
    tf, tr, _  = _sq(move, ni)
    return ff, fr, tf, tr


def _make_pose(x: float, y: float, z: float) -> Pose:
    """Create a Pose with fixed downward orientation (gripper pointing down)."""
    pose = Pose()
    pose.position = Point(x=x, y=y, z=z)
    # Quaternion for downward-pointing end-effector (Z-down, gripper facing down)
    pose.orientation = Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)
    return pose


class MoveTranslator:
    """Converts Xiangqi board coordinates to robot workspace Cartesian poses."""

    def __init__(
        self,
        calibration: BoardCalibration,
        graveyard_slot_x: float = 0.25,
        graveyard_slot_z: float = 0.02,
    ):
        self._cal = calibration
        self._red_graveyard = self._build_graveyard_slots(
            'red', graveyard_slot_x, graveyard_slot_z, DEFAULT_RED_GRAVEYARD, calibration
        )
        self._black_graveyard = self._build_graveyard_slots(
            'black', graveyard_slot_x, graveyard_slot_z, DEFAULT_BLACK_GRAVEYARD, calibration
        )
        self._red_graveyard_idx = 0
        self._black_graveyard_idx = 0

    @staticmethod
    def _build_graveyard_slots(
        side: str,
        slot_x: float,
        slot_z: float,
        default_slots: list,
        cal: BoardCalibration,
    ) -> list:
        """Build drop-slot list from taught graveyard_{red|black}_y in calibration YAML."""
        y = getattr(cal, f'graveyard_{side}_y', None)
        if y is None:
            return list(default_slots)
        y = float(y)
        return [(float(slot_x) + i * 0.05, y, float(slot_z)) for i in range(16)]

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
        flat_locator: Optional['FlatBoardLocator'] = None,
    ) -> Tuple[Pose, Pose, Pose, Pose, Pose]:
        """Convert a 4-char move to the 5 key waypoint poses:
          (approach_pick, grasp, lift, approach_place, place)

        When flat_locator is provided the board is treated as flat: XY is computed
        analytically from the ArUco-derived marker positions and Z is the fixed board
        surface (flat_locator.board_z) plus the configured height offsets.
        """
        if approach_height is None:
            approach_height = self._approach_height_m()
        if grasp_height is None:
            grasp_height = self._grasp_height_m()
        if transit_height is None:
            transit_height = self._transit_height_m()

        from_file, from_rank, to_file, to_rank = _parse_move(move)

        if flat_locator is not None:
            board_z = flat_locator.board_z
            pick_xyz  = np.array([*flat_locator.cell_xyz(from_file, from_rank)[:2], board_z])
            place_xyz = np.array([*flat_locator.cell_xyz(to_file,   to_rank)[:2],  board_z])
        else:
            pick_xyz  = self._cal.grid_to_world(from_file, from_rank)
            place_xyz = self._cal.grid_to_world(to_file,   to_rank)
            board_z   = float(pick_xyz[2])

        approach_pick  = _make_pose(pick_xyz[0],  pick_xyz[1],  board_z + approach_height)
        grasp_pose     = _make_pose(pick_xyz[0],  pick_xyz[1],  board_z + grasp_height)
        lift_pose      = _make_pose(pick_xyz[0],  pick_xyz[1],  board_z + transit_height)
        approach_place = _make_pose(place_xyz[0], place_xyz[1], board_z + approach_height)
        place_pose     = _make_pose(place_xyz[0], place_xyz[1], board_z + grasp_height)

        return approach_pick, grasp_pose, lift_pose, approach_place, place_pose

    def graveyard_pose(self, is_red_piece: bool) -> Pose:
        """Return the next available graveyard position for a captured piece.

        Red captured pieces go to the red graveyard zone; black pieces to the black zone.
        Slot XY comes from calibration ``graveyard_{red|black}_y`` when taught.
        """
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
