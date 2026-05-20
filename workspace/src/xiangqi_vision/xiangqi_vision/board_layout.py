"""
Normalised warp grid geometry for the 4×A3 portrait mat.

Homography maps ArUco **sheet corners** to the outer margin quad in the 800×890
warp. Intersections sit **inset** on the printed mat (graveyard strips, ArUco
bands). Mapping detections with uniform margin spacing shifts ranks ~1 toward
the centre — use the same layout as tools/generate_board_svg.py (LAYOUT_4XA3).
"""

from __future__ import annotations

import math
from typing import Tuple

# Must match BoardDetector / PieceDetector warp size
NORM_W = 800
NORM_H = 890
MARGIN = 44

BOARD_FILES = 9
BOARD_RANKS = 10

# 4×A3 portrait mat (mm) — tools/generate_board_svg.py
PAGE_W_MM_4XA3 = 594.0
PAGE_H_MM_4XA3 = 840.0

ARUCO_PAGE_INSET_MM = 8.0
GRAVEYARD_GAP_MM = 5.0

LAYOUT_4XA3 = {
    'graveyard_w_mm': 85.0,
    'graveyard_placement': 'ends',
    'aruco_mm': 38.0,
    'aruco_band_gap_mm': 6.0,
    'corner_inset_mm': 1.0,
    'graveyard_clip_to_aruco_band': True,
}


def _compute_4xa3_grid_mm() -> Tuple[object, object]:
    """Return (gx, gy) callables in page mm; rank 0 bottom, rank 9 top."""
    opts = LAYOUT_4XA3
    page_w, page_h = PAGE_W_MM_4XA3, PAGE_H_MM_4XA3
    graveyard_w = float(opts['graveyard_w_mm'])
    aruco = float(opts['aruco_mm'])
    inset = float(opts['corner_inset_mm'])
    band = float(opts['aruco_band_gap_mm'])

    aruco_inset = max(inset, ARUCO_PAGE_INSET_MM)
    inner_left = aruco_inset + aruco + band
    inner_right = page_w - aruco_inset - aruco - band
    inner_top = aruco_inset + aruco + band
    inner_bottom = page_h - aruco_inset - aruco - band

    strip_d = graveyard_w
    bl = inner_left
    br = inner_right
    bw = br - bl
    board_top = inner_top + strip_d + GRAVEYARD_GAP_MM
    board_bottom = inner_bottom - strip_d - GRAVEYARD_GAP_MM
    bh = board_bottom - board_top
    cell = min(bw / 8.0, bh / 9.0)
    grid_w = cell * 8.0
    grid_h = cell * 9.0
    bl_grid = bl + (bw - grid_w) / 2.0
    v_lo = board_top + (bh - grid_h) / 2.0
    bb_grid = v_lo + grid_h

    def gx(file_idx: int) -> float:
        return bl_grid + file_idx * cell

    def gy(rank_idx: int) -> float:
        return bb_grid - rank_idx * cell

    return gx, gy, inner_left, inner_right, inner_top, inner_bottom


def _dst_quad_corners() -> Tuple[Tuple[float, float], ...]:
    """TL, TR, BR, BL in norm pixels (matches BoardDetector._dst_corners)."""
    return (
        (MARGIN, NORM_H - MARGIN),          # ID 0 — file 0, rank 9
        (NORM_W - MARGIN, NORM_H - MARGIN),  # ID 1
        (NORM_W - MARGIN, MARGIN),          # ID 2 — file 8, rank 0
        (MARGIN, MARGIN),                   # ID 3 — file 0, rank 0
    )


def norm_pixel_at_intersection(file_idx: int, rank_idx: int) -> Tuple[float, float]:
    """
    Expected (px, py) in the warped image for a grid intersection.

    Uses bilinear (u, v) inside the inner mat band, mapped to the same outer
    quad as the ArUco homography destination corners.
    """
    gx, gy, il, ir, it, ib = _compute_4xa3_grid_mm()
    u = (gx(file_idx) - il) / (ir - il)
    v = (gy(rank_idx) - it) / (ib - it)
    u = max(0.0, min(1.0, u))
    v = max(0.0, min(1.0, v))

    tl, tr, br, bl = _dst_quad_corners()
    px = (
        (1.0 - u) * (1.0 - v) * tl[0]
        + u * (1.0 - v) * tr[0]
        + u * v * br[0]
        + (1.0 - u) * v * bl[0]
    )
    py = (
        (1.0 - u) * (1.0 - v) * tl[1]
        + u * (1.0 - v) * tr[1]
        + u * v * br[1]
        + (1.0 - u) * v * bl[1]
    )
    return px, py


def pixel_to_grid(px: float, py: float) -> Tuple[int, int]:
    """Snap to the closest of 90 intersections (handles inset grid vs margin warp)."""
    best_f, best_r = -1, -1
    best_d = math.inf
    for f in range(BOARD_FILES):
        for r in range(BOARD_RANKS):
            ex, ey = norm_pixel_at_intersection(f, r)
            d = (px - ex) ** 2 + (py - ey) ** 2
            if d < best_d:
                best_d = d
                best_f, best_r = f, r
    return best_f, best_r
