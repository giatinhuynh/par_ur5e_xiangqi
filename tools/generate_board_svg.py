#!/usr/bin/env python3
"""
generate_board_svg.py
Generates a print-ready SVG file for the Xiangqi robot board mat.

The board mat includes:
  - Standard 9x10 Xiangqi grid (files a-i, ranks 0-9)
  - River zone (blank gap between ranks 4 and 5)
  - Palace diagonal lines (ranks 0-2 and 7-9)
  - ArUco markers (4x4_50, IDs 0-3) in each corner for automatic calibration
  - Graveyard zones on left and right sides for captured pieces
  - File/rank labels for debugging

Output: board_mat.svg (A2 size: 594mm x 420mm landscape)
        Also generates board_mat_A3.svg (420mm x 297mm) for smaller prints.

Usage:
    python3 tools/generate_board_svg.py

Requires: numpy, opencv-contrib-python (for ArUco generation)
    pip install numpy opencv-contrib-python
"""

import math
import sys
import os
import numpy as np

try:
    import cv2
    OPENCV_OK = True
except ImportError:
    OPENCV_OK = False
    print("WARNING: opencv-contrib-python not installed. ArUco markers will be placeholder squares.")

# -----------------------------------------------------------------------
# Board geometry constants (all in mm, for A2 landscape: 594 x 420 mm)
# -----------------------------------------------------------------------

PAGE_W_MM   = 594.0   # A2 landscape width
PAGE_H_MM   = 420.0   # A2 landscape height

BOARD_MARGIN_MM  = 30.0   # Margin from page edge to outer board line
ARUCO_SIZE_MM    = 22.0   # ArUco marker square side length
ARUCO_MARGIN_MM  = 4.0    # Gap between ArUco marker and nearest board line
GRAVEYARD_W_MM   = 30.0   # Width of each graveyard zone (left/right of board)
GRAVEYARD_GAP_MM = 5.0    # Gap between graveyard zone and board

FILES = 9
RANKS = 10

# Compute cell spacing from available board width
# Board occupies from (BOARD_MARGIN_MM + GRAVEYARD_W_MM + GRAVEYARD_GAP_MM) to
# (PAGE_W_MM - BOARD_MARGIN_MM - GRAVEYARD_W_MM - GRAVEYARD_GAP_MM)
BOARD_LEFT   = BOARD_MARGIN_MM + GRAVEYARD_W_MM + GRAVEYARD_GAP_MM
BOARD_RIGHT  = PAGE_W_MM - BOARD_MARGIN_MM - GRAVEYARD_W_MM - GRAVEYARD_GAP_MM
BOARD_TOP    = BOARD_MARGIN_MM + ARUCO_SIZE_MM + ARUCO_MARGIN_MM
BOARD_BOTTOM = PAGE_H_MM - BOARD_MARGIN_MM - ARUCO_SIZE_MM - ARUCO_MARGIN_MM

BOARD_W = BOARD_RIGHT - BOARD_LEFT
BOARD_H = BOARD_BOTTOM - BOARD_TOP

CELL_X = BOARD_W / (FILES - 1)   # Spacing between files (columns)
CELL_Y = BOARD_H / (RANKS - 1)   # Spacing between ranks (rows)

# rank 0 = red/robot side = BOARD_BOTTOM, rank 9 = black side = BOARD_TOP
def gx(file_idx: int) -> float:
    return BOARD_LEFT + file_idx * CELL_X

def gy(rank_idx: int) -> float:
    return BOARD_BOTTOM - rank_idx * CELL_Y

# -----------------------------------------------------------------------
# SVG helpers
# -----------------------------------------------------------------------

def line(x1, y1, x2, y2, stroke='#1a1a1a', width=0.5):
    return f'  <line x1="{x1:.2f}" y1="{y1:.2f}" x2="{x2:.2f}" y2="{y2:.2f}" stroke="{stroke}" stroke-width="{width}"/>\n'

def rect(x, y, w, h, fill='none', stroke='#1a1a1a', width=0.5, rx=0):
    return f'  <rect x="{x:.2f}" y="{y:.2f}" width="{w:.2f}" height="{h:.2f}" fill="{fill}" stroke="{stroke}" stroke-width="{width}" rx="{rx}"/>\n'

def text(x, y, content, size=3.5, anchor='middle', fill='#1a1a1a', bold=False):
    weight = ' font-weight="bold"' if bold else ''
    return f'  <text x="{x:.2f}" y="{y:.2f}" font-size="{size}" text-anchor="{anchor}" fill="{fill}"{weight} font-family="serif">{content}</text>\n'

def path(d, stroke='#1a1a1a', width=0.5, fill='none'):
    return f'  <path d="{d}" stroke="{stroke}" stroke-width="{width}" fill="{fill}"/>\n'

def aruco_placeholder(x, y, size, marker_id):
    """Render a placeholder square with marker ID text if OpenCV is not available."""
    svg = rect(x, y, size, size, fill='#000000', stroke='none')
    svg += text(x + size/2, y + size/2 + 1.5, f'ID:{marker_id}', size=3.0, fill='#ffffff')
    return svg

def aruco_to_svg(x, y, size_mm, marker_id, pixels=7):
    """
    Render an ArUco 4x4_50 marker at (x, y) with given size in mm.
    Falls back to placeholder if OpenCV unavailable.
    """
    if not OPENCV_OK:
        return aruco_placeholder(x, y, size_mm, marker_id)

    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    marker_img = np.zeros((pixels, pixels), dtype=np.uint8)
    marker_img = cv2.aruco.generateImageMarker(aruco_dict, marker_id, pixels)

    cell = size_mm / pixels
    svg = ''
    # White background
    svg += rect(x, y, size_mm, size_mm, fill='#ffffff', stroke='none')
    for row in range(pixels):
        for col in range(pixels):
            if marker_img[row, col] == 0:  # Black cell
                cx = x + col * cell
                cy = y + row * cell
                svg += f'  <rect x="{cx:.3f}" y="{cy:.3f}" width="{cell:.3f}" height="{cell:.3f}" fill="#000000"/>\n'
    return svg

# -----------------------------------------------------------------------
# Main SVG generation
# -----------------------------------------------------------------------

def generate_board_svg(output_path: str, page_w=PAGE_W_MM, page_h=PAGE_H_MM):
    svg_parts = []

    # SVG header
    svg_parts.append(
        f'<?xml version="1.0" encoding="UTF-8"?>\n'
        f'<svg xmlns="http://www.w3.org/2000/svg" '
        f'width="{page_w}mm" height="{page_h}mm" '
        f'viewBox="0 0 {page_w} {page_h}">\n'
    )

    # Background
    svg_parts.append(rect(0, 0, page_w, page_h, fill='#f5e6c8', stroke='none'))

    # -----------------------------------------------------------------------
    # ArUco markers at 4 corners
    # Placement: ID 0 = top-left, ID 1 = top-right, ID 2 = bottom-right, ID 3 = bottom-left
    # -----------------------------------------------------------------------
    margin = BOARD_MARGIN_MM
    marker_positions = [
        (margin, margin, 0),                                                         # top-left
        (page_w - margin - ARUCO_SIZE_MM, margin, 1),                                # top-right
        (page_w - margin - ARUCO_SIZE_MM, page_h - margin - ARUCO_SIZE_MM, 2),       # bottom-right
        (margin, page_h - margin - ARUCO_SIZE_MM, 3),                                # bottom-left
    ]
    for mx, my, mid in marker_positions:
        svg_parts.append(aruco_to_svg(mx, my, ARUCO_SIZE_MM, mid))
        svg_parts.append(text(mx + ARUCO_SIZE_MM/2, my + ARUCO_SIZE_MM + 4, f'ArUco {mid}', size=3.0))

    # -----------------------------------------------------------------------
    # Graveyard zones
    # -----------------------------------------------------------------------
    gy_left  = BOARD_LEFT - GRAVEYARD_GAP_MM - GRAVEYARD_W_MM
    gy_right = BOARD_RIGHT + GRAVEYARD_GAP_MM
    svg_parts.append(rect(gy_left, BOARD_TOP, GRAVEYARD_W_MM, BOARD_H,
                          fill='#e8d5a0', stroke='#888', width=0.4))
    svg_parts.append(text(gy_left + GRAVEYARD_W_MM/2, BOARD_TOP - 3, 'BLACK\nCAPTURED', size=3.0))
    svg_parts.append(rect(gy_right, BOARD_TOP, GRAVEYARD_W_MM, BOARD_H,
                          fill='#e8d5a0', stroke='#888', width=0.4))
    svg_parts.append(text(gy_right + GRAVEYARD_W_MM/2, BOARD_TOP - 3, 'RED\nCAPTURED', size=3.0))

    # -----------------------------------------------------------------------
    # Board grid
    # -----------------------------------------------------------------------

    # Vertical lines (file lines)
    for f in range(FILES):
        x = gx(f)
        if f == 0 or f == FILES - 1:
            # Edge files: full continuous line
            svg_parts.append(line(x, gy(0), x, gy(RANKS - 1), width=0.7))
        else:
            # Interior files: split at river (gap between ranks 4 and 5)
            svg_parts.append(line(x, gy(0), x, gy(4), width=0.5))
            svg_parts.append(line(x, gy(5), x, gy(RANKS - 1), width=0.5))

    # Horizontal lines (rank lines)
    for r in range(RANKS):
        y = gy(r)
        svg_parts.append(line(gx(0), y, gx(FILES - 1), y, width=0.5))

    # -----------------------------------------------------------------------
    # Palace diagonals
    # Red palace: files 3-5, ranks 0-2
    # Black palace: files 3-5, ranks 7-9
    # -----------------------------------------------------------------------
    for (r_lo, r_hi) in [(0, 2), (7, 9)]:
        svg_parts.append(line(gx(3), gy(r_lo), gx(5), gy(r_hi), width=0.5))
        svg_parts.append(line(gx(5), gy(r_lo), gx(3), gy(r_hi), width=0.5))

    # -----------------------------------------------------------------------
    # River zone label
    # -----------------------------------------------------------------------
    river_cy = (gy(4) + gy(5)) / 2
    svg_parts.append(text((gx(0) + gx(4)) / 2, river_cy + 1.5, '楚  河', size=7, fill='#5c3d1a', bold=True))
    svg_parts.append(text((gx(4) + gx(8)) / 2, river_cy + 1.5, '漢  界', size=7, fill='#5c3d1a', bold=True))

    # -----------------------------------------------------------------------
    # Cannon starting position markers (small circles at b2, h2, b7, h7)
    # -----------------------------------------------------------------------
    cannon_positions = [(1, 2), (7, 2), (1, 7), (7, 7)]
    for (f, r) in cannon_positions:
        cx, cy = gx(f), gy(r)
        for dx, dy in [(-1,-1),(-1,1),(1,-1),(1,1)]:
            tick_len = 2.0
            svg_parts.append(line(cx + dx * 0.8, cy + dy * 0.8,
                                  cx + dx * (0.8 + tick_len), cy,  width=0.35))
            svg_parts.append(line(cx, cy + dy * 0.8,
                                  cx, cy + dy * (0.8 + tick_len), width=0.35))

    # Soldier starting position markers (ranks 3 and 6, files 0,2,4,6,8)
    soldier_positions = [(f, r) for f in [0,2,4,6,8] for r in [3,6]]
    for (f, r) in soldier_positions:
        cx, cy = gx(f), gy(r)
        tick_len = 1.5
        sides = []
        if f > 0: sides.append((-1, 0))
        if f < 8: sides.append(( 1, 0))
        for (dx, dy) in sides:
            for oy in [-1, 1]:
                svg_parts.append(line(cx + dx * 0.7, cy + oy * 0.7,
                                      cx + dx * (0.7 + tick_len), cy + oy * 0.7, width=0.3))
                svg_parts.append(line(cx + dx * (0.7 + tick_len), cy + oy * 0.7,
                                      cx + dx * (0.7 + tick_len), cy, width=0.3))

    # -----------------------------------------------------------------------
    # File labels (a-i) below the board
    # -----------------------------------------------------------------------
    for f, label in enumerate('abcdefghi'):
        svg_parts.append(text(gx(f), gy(0) + 7, label.upper(), size=3.5, fill='#555'))

    # Rank labels (0-9) to the left of the board
    for r in range(RANKS):
        svg_parts.append(text(gx(0) - 5, gy(r) + 1.2, str(r), size=3.5, anchor='end', fill='#555'))

    # -----------------------------------------------------------------------
    # Side labels (RED / BLACK)
    # -----------------------------------------------------------------------
    svg_parts.append(text((gx(0) + gx(8)) / 2, gy(0) + 14, '红方  RED (Robot)', size=4.5, fill='#c0392b', bold=True))
    svg_parts.append(text((gx(0) + gx(8)) / 2, gy(9) - 6,  '黑方  BLACK (Human)', size=4.5, fill='#1a1a1a', bold=True))

    # -----------------------------------------------------------------------
    # Board outer border
    # -----------------------------------------------------------------------
    svg_parts.append(rect(gx(0), gy(RANKS-1), BOARD_W, BOARD_H, fill='none', stroke='#1a1a1a', width=1.0))

    # -----------------------------------------------------------------------
    # Calibration info box
    # -----------------------------------------------------------------------
    info_x = margin
    info_y = page_h - margin - 8
    svg_parts.append(text(info_x, info_y,
        f'Grid: {FILES}x{RANKS} | Cell: {CELL_X:.1f}x{CELL_Y:.1f} mm | ArUco 4x4_50 IDs 0-3',
        size=2.8, anchor='start', fill='#888'))

    svg_parts.append('</svg>\n')
    svg_content = ''.join(svg_parts)

    with open(output_path, 'w', encoding='utf-8') as f:
        f.write(svg_content)

    print(f'Board SVG written to: {output_path}')
    print(f'Page size: {page_w}mm x {page_h}mm')
    print(f'Board area: {BOARD_W:.1f}mm x {BOARD_H:.1f}mm')
    print(f'Cell spacing: {CELL_X:.2f}mm (file) x {CELL_Y:.2f}mm (rank)')
    print(f'ArUco marker size: {ARUCO_SIZE_MM}mm')
    return CELL_X, CELL_Y


if __name__ == '__main__':
    out_a2 = os.path.join(os.path.dirname(__file__), '..', 'docs', 'board_mat_A2.svg')
    cx, cy = generate_board_svg(out_a2, PAGE_W_MM, PAGE_H_MM)

    # Also generate A3 version (scaled down)
    scale = 420.0 / 594.0
    out_a3 = os.path.join(os.path.dirname(__file__), '..', 'docs', 'board_mat_A3.svg')

    # For A3, reduce page but keep same design -- just scale page
    # Regenerate with A3 dimensions: 420mm x 297mm landscape
    generate_board_svg(out_a3, 420.0, 297.0)

    print('\nPrint instructions:')
    print('  A2: Open board_mat_A2.svg in a PDF viewer, print to A2 at 100% scale (no fit-to-page)')
    print('  A3: Open board_mat_A3.svg, print to A3 at 100% scale')
    print(f'\nAfter printing, verify the cell spacing with a ruler:')
    print(f'  A2 version: {cx:.1f}mm between adjacent file lines')
