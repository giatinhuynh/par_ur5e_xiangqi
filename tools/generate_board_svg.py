#!/usr/bin/env python3
"""
generate_board_svg.py
Generates a print-ready SVG for a Xiangqi robot board mat.

Layout rules (fixes overlap / corner issues):
  - ArUco markers sit just **outside the 9×10 grid frame** (same corners as
    xiangqi_vision BoardDetector homography). A **table-safe page inset** keeps
    marker squares off the sheet / table edge.
  - **Square** grid cells: cell = min(file_pitch, rank_pitch), grid centred
    in the interior rectangle (standard look; homography still valid).
  - Spec / calibration line in the **top** margin band.
  - **a–i** sit directly under the bottom grid line with the same clearance as **0–9** left of gx(0).
  - **红方** sits **below** the file row; **黑方** sits **above** the top grid with extra margin (CJK/serif).

Output: docs/board_mat_A2.svg, docs/board_mat_A3.svg
"""

import os
import math
import numpy as np

try:
    import cv2
    OPENCV_OK = True
except ImportError:
    OPENCV_OK = False
    print('WARNING: opencv-contrib-python not installed. ArUco markers will be placeholder squares.')

# -----------------------------------------------------------------------
# ISO page sizes (mm, landscape)
# -----------------------------------------------------------------------
PAGE_W_MM_A2 = 594.0
PAGE_H_MM_A2 = 420.0
PAGE_W_MM_A3 = 420.0
PAGE_H_MM_A3 = 297.0

# Corner ArUcos: bleed inset (mm) used for inner layout band only.
CORNER_INSET_MM = 1.5

# Minimum distance from **page edge** to **marker square outer edge** (mm).
# Use this so markers stay on the physical table when the sheet is trimmed or smaller than the table lip.
ARUCO_PAGE_INSET_MM = 8.0

# Gap between ArUco block and inner content (graveyards + board) — stops text/grid touching markers
ARUCO_BAND_GAP_MM = 10.0

# Distance from outer grid intersection to ArUco **centre** along the outward diagonal
# (marker sits outside the line frame; must stay inside page / clear of strips).
ARUCO_GRID_STANDOFF_MM = 1.5

GRAVEYARD_W_MM = 28.0
GRAVEYARD_GAP_MM = 5.0

# Rank (0–9) / file (a–i) labels: same font; horizontal gap = gx(0) - rank_label_x;
# file capitals use baseline so (top of glyph) − gy(0) equals that gap.
COORD_LABEL_FONT_MM = 3.5
COORD_LABEL_LINE_GAP_MM = 5.0  # preferred gap grid line → glyph; overridden if graveyard clamps rank x
CAP_ASCENT_FROM_BASELINE_MM = 0.76 * COORD_LABEL_FONT_MM
COORD_DESCENDER_PAD_MM = 1.0  # ink below coord baseline (serif caps)

# Human / robot titles — outside grid; RED stacks under a–i so coord spacing stays symmetric.
SIDE_LABEL_FONT_MM = 3.6
SIDE_LABEL_CAP_ASCENT_MM = 0.82 * SIDE_LABEL_FONT_MM
SIDE_DESCENDER_PAD_MM = 1.1  # below RED baseline
TITLE_GRID_EXTRA_CLEAR_MM = 3.8  # extra above top line so 黑方 does not touch rank-9 line (CJK)
COORD_TO_TITLE_VERTICAL_GAP_MM = 3.0  # gap: bottom of a–i row → top of 红方

FILES = 9
RANKS = 10


def aruco_size_for_page(page_w: float, page_h: float) -> float:
    """Larger markers on bigger sheets; keep within print margins."""
    s = min(page_w, page_h) * 0.065
    return max(26.0, min(42.0, s))


def rect(x, y, w, h, fill='none', stroke='#1a1a1a', width=0.5, rx=0):
    return (
        f'  <rect x="{x:.2f}" y="{y:.2f}" width="{w:.2f}" height="{h:.2f}" '
        f'fill="{fill}" stroke="{stroke}" stroke-width="{width}" rx="{rx}"/>\n'
    )


def line(x1, y1, x2, y2, stroke='#1a1a1a', width=0.5):
    return (
        f'  <line x1="{x1:.2f}" y1="{y1:.2f}" x2="{x2:.2f}" y2="{y2:.2f}" '
        f'stroke="{stroke}" stroke-width="{width}"/>\n'
    )


def text(x, y, content, size=3.5, anchor='middle', fill='#1a1a1a', bold=False, baseline='auto'):
    weight = ' font-weight="bold"' if bold else ''
    dom = f' dominant-baseline="{baseline}"' if baseline != 'auto' else ''
    return (
        f'  <text x="{x:.2f}" y="{y:.2f}" font-size="{size}" text-anchor="{anchor}" '
        f'fill="{fill}"{weight} font-family="serif"{dom}>{content}</text>\n'
    )


def aruco_placeholder(x, y, size, marker_id):
    svg = rect(x, y, size, size, fill='#000000', stroke='none')
    svg += text(x + size / 2, y + size / 2 + 1.5, f'ID:{marker_id}', size=3.0, fill='#ffffff')
    return svg


def aruco_to_svg(x, y, size_mm, marker_id, pixels=7):
    if not OPENCV_OK:
        return aruco_placeholder(x, y, size_mm, marker_id)

    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    marker_img = cv2.aruco.generateImageMarker(aruco_dict, marker_id, pixels)
    cell = size_mm / pixels
    svg = rect(x, y, size_mm, size_mm, fill='#ffffff', stroke='none')
    for row in range(pixels):
        for col in range(pixels):
            if marker_img[row, col] == 0:
                cx = x + col * cell
                cy = y + row * cell
                svg += f'  <rect x="{cx:.3f}" y="{cy:.3f}" width="{cell:.3f}" height="{cell:.3f}" fill="#000000"/>\n'
    return svg


def compute_layout(page_w: float, page_h: float):
    """
    Returns dict with geometry helpers.
    Origin top-left; rank 0 = bottom (robot/red), rank 9 = top (human/black).
    """
    aruco = aruco_size_for_page(page_w, page_h)
    inset = CORNER_INSET_MM
    band = ARUCO_BAND_GAP_MM

    # Inner rectangle that may contain graveyards + board (never overlaps corner ArUcos)
    inner_left = inset + aruco + band
    inner_right = page_w - inset - aruco - band
    inner_top = inset + aruco + band
    inner_bottom = page_h - inset - aruco - band

    iw = inner_right - inner_left
    ih = inner_bottom - inner_top
    if iw < 120 or ih < 100:
        raise ValueError(
            f'Page {page_w}×{page_h} mm too small after ArUco bands (inner {iw:.0f}×{ih:.0f} mm).'
        )

    bl = inner_left + GRAVEYARD_W_MM + GRAVEYARD_GAP_MM
    br = inner_right - GRAVEYARD_W_MM - GRAVEYARD_GAP_MM
    bw = br - bl
    bh = inner_bottom - inner_top
    if bw < 60 or bh < 60:
        raise ValueError('Board interior too small; reduce graveyard width or print larger.')

    # Square cells, centred in [bl, br] × [inner_top, inner_bottom]
    cell = min(bw / (FILES - 1), bh / (RANKS - 1))
    grid_w = cell * (FILES - 1)
    grid_h = cell * (RANKS - 1)
    bl_grid = bl + (bw - grid_w) / 2.0
    v_lo = inner_top + (bh - grid_h) / 2.0
    bb_grid = v_lo + grid_h  # y of rank-0 horizontal (bottom edge of grid)

    def gx(file_idx: int) -> float:
        return bl_grid + file_idx * cell

    def gy(rank_idx: int) -> float:
        return bb_grid - rank_idx * cell

    # Graveyard strips: same vertical span as play grid, clear of corner markers
    gy_left = inner_left
    gy_right = br + GRAVEYARD_GAP_MM

    return {
        'aruco': aruco,
        'inset': inset,
        'inner_left': inner_left,
        'inner_right': inner_right,
        'inner_top': inner_top,
        'inner_bottom': inner_bottom,
        'bl_grid': bl_grid,
        'br': br,
        'bb_grid': bb_grid,
        'cell': cell,
        'grid_w': grid_w,
        'grid_h': grid_h,
        'gx': gx,
        'gy': gy,
        'gy_left': gy_left,
        'gy_right': gy_right,
        'grave_top': v_lo,
        'grave_h': grid_h,
    }


def generate_board_svg(output_path: str, page_w: float, page_h: float):
    L = compute_layout(page_w, page_h)
    aruco = L['aruco']
    inset = L['inset']
    gx, gy = L['gx'], L['gy']
    bl_grid, br = L['bl_grid'], L['br']
    bb_grid = L['bb_grid']
    cell = L['cell']
    grid_w, grid_h = L['grid_w'], L['grid_h']
    gy_left, gy_right = L['gy_left'], L['gy_right']
    gt, gh = L['grave_top'], L['grave_h']

    svg_parts = [
        '<?xml version="1.0" encoding="UTF-8"?>\n',
        f'<svg xmlns="http://www.w3.org/2000/svg" '
        f'width="{page_w}mm" height="{page_h}mm" viewBox="0 0 {page_w} {page_h}">\n',
        rect(0, 0, page_w, page_h, fill='#f5e6c8', stroke='none'),
    ]

    # --- ArUco: corners of grid frame (IDs match board_detector.py) ---
    # ID 0 TL (file 0, rank 9), 1 TR, 2 BR, 3 BL — centres offset outside intersections
    # so markers do not cover piece homes; quad is ~parallel to grid for homography.
    cx = (gx(0) + gx(8)) / 2.0
    cy = (gy(0) + gy(9)) / 2.0
    half_diag = aruco / 2.0 + ARUCO_GRID_STANDOFF_MM
    grid_corners = [
        (gx(0), gy(9)),  # 0 TL
        (gx(8), gy(9)),  # 1 TR
        (gx(8), gy(0)),  # 2 BR
        (gx(0), gy(0)),  # 3 BL
    ]
    min_cx = gy_left + GRAVEYARD_W_MM + GRAVEYARD_GAP_MM + aruco / 2.0 + 0.5
    aruco_positions_ul = []
    for mid, (px, py) in enumerate(grid_corners):
        vx, vy = px - cx, py - cy
        n = math.hypot(vx, vy) or 1.0
        mxc = px + (vx / n) * half_diag
        myc = py + (vy / n) * half_diag
        if mid in (0, 3):
            mxc = max(min_cx, mxc)
        hm = aruco / 2.0
        # Keep marker **square** fully inside page (table-safe margin on all sides).
        edge = ARUCO_PAGE_INSET_MM + hm
        mxc = max(edge, min(page_w - edge, mxc))
        myc = max(edge, min(page_h - edge, myc))
        mx_ul, my_ul = mxc - hm, myc - hm
        aruco_positions_ul.append((mx_ul, my_ul, mid))

    for mx, my, mid in aruco_positions_ul:
        svg_parts.append(aruco_to_svg(mx, my, aruco, mid))
        if mid == 0:
            lx, ly = mx + aruco + 2.0, my + aruco * 0.55
            anc = 'start'
        elif mid == 1:
            lx, ly = mx - 2.0, my + aruco * 0.55
            anc = 'end'
        elif mid == 2:
            lx, ly = mx - 2.0, my + aruco * 0.45
            anc = 'end'
        else:
            lx, ly = mx + aruco + 2.0, my + aruco * 0.45
            anc = 'start'
        svg_parts.append(text(lx, ly, f'{mid}', size=2.8, anchor=anc, fill='#666'))

    # --- Graveyards (full height of grid band; labels centred inside) ---
    svg_parts.append(
        rect(gy_left, gt, GRAVEYARD_W_MM, gh, fill='#e8d5a0', stroke='#888', width=0.4)
    )
    svg_parts.append(
        text(gy_left + GRAVEYARD_W_MM / 2, gt + gh / 2 - 2, 'BLACK', size=3.2, fill='#222', baseline='middle')
    )
    svg_parts.append(
        text(gy_left + GRAVEYARD_W_MM / 2, gt + gh / 2 + 3.5, 'CAPTURED', size=2.6, fill='#444', baseline='middle')
    )

    svg_parts.append(
        rect(gy_right, gt, GRAVEYARD_W_MM, gh, fill='#e8d5a0', stroke='#888', width=0.4)
    )
    svg_parts.append(
        text(gy_right + GRAVEYARD_W_MM / 2, gt + gh / 2 - 2, 'RED', size=3.2, fill='#8b0000', baseline='middle')
    )
    svg_parts.append(
        text(gy_right + GRAVEYARD_W_MM / 2, gt + gh / 2 + 3.5, 'CAPTURED', size=2.6, fill='#444', baseline='middle')
    )

    # --- Grid ---
    for f in range(FILES):
        x = gx(f)
        if f == 0 or f == FILES - 1:
            svg_parts.append(line(x, gy(0), x, gy(RANKS - 1), width=0.7))
        else:
            svg_parts.append(line(x, gy(0), x, gy(4), width=0.5))
            svg_parts.append(line(x, gy(5), x, gy(RANKS - 1), width=0.5))

    for r in range(RANKS):
        y = gy(r)
        svg_parts.append(line(gx(0), y, gx(FILES - 1), y, width=0.5))

    for (r_lo, r_hi) in [(0, 2), (7, 9)]:
        svg_parts.append(line(gx(3), gy(r_lo), gx(5), gy(r_hi), width=0.5))
        svg_parts.append(line(gx(5), gy(r_lo), gx(3), gy(r_hi), width=0.5))

    river_cy = (gy(4) + gy(5)) / 2
    svg_parts.append(text((gx(0) + gx(4)) / 2, river_cy + 1.5, '楚  河', size=7, fill='#5c3d1a', bold=True))
    svg_parts.append(text((gx(4) + gx(8)) / 2, river_cy + 1.5, '漢  界', size=7, fill='#5c3d1a', bold=True))

    for (f, r) in [(1, 2), (7, 2), (1, 7), (7, 7)]:
        px, py = gx(f), gy(r)
        for dx, dy in [(-1, -1), (-1, 1), (1, -1), (1, 1)]:
            tick_len = 2.0
            svg_parts.append(
                line(px + dx * 0.8, py + dy * 0.8, px + dx * (0.8 + tick_len), py, width=0.35)
            )
            svg_parts.append(
                line(px, py + dy * 0.8, px, py + dy * (0.8 + tick_len), width=0.35)
            )

    for (f, r) in [(f, r) for f in [0, 2, 4, 6, 8] for r in [3, 6]]:
        px, py = gx(f), gy(r)
        tick_len = 1.5
        sides = []
        if f > 0:
            sides.append((-1, 0))
        if f < 8:
            sides.append((1, 0))
        for (dx, dy) in sides:
            for oy in [-1, 1]:
                svg_parts.append(
                    line(
                        px + dx * 0.7,
                        py + oy * 0.7,
                        px + dx * (0.7 + tick_len),
                        py + oy * 0.7,
                        width=0.3,
                    )
                )
                svg_parts.append(
                    line(
                        px + dx * (0.7 + tick_len),
                        py + oy * 0.7,
                        px + dx * (0.7 + tick_len),
                        py,
                        width=0.3,
                    )
                )

    rank_label_x = max(
        gx(0) - COORD_LABEL_LINE_GAP_MM,
        gy_left + GRAVEYARD_W_MM + 2.0,
    )
    coord_gap_mm = gx(0) - rank_label_x
    for r in range(RANKS):
        svg_parts.append(
            text(
                rank_label_x,
                gy(r) + 1.2,
                str(r),
                size=COORD_LABEL_FONT_MM,
                anchor='end',
                fill='#555',
            )
        )

    # --- Border rect (then labels on top of stroke) ---
    svg_parts.append(rect(gx(0), gy(RANKS - 1), grid_w, grid_h, fill='none', stroke='#1a1a1a', width=1.0))

    # Spec line (top margin) first so it stays under any rare overlap with 黑方.
    gap_mid_y = inset + aruco + ARUCO_BAND_GAP_MM * 0.55
    svg_parts.append(
        text(
            page_w / 2,
            gap_mid_y,
            (
                f'{page_w:.0f}×{page_h:.0f} mm | {FILES}×{RANKS} | cell {cell:.1f} mm sq | '
                f'ArUco 4×4_50 0–3 | {aruco:.0f} mm'
            ),
            size=2.3,
            anchor='middle',
            fill='#777',
            baseline='middle',
        )
    )

    # BLACK: baseline well above rank-9 line (CJK can sit low on alphabetic baseline).
    black_y = gy(9) - coord_gap_mm - TITLE_GRID_EXTRA_CLEAR_MM
    svg_parts.append(
        text(
            (gx(0) + gx(8)) / 2,
            black_y,
            '黑方  BLACK (Human)',
            size=SIDE_LABEL_FONT_MM,
            fill='#1a1a1a',
            bold=True,
        )
    )

    # File letters: same geometry as rank digits (line → top of cap = coord_gap_mm).
    label_y = gy(0) + coord_gap_mm + CAP_ASCENT_FROM_BASELINE_MM
    for f, label in enumerate('abcdefghi'):
        svg_parts.append(text(gx(f), label_y, label.upper(), size=COORD_LABEL_FONT_MM, fill='#555'))

    # RED: below a–i row (keeps A–I tight to grid like 0–9 on the left).
    file_row_bottom = label_y + COORD_DESCENDER_PAD_MM
    red_baseline = file_row_bottom + COORD_TO_TITLE_VERTICAL_GAP_MM + SIDE_LABEL_CAP_ASCENT_MM
    svg_parts.append(
        text(
            (gx(0) + gx(8)) / 2,
            red_baseline,
            '红方  RED (Robot)',
            size=SIDE_LABEL_FONT_MM,
            fill='#c0392b',
            bold=True,
        )
    )

    bottom_safe = page_h - inset - 3.0
    red_bottom = red_baseline + SIDE_DESCENDER_PAD_MM
    if red_bottom > bottom_safe:
        raise ValueError(
            f'Bottom margin too tight for titles (red_bottom {red_bottom:.1f} > {bottom_safe:.1f} mm). '
            'Use a larger page or reduce SIDE_LABEL_FONT_MM / graveyard width.'
        )

    svg_parts.append('</svg>\n')
    with open(output_path, 'w', encoding='utf-8') as f:
        f.write(''.join(svg_parts))

    print(f'Board SVG written to: {output_path}')
    print(f'Page: {page_w}×{page_h} mm | ArUco: {aruco:.1f} mm | cell: {cell:.2f} mm (square)')
    print(f'Grid box: {grid_w:.1f}×{grid_h:.1f} mm')
    return cell, cell


def write_board_geometry_yaml(path: str, cell_mm: float, page_w: float, page_h: float) -> None:
    """Emit spacing for board_calibration.yaml (must match printed intersection spacing)."""
    lines = [
        '# Auto-generated by tools/generate_board_svg.py',
        '# Copy grid_spacing_mm into workspace/src/xiangqi_bringup/config/board_calibration.yaml',
        '# before running calibration_tool / vision so the arm matches this mat.',
        f'grid_spacing_mm: {cell_mm:.4f}',
        f'page_size_mm: [{page_w:.1f}, {page_h:.1f}]',
        '',
    ]
    with open(path, 'w', encoding='utf-8') as f:
        f.write('\n'.join(lines))


if __name__ == '__main__':
    root = os.path.dirname(__file__)
    out_a2 = os.path.join(root, '..', 'docs', 'board_mat_A2.svg')
    out_a3 = os.path.join(root, '..', 'docs', 'board_mat_A3.svg')
    geo_dir = os.path.join(root, '..', 'docs')
    c2, _ = generate_board_svg(out_a2, PAGE_W_MM_A2, PAGE_H_MM_A2)
    c3, _ = generate_board_svg(out_a3, PAGE_W_MM_A3, PAGE_H_MM_A3)
    write_board_geometry_yaml(os.path.join(geo_dir, 'board_geometry_A2.yaml'), c2, PAGE_W_MM_A2, PAGE_H_MM_A2)
    write_board_geometry_yaml(os.path.join(geo_dir, 'board_geometry_A3.yaml'), c3, PAGE_W_MM_A3, PAGE_H_MM_A3)
    print('\nA2 file-a spacing:', c2, 'mm  |  A3:', c3, 'mm')
    print('Wrote docs/board_geometry_A2.yaml and docs/board_geometry_A3.yaml')
