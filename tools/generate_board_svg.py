#!/usr/bin/env python3
"""
generate_board_svg.py
Generates a print-ready SVG for a Xiangqi robot board mat.

Layout rules (fixes overlap / corner issues):
  - ArUco markers at **sheet corners** (dataset-style); IDs 0–3 match board_detector.py.
    Use ARUCO_PAGE_INSET_MM so squares stay slightly inside the paper edge on the table.
  - **Square** grid cells: cell = min(file_pitch, rank_pitch), grid centred
    in the interior rectangle (standard look; homography still valid).
  - Spec / calibration line in the **top** margin band.
  - **a–i** sit directly under the bottom grid line with the same clearance as **0–9** left of gx(0).
  - **红方** sits **below** the file row; **黑方** sits **above** the top grid with extra margin (CJK/serif).

Output: docs/board_mat_A2.svg, docs/board_mat_A3.svg,
         docs/board_mat_2xA3_left.svg, docs/board_mat_2xA3_right.svg
         (two portrait A3 → A2-sized mat),
         docs/board_mat_4xA3_{TL,TR,BL,BR}.svg
         (four portrait A3 in 2×2 → 594×840 mm tall mat, wide capture strips)
"""

import os
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
# Portrait A3 (print with long edge vertical): 297 × 420 mm per sheet.
PAGE_W_MM_A3_PORTRAIT = PAGE_H_MM_A3
PAGE_H_MM_A3_PORTRAIT = PAGE_W_MM_A3
# Two portrait A3 sheets taped along the 420 mm height → same footprint as A2.
PAGE_W_MM_2XA3 = PAGE_W_MM_A2
PAGE_H_MM_2XA3 = PAGE_H_MM_A2
# Four portrait A3 sheets in 2×2 → 594 × 840 mm (board runs vertically).
PAGE_W_MM_4XA3 = PAGE_W_MM_A3_PORTRAIT * 2.0
PAGE_H_MM_4XA3 = PAGE_H_MM_A3_PORTRAIT * 2.0

# Corner ArUcos: bleed inset (mm) used for inner layout band only.
CORNER_INSET_MM = 1.5

# Minimum distance from **page edge** to **marker square outer edge** (mm).
# Use this so markers stay on the physical table when the sheet is trimmed or smaller than the table lip.
ARUCO_PAGE_INSET_MM = 8.0

# Gap between ArUco block and inner content (graveyards + board) — stops text/grid touching markers
ARUCO_BAND_GAP_MM = 10.0

GRAVEYARD_W_MM = 56.0
GRAVEYARD_GAP_MM = 5.0

# 4×A3 (594×840 mm portrait mat): capture strips above/below the grid (BLACK top,
# RED bottom); large ArUco; strips stay between side corner markers vertically.
LAYOUT_4XA3 = {
    'graveyard_w_mm': 85.0,
    'graveyard_placement': 'ends',
    'aruco_mm': 38.0,
    'aruco_band_gap_mm': 6.0,
    'corner_inset_mm': 1.0,
    'graveyard_clip_to_aruco_band': True,
}
# Extend each graveyard strip above/below the grid (mm per side), within inner band only.
GRAVEYARD_VERTICAL_EXTEND_MM = 22.0
# Legacy bleed toward page edge (disabled when graveyard_clip_to_aruco_band is set).
GRAVEYARD_VERTICAL_BLEED_MM = 10.0

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


def compute_layout(page_w: float, page_h: float, layout: dict | None = None):
    """
    Returns dict with geometry helpers.
    Origin top-left; rank 0 = bottom (robot/red), rank 9 = top (human/black).

    layout: optional overrides — graveyard_w_mm, graveyard_placement ('sides'|'ends'),
            aruco_mm, aruco_band_gap_mm, corner_inset_mm, graveyard_clip_to_aruco_band.
    """
    opts = layout or {}
    graveyard_w = float(opts.get('graveyard_w_mm', GRAVEYARD_W_MM))
    placement = str(opts.get('graveyard_placement', 'sides'))
    aruco = float(opts['aruco_mm']) if 'aruco_mm' in opts else aruco_size_for_page(page_w, page_h)
    inset = float(opts.get('corner_inset_mm', CORNER_INSET_MM))
    band = float(opts.get('aruco_band_gap_mm', ARUCO_BAND_GAP_MM))
    clip_graveyards = bool(opts.get('graveyard_clip_to_aruco_band', False))

    # Match build_board_svg_parts: markers sit at max(corner inset, page inset).
    aruco_inset = max(inset, ARUCO_PAGE_INSET_MM)

    # Inner rectangle between corner ArUcos (graveyards + grid stay inside).
    inner_left = aruco_inset + aruco + band
    inner_right = page_w - aruco_inset - aruco - band
    inner_top = aruco_inset + aruco + band
    inner_bottom = page_h - aruco_inset - aruco - band

    iw = inner_right - inner_left
    ih = inner_bottom - inner_top
    if iw < 120 or ih < 100:
        raise ValueError(
            f'Page {page_w}×{page_h} mm too small after ArUco bands (inner {iw:.0f}×{ih:.0f} mm).'
        )

    gy_left = inner_left
    gy_right = inner_right

    if placement == 'ends':
        # Horizontal strips: BLACK captured above grid, RED below (portrait-friendly).
        strip_d = graveyard_w
        bl = inner_left
        br = inner_right
        bw = br - bl
        board_top = inner_top + strip_d + GRAVEYARD_GAP_MM
        board_bottom = inner_bottom - strip_d - GRAVEYARD_GAP_MM
        bh = board_bottom - board_top
        if bw < 60 or bh < 60:
            raise ValueError('Board interior too small; reduce graveyard strip depth or print larger.')
        cell = min(bw / (FILES - 1), bh / (RANKS - 1))
        grid_w = cell * (FILES - 1)
        grid_h = cell * (RANKS - 1)
        bl_grid = bl + (bw - grid_w) / 2.0
        v_lo = board_top + (bh - grid_h) / 2.0
        bb_grid = v_lo + grid_h
        grave_top = inner_top
        grave_h = strip_d
        grave_strip_w = iw
        grave_bottom_y = inner_bottom - strip_d
    else:
        bl = inner_left + graveyard_w + GRAVEYARD_GAP_MM
        br = inner_right - graveyard_w - GRAVEYARD_GAP_MM
        bw = br - bl
        bh = inner_bottom - inner_top
        if bw < 60 or bh < 60:
            raise ValueError('Board interior too small; reduce graveyard width or print larger.')
        cell = min(bw / (FILES - 1), bh / (RANKS - 1))
        grid_w = cell * (FILES - 1)
        grid_h = cell * (RANKS - 1)
        bl_grid = bl + (bw - grid_w) / 2.0
        v_lo = inner_top + (bh - grid_h) / 2.0
        bb_grid = v_lo + grid_h
        gy_right = br + GRAVEYARD_GAP_MM
        grave_strip_w = graveyard_w
        grave_bottom_y = None
        ext = GRAVEYARD_VERTICAL_EXTEND_MM
        if clip_graveyards:
            grave_top = max(inner_top, v_lo - ext)
            grave_bottom = min(inner_bottom, v_lo + grid_h + ext)
        else:
            bleed = GRAVEYARD_VERTICAL_BLEED_MM
            grave_top = max(inner_top - bleed, v_lo - ext)
            grave_bottom = min(inner_bottom + bleed, v_lo + grid_h + ext)
            grave_top = min(grave_top, v_lo)
            grave_bottom = max(grave_bottom, v_lo + grid_h)
            edge = 2.0
            grave_top = max(edge, grave_top)
            grave_bottom = min(page_h - edge, grave_bottom)
        grave_top = max(grave_top, inner_top)
        grave_bottom = min(grave_bottom, inner_bottom)
        grave_h = grave_bottom - grave_top
        if grave_h < grid_h:
            grave_top = max(inner_top, v_lo)
            grave_h = min(grid_h, inner_bottom - grave_top)

    def gx(file_idx: int) -> float:
        return bl_grid + file_idx * cell

    def gy(rank_idx: int) -> float:
        return bb_grid - rank_idx * cell

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
        'grave_top': grave_top,
        'grave_h': grave_h,
        'graveyard_w': graveyard_w,
        'graveyard_placement': placement,
        'grave_strip_w': grave_strip_w if placement == 'ends' else graveyard_w,
        'grave_bottom_y': grave_bottom_y if placement == 'ends' else None,
        'aruco_inset': aruco_inset,
    }


def tile_seam_marks(
    tile_w: float,
    tile_h: float,
    seam_edges: frozenset[str],
    tile_label: str,
) -> str:
    """
    Registration marks on tile edges that join a neighbour.
    seam_edges: subset of 'left', 'right', 'top', 'bottom' (this tile's outward seams).
    """
    tick = 8.0
    gap = 3.0
    stroke = '#c0392b'
    sw = 0.45
    parts: list[str] = []

    if 'right' in seam_edges:
        sx = tile_w - gap
        for y in (tile_h * 0.22, tile_h * 0.5, tile_h * 0.78):
            parts.append(line(sx - tick, y, sx, y, stroke=stroke, width=sw))
            parts.append(line(sx, y - tick * 0.35, sx, y + tick * 0.35, stroke=stroke, width=sw))
        parts.append(text(tile_w - 18.0, tile_h - 12.0, 'TAPE →', size=2.5, fill='#c0392b'))
    if 'left' in seam_edges:
        sx = gap
        for y in (tile_h * 0.22, tile_h * 0.5, tile_h * 0.78):
            parts.append(line(sx, y, sx + tick, y, stroke=stroke, width=sw))
            parts.append(line(sx, y - tick * 0.35, sx, y + tick * 0.35, stroke=stroke, width=sw))
        parts.append(text(18.0, tile_h - 12.0, '← TAPE', size=2.5, fill='#c0392b'))
    if 'bottom' in seam_edges:
        sy = tile_h - gap
        for x in (tile_w * 0.22, tile_w * 0.5, tile_w * 0.78):
            parts.append(line(x, sy - tick, x, sy, stroke=stroke, width=sw))
            parts.append(line(x - tick * 0.35, sy, x + tick * 0.35, sy, stroke=stroke, width=sw))
        parts.append(text(tile_w * 0.5, tile_h - 12.0, 'TAPE ↓', size=2.5, fill='#c0392b'))
    if 'top' in seam_edges:
        sy = gap
        for x in (tile_w * 0.22, tile_w * 0.5, tile_w * 0.78):
            parts.append(line(x, sy, x, sy + tick, stroke=stroke, width=sw))
            parts.append(line(x - tick * 0.35, sy, x + tick * 0.35, sy, stroke=stroke, width=sw))
        parts.append(text(tile_w * 0.5, 14.0, '↑ TAPE', size=2.5, fill='#c0392b'))

    parts.append(
        text(tile_w - 10.0, 10.0, tile_label, size=2.4, anchor='end', fill='#888')
    )
    return ''.join(parts)


def build_board_svg_parts(
    page_w: float,
    page_h: float,
    *,
    aruco_ids: set[int] | None = None,
    sheet_label: str | None = None,
    layout: dict | None = None,
) -> list[str]:
    """Return SVG fragment strings (no outer <svg> wrapper)."""
    if aruco_ids is None:
        aruco_ids = {0, 1, 2, 3}

    L = compute_layout(page_w, page_h, layout)
    aruco = L['aruco']
    inset = L['inset']
    gx, gy = L['gx'], L['gy']
    bl_grid, br = L['bl_grid'], L['br']
    bb_grid = L['bb_grid']
    cell = L['cell']
    grid_w, grid_h = L['grid_w'], L['grid_h']
    gy_left, gy_right = L['gy_left'], L['gy_right']
    gt, gh = L['grave_top'], L['grave_h']
    graveyard_w = L['graveyard_w']
    grave_placement = L['graveyard_placement']
    grave_strip_w = L['grave_strip_w']
    grave_bottom_y = L['grave_bottom_y']
    inner_left = L['inner_left']

    svg_parts: list[str] = [
        rect(0, 0, page_w, page_h, fill='#f5e6c8', stroke='none'),
    ]

    # --- ArUco: paper / sheet corners (dataset-style); IDs match board_detector.py ---
    aruco_ul_inset = max(float(inset), float(ARUCO_PAGE_INSET_MM))
    aruco_positions_ul = [
        (aruco_ul_inset, aruco_ul_inset, 0),
        (page_w - aruco_ul_inset - aruco, aruco_ul_inset, 1),
        (page_w - aruco_ul_inset - aruco, page_h - aruco_ul_inset - aruco, 2),
        (aruco_ul_inset, page_h - aruco_ul_inset - aruco, 3),
    ]

    for mx, my, mid in aruco_positions_ul:
        if mid not in aruco_ids:
            continue
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

    # --- Graveyards: ends = top BLACK / bottom RED strips; sides = left / right strips ---
    if grave_placement == 'ends':
        cx = inner_left + grave_strip_w / 2.0
        svg_parts.append(
            rect(inner_left, gt, grave_strip_w, gh, fill='#e8d5a0', stroke='#888', width=0.4)
        )
        svg_parts.append(
            text(cx, gt + gh / 2 - 3.5, 'BLACK', size=4.2, fill='#222', baseline='middle')
        )
        svg_parts.append(
            text(cx, gt + gh / 2 + 4.5, 'CAPTURED', size=3.5, fill='#444', baseline='middle')
        )
        svg_parts.append(
            rect(inner_left, grave_bottom_y, grave_strip_w, gh, fill='#e8d5a0', stroke='#888', width=0.4)
        )
        svg_parts.append(
            text(cx, grave_bottom_y + gh / 2 - 3.5, 'RED', size=4.2, fill='#8b0000', baseline='middle')
        )
        svg_parts.append(
            text(cx, grave_bottom_y + gh / 2 + 4.5, 'CAPTURED', size=3.5, fill='#444', baseline='middle')
        )
    else:
        svg_parts.append(
            rect(gy_left, gt, graveyard_w, gh, fill='#e8d5a0', stroke='#888', width=0.4)
        )
        svg_parts.append(
            text(gy_left + graveyard_w / 2, gt + gh / 2 - 3.5, 'BLACK', size=4.2, fill='#222', baseline='middle')
        )
        svg_parts.append(
            text(gy_left + graveyard_w / 2, gt + gh / 2 + 4.5, 'CAPTURED', size=3.5, fill='#444', baseline='middle')
        )
        svg_parts.append(
            rect(gy_right, gt, graveyard_w, gh, fill='#e8d5a0', stroke='#888', width=0.4)
        )
        svg_parts.append(
            text(gy_right + graveyard_w / 2, gt + gh / 2 - 3.5, 'RED', size=4.2, fill='#8b0000', baseline='middle')
        )
        svg_parts.append(
            text(gy_right + graveyard_w / 2, gt + gh / 2 + 4.5, 'CAPTURED', size=3.5, fill='#444', baseline='middle')
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
        inner_left + (2.0 if grave_placement == 'ends' else graveyard_w + 2.0),
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
    spec = (
        f'{page_w:.0f}×{page_h:.0f} mm | {FILES}×{RANKS} | cell {cell:.1f} mm sq | '
        f'ArUco 4×4_50 0–3 | {aruco:.0f} mm'
    )
    if sheet_label:
        spec = f'{sheet_label} | {spec}'
    svg_parts.append(
        text(
            page_w / 2,
            gap_mid_y,
            spec,
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

    return svg_parts, cell


def write_svg_file(output_path: str, parts: list[str], page_w: float, page_h: float) -> None:
    with open(output_path, 'w', encoding='utf-8') as f:
        f.write('<?xml version="1.0" encoding="UTF-8"?>\n')
        f.write(
            f'<svg xmlns="http://www.w3.org/2000/svg" '
            f'width="{page_w}mm" height="{page_h}mm" viewBox="0 0 {page_w} {page_h}">\n'
        )
        f.write(''.join(parts))
        f.write('</svg>\n')


def write_mat_tile(
    output_path: str,
    parts: list[str],
    *,
    tile_x0: float,
    tile_y0: float,
    tile_w: float,
    tile_h: float,
    seam_edges: frozenset[str],
    tile_label: str,
) -> None:
    """Write one print tile clipped from a larger combined mat."""
    clip_id = 'tile_clip'
    extra = tile_seam_marks(tile_w, tile_h, seam_edges, tile_label)
    with open(output_path, 'w', encoding='utf-8') as f:
        f.write('<?xml version="1.0" encoding="UTF-8"?>\n')
        f.write(
            f'<svg xmlns="http://www.w3.org/2000/svg" '
            f'width="{tile_w}mm" height="{tile_h}mm" viewBox="0 0 {tile_w} {tile_h}">\n'
        )
        f.write(
            f'  <defs><clipPath id="{clip_id}">'
            f'<rect x="0" y="0" width="{tile_w:.2f}" height="{tile_h:.2f}"/></clipPath></defs>\n'
        )
        f.write(f'  <g clip-path="url(#{clip_id})">\n')
        f.write(f'    <g transform="translate({-tile_x0:.2f},{-tile_y0:.2f})">\n')
        f.write(''.join(parts))
        f.write('    </g>\n')
        f.write('  </g>\n')
        f.write(extra)
        f.write('</svg>\n')


def generate_board_svg(output_path: str, page_w: float, page_h: float):
    parts, cell = build_board_svg_parts(page_w, page_h)
    write_svg_file(output_path, parts, page_w, page_h)
    print(f'Board SVG written to: {output_path}')
    print(f'Page: {page_w}×{page_h} mm | cell: {cell:.2f} mm (square)')
    return cell, cell


def generate_board_2xa3_tiles(out_left: str, out_right: str, geo_path: str) -> float:
    """Full mat = A2 (594×420 mm); each tile is portrait A3 (297×420 mm)."""
    parts, cell = build_board_svg_parts(
        PAGE_W_MM_2XA3,
        PAGE_H_MM_2XA3,
        aruco_ids={0, 1, 2, 3},
        sheet_label='2×A3 portrait → A2 size',
    )
    tw, th = PAGE_W_MM_A3_PORTRAIT, PAGE_H_MM_A3_PORTRAIT
    write_mat_tile(
        out_left,
        parts,
        tile_x0=0.0,
        tile_y0=0.0,
        tile_w=tw,
        tile_h=th,
        seam_edges=frozenset({'right'}),
        tile_label='2×A3 · L',
    )
    write_mat_tile(
        out_right,
        parts,
        tile_x0=tw,
        tile_y0=0.0,
        tile_w=tw,
        tile_h=th,
        seam_edges=frozenset({'left'}),
        tile_label='2×A3 · R',
    )
    write_board_geometry_yaml(geo_path, cell, PAGE_W_MM_2XA3, PAGE_H_MM_2XA3)
    print(f'2×A3 tiles written: {out_left}')
    print(f'                  {out_right}')
    print(
        f'Joined mat: {PAGE_W_MM_2XA3:.0f}×{PAGE_H_MM_2XA3:.0f} mm (A2) | '
        f'each tile: {PAGE_W_MM_A3_PORTRAIT:.0f}×{PAGE_H_MM_A3_PORTRAIT:.0f} mm portrait | '
        f'cell: {cell:.2f} mm'
    )
    return cell


def generate_board_4xa3_tiles(out_paths: dict[str, str], geo_path: str) -> float:
    """
    Full mat 594×840 mm (portrait) from four portrait A3 sheets (2×2).

    Print each tile A3 portrait (297×420 mm), 100% scale. Tape centre cross:
      TL—TR, BL—BR, then join top row to bottom row.
    """
    parts, cell = build_board_svg_parts(
        PAGE_W_MM_4XA3,
        PAGE_H_MM_4XA3,
        aruco_ids={0, 1, 2, 3},
        sheet_label='4×A3 portrait 2×2',
        layout=LAYOUT_4XA3,
    )
    tw, th = PAGE_W_MM_A3_PORTRAIT, PAGE_H_MM_A3_PORTRAIT
    tiles = {
        'TL': (0.0, 0.0, frozenset({'right', 'bottom'}), '4×A3 · TL'),
        'TR': (tw, 0.0, frozenset({'left', 'bottom'}), '4×A3 · TR'),
        'BL': (0.0, th, frozenset({'right', 'top'}), '4×A3 · BL'),
        'BR': (tw, th, frozenset({'left', 'top'}), '4×A3 · BR'),
    }
    for key, (x0, y0, edges, label) in tiles.items():
        write_mat_tile(
            out_paths[key],
            parts,
            tile_x0=x0,
            tile_y0=y0,
            tile_w=tw,
            tile_h=th,
            seam_edges=edges,
            tile_label=label,
        )
    write_board_geometry_yaml(geo_path, cell, PAGE_W_MM_4XA3, PAGE_H_MM_4XA3)
    print('4×A3 tiles written:')
    for key in ('TL', 'TR', 'BL', 'BR'):
        print(f'  {key}: {out_paths[key]}')
    print(
        f'Joined mat: {PAGE_W_MM_4XA3:.0f}×{PAGE_H_MM_4XA3:.0f} mm | '
        f'each tile: {tw:.0f}×{th:.0f} mm portrait | cell: {cell:.2f} mm'
    )
    return cell


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
    docs = os.path.join(root, '..', 'docs')
    out_a2 = os.path.join(docs, 'board_mat_A2.svg')
    out_a3 = os.path.join(docs, 'board_mat_A3.svg')
    out_2xa3_l = os.path.join(docs, 'board_mat_2xA3_left.svg')
    out_2xa3_r = os.path.join(docs, 'board_mat_2xA3_right.svg')
    out_4xa3 = {
        k: os.path.join(docs, f'board_mat_4xA3_{k}.svg')
        for k in ('TL', 'TR', 'BL', 'BR')
    }
    c2, _ = generate_board_svg(out_a2, PAGE_W_MM_A2, PAGE_H_MM_A2)
    c3, _ = generate_board_svg(out_a3, PAGE_W_MM_A3, PAGE_H_MM_A3)
    c2x = generate_board_2xa3_tiles(
        out_2xa3_l,
        out_2xa3_r,
        os.path.join(docs, 'board_geometry_2xA3.yaml'),
    )
    c4x = generate_board_4xa3_tiles(
        out_4xa3,
        os.path.join(docs, 'board_geometry_4xA3.yaml'),
    )
    write_board_geometry_yaml(os.path.join(docs, 'board_geometry_A2.yaml'), c2, PAGE_W_MM_A2, PAGE_H_MM_A2)
    write_board_geometry_yaml(os.path.join(docs, 'board_geometry_A3.yaml'), c3, PAGE_W_MM_A3, PAGE_H_MM_A3)
    print('\nA2:', c2, 'mm  |  A3:', c3, 'mm  |  2×A3:', c2x, 'mm  |  4×A3:', c4x, 'mm')
    print('Wrote board_geometry_*.yaml (A2, A3, 2xA3, 4xA3)')
