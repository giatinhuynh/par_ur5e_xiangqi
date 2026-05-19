#!/usr/bin/env python3
"""
generate_aruco_test_a4.py

A4 test sheet with four large markers using OpenCV's official ArUco API
(cv2.aruco.generateImageMarker), same dictionary as board_detector.py (DICT_4X4_50).

Requires: opencv-contrib-python or python3-opencv (lab Docker image has this).

  python3 tools/generate_aruco_test_a4.py
  python3 tools/generate_aruco_test_a4.py --dpi 300 --max-size --verify
"""

from __future__ import annotations

import argparse
import os
import sys

try:
    import cv2
    import numpy as np
except ImportError:
    print(
        'ERROR: OpenCV with ArUco is required.\n'
        '  pip install opencv-contrib-python-headless numpy\n'
        '  or run inside the lab ROS container (python3-opencv).',
        file=sys.stderr,
    )
    sys.exit(1)

if not hasattr(cv2.aruco, 'generateImageMarker'):
    print(
        'ERROR: This OpenCV build has no cv2.aruco.generateImageMarker.\n'
        '       Install opencv-contrib-python-headless (not opencv-python only).',
        file=sys.stderr,
    )
    sys.exit(1)

PAGE_W_MM = 210.0
PAGE_H_MM = 297.0
MARGIN_MM = 15.0
GAP_MM = 12.0
DEFAULT_ARUCO_MM = 80.0
DEFAULT_DPI = 300
MARKER_SIDE_PX = 200  # resolution passed to generateImageMarker before NEAREST resize
REQUIRED_IDS = (0, 1, 2, 3)
ARUCO_DICT_ID = cv2.aruco.DICT_4X4_50


def mm_to_px(mm: float, dpi: float) -> int:
    return int(round(mm * dpi / 25.4))


def max_aruco_mm(margin: float = MARGIN_MM, gap: float = GAP_MM) -> float:
    by_w = (PAGE_W_MM - 2.0 * margin - gap) / 2.0
    by_h = (PAGE_H_MM - 2.0 * margin - gap) / 2.0
    return max(20.0, min(by_w, by_h))


def layout_2x2(aruco_mm: float, gap: float = GAP_MM) -> list[tuple[float, float, int]]:
    grid_w = 2.0 * aruco_mm + gap
    x0 = (PAGE_W_MM - grid_w) / 2.0
    y0 = (PAGE_H_MM - grid_w) / 2.0
    step = aruco_mm + gap
    return [
        (x0, y0, 0),
        (x0 + step, y0, 1),
        (x0, y0 + step, 2),
        (x0 + step, y0 + step, 3),
    ]


def build_sheet(
    aruco_mm: float,
    dpi: float,
    margin_mm: float = MARGIN_MM,
    gap_mm: float = GAP_MM,
) -> np.ndarray:
    """Grayscale A4 image (white background, official marker bitmaps)."""
    page_w = mm_to_px(PAGE_W_MM, dpi)
    page_h = mm_to_px(PAGE_H_MM, dpi)
    canvas = np.full((page_h, page_w), 255, dtype=np.uint8)

    dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICT_ID)
    marker_px = mm_to_px(aruco_mm, dpi)

    for mx_mm, my_mm, marker_id in layout_2x2(aruco_mm, gap_mm):
        marker = cv2.aruco.generateImageMarker(dictionary, marker_id, MARKER_SIDE_PX)
        scaled = cv2.resize(marker, (marker_px, marker_px), interpolation=cv2.INTER_NEAREST)
        x0 = mm_to_px(mx_mm, dpi)
        y0 = mm_to_px(my_mm, dpi)
        canvas[y0 : y0 + marker_px, x0 : x0 + marker_px] = scaled

    return canvas


def verify_detection(image: np.ndarray) -> tuple[bool, list[int]]:
    dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICT_ID)
    detector = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())
    _corners, ids, _ = detector.detectMarkers(image)
    found = sorted(int(x) for x in ids.flatten()) if ids is not None else []
    missing = [i for i in REQUIRED_IDS if i not in found]
    return len(missing) == 0, found


def write_debug_overlay(image: np.ndarray, out_path: str) -> None:
    dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICT_ID)
    detector = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())
    corners, ids, _ = detector.detectMarkers(image)
    debug = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    if ids is not None:
        cv2.aruco.drawDetectedMarkers(debug, corners, ids)
    cv2.imwrite(out_path, debug)


def generate_aruco_test_a4(
    out_path: str,
    aruco_mm: float = DEFAULT_ARUCO_MM,
    dpi: float = DEFAULT_DPI,
    margin_mm: float = MARGIN_MM,
    gap_mm: float = GAP_MM,
    do_verify: bool = False,
    debug_path: str | None = None,
) -> float:
    cap = max_aruco_mm(margin_mm, gap_mm)
    if aruco_mm > cap:
        raise ValueError(f'--size-mm {aruco_mm:.1f} too large for A4 (max {cap:.1f} mm).')

    sheet = build_sheet(aruco_mm, dpi, margin_mm, gap_mm)

    os.makedirs(os.path.dirname(os.path.abspath(out_path)) or '.', exist_ok=True)
    if not cv2.imwrite(out_path, sheet):
        raise RuntimeError(f'Failed to write {out_path}')

    if do_verify:
        ok, found = verify_detection(sheet)
        if debug_path:
            write_debug_overlay(sheet, debug_path)
        if not ok:
            missing = [i for i in REQUIRED_IDS if i not in found]
            raise RuntimeError(
                f'OpenCV could not detect all markers after generation. '
                f'Found {found}, missing {missing}. '
                f'Try another OpenCV version (opencv-contrib 4.8–4.10).'
            )

    return aruco_mm


def main() -> None:
    root = os.path.dirname(os.path.abspath(__file__))
    default_out = os.path.join(root, '..', 'docs', 'aruco_test_A4.png')
    default_debug = os.path.join(root, '..', 'docs', 'aruco_test_A4_detected.png')

    parser = argparse.ArgumentParser(
        description='A4 ArUco test sheet via OpenCV generateImageMarker (DICT_4X4_50).',
    )
    parser.add_argument('--out', default=default_out, help='Output PNG path')
    parser.add_argument('--dpi', type=float, default=DEFAULT_DPI, help='Print resolution')
    parser.add_argument('--size-mm', type=float, default=DEFAULT_ARUCO_MM)
    parser.add_argument('--max-size', action='store_true', help='Largest markers that fit')
    parser.add_argument(
        '--verify',
        action='store_true',
        help='Run ArUco detection on the PNG after writing (fails if any ID missing)',
    )
    parser.add_argument(
        '--debug',
        nargs='?',
        const=default_debug,
        default=None,
        help='Write detection overlay PNG (default: docs/aruco_test_A4_detected.png)',
    )
    args = parser.parse_args()

    size = max_aruco_mm() if args.max_size else args.size_mm
    debug_path = args.debug if args.verify or args.debug is not None else None
    if args.verify and args.debug is None:
        debug_path = default_debug

    used = generate_aruco_test_a4(
        args.out,
        aruco_mm=size,
        dpi=args.dpi,
        do_verify=args.verify,
        debug_path=debug_path,
    )

    print(f'OpenCV {cv2.__version__} | DICT_4X4_50 | generateImageMarker')
    print(f'Wrote {os.path.abspath(args.out)}')
    print(f'  {used:.0f} mm markers @ {args.dpi:.0f} DPI — print at 100% / actual size')
    if args.verify:
        print('  Verify: PASS (all IDs 0–3 detected on generated image)')
    if debug_path:
        print(f'  Debug:  {os.path.abspath(debug_path)}')


if __name__ == '__main__':
    main()
