#!/usr/bin/env python3
"""Run ArUco detection on docs/aruco_test_A4.png (or another image)."""

from __future__ import annotations

import argparse
import os
import sys

try:
    import cv2
except ImportError:
    print('ERROR: pip install opencv-contrib-python-headless', file=sys.stderr)
    sys.exit(1)

ROOT = os.path.dirname(os.path.abspath(__file__))
DEFAULT_IMG = os.path.join(ROOT, '..', 'docs', 'aruco_test_A4.png')
DEFAULT_DEBUG = os.path.join(ROOT, '..', 'docs', 'aruco_test_A4_scan.png')
REQUIRED = (0, 1, 2, 3)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--image', default=DEFAULT_IMG)
    parser.add_argument('--out', default=DEFAULT_DEBUG)
    args = parser.parse_args()

    img = cv2.imread(args.image, cv2.IMREAD_GRAYSCALE)
    if img is None:
        print(f'ERROR: cannot read {args.image}', file=sys.stderr)
        sys.exit(1)

    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    detector = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())
    corners, ids, _ = detector.detectMarkers(img)

    found = sorted(int(x) for x in ids.flatten()) if ids is not None else []
    missing = [i for i in REQUIRED if i not in found]

    debug = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    if ids is not None:
        cv2.aruco.drawDetectedMarkers(debug, corners, ids)
    cv2.imwrite(args.out, debug)

    print(f'Image: {args.image} ({img.shape[1]}×{img.shape[0]})')
    print(f'Detected: {found}')
    print(f'Missing:  {missing or "none"}')
    print(f'Debug:    {args.out}')
    sys.exit(0 if not missing else 1)


if __name__ == '__main__':
    main()
