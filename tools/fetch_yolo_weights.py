#!/usr/bin/env python3
"""Download YOLO .pt weights to a destination path (for vision_node models/)."""

from __future__ import annotations

import argparse
import os
import urllib.request


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--url', required=True, help='HTTP(S) URL to the .pt file')
    p.add_argument(
        '--dest',
        required=True,
        help='Destination file path (parent dirs are created)',
    )
    args = p.parse_args()
    dest = os.path.abspath(args.dest)
    parent = os.path.dirname(dest)
    if parent:
        os.makedirs(parent, exist_ok=True)
    print(f'Downloading {args.url!r} -> {dest!r}')
    urllib.request.urlretrieve(args.url, dest)
    size = os.path.getsize(dest)
    print(f'OK ({size} bytes)')


if __name__ == '__main__':
    main()
