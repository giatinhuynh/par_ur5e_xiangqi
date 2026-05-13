#!/usr/bin/env python3
"""
Preview YOLO detections on sample images (paste into a Kaggle notebook cell, or run:

  !python tools/kaggle_preview_detections.py

Paths default to this repo's Kaggle layout; override with env vars or flags.

With no --source: samples ``--per-type`` images from each ``--subsets`` folder under
``--dataset-root`` (<type>/images/), so all board types appear in one grid.
"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

import cv2
import matplotlib.pyplot as plt
import numpy as np


def _argv_for_cli() -> list[str]:
    argv = sys.argv[1:]
    out: list[str] = []
    i = 0
    while i < len(argv):
        if argv[i] == "-f" and i + 1 < len(argv):
            i += 2
            continue
        out.append(argv[i])
        i += 1
    return out


def parse_args() -> argparse.Namespace:
    default_kaggle_root = Path(
        "/kaggle/input/datasets/hunhcgiatn/chess-dataset/chess_dataset"
    )
    default_model = Path("/kaggle/working/xiangqi_yolo/xiangqi_kaggle_v1_best.pt")
    fallback_model = Path("/kaggle/working/xiangqi_yolo/runs/xiangqi_kaggle_v1/weights/best.pt")
    default_subsets = ["Wooden", "Acrylic", "Stainless Steel", "All Types"]

    p = argparse.ArgumentParser(description="Plot YOLO detections on sample images")
    p.add_argument("--model", type=Path, default=None, help="Path to .pt weights")
    p.add_argument(
        "--source",
        type=Path,
        default=None,
        help="Single directory of images or one image (if set, ignores dataset-root / subsets)",
    )
    p.add_argument(
        "--dataset-root",
        type=Path,
        default=None,
        help="chess_dataset root with Wooden/, Acrylic/, … (used when --source is omitted)",
    )
    p.add_argument(
        "--subsets",
        nargs="+",
        default=default_subsets,
        metavar="NAME",
        help="Board-type folder names under dataset-root",
    )
    p.add_argument(
        "--per-type",
        type=int,
        default=3,
        help="Images per subset when using dataset-root (ignored with --source)",
    )
    p.add_argument("--num", type=int, default=9, help="Max images when using --source only")
    p.add_argument("--conf", type=float, default=0.25)
    p.add_argument("--imgsz", type=int, default=640)
    p.add_argument(
        "--save",
        type=Path,
        default=None,
        help="Optional PNG path to save grid (e.g. /kaggle/working/preview.png)",
    )
    args, _ = p.parse_known_args(_argv_for_cli())

    if args.model is None:
        candidates: list[Path] = []
        env_m = os.environ.get("XIANGQI_MODEL_PT")
        if env_m:
            candidates.append(Path(env_m))
        candidates.extend([default_model, fallback_model])
        for cand in candidates:
            if cand.is_file():
                args.model = cand
                break
    if args.model is None or not args.model.is_file():
        print("ERROR: set --model to your .pt file", file=sys.stderr)
        sys.exit(1)

    if args.source is None:
        env_s = os.environ.get("XIANGQI_PREVIEW_IMAGES")
        if env_s:
            args.source = Path(env_s)

    if args.source is None:
        if args.dataset_root is None:
            env_root = os.environ.get("XIANGQI_DATASET_ROOT")
            if env_root:
                args.dataset_root = Path(env_root)
            elif default_kaggle_root.is_dir():
                args.dataset_root = default_kaggle_root

        if args.dataset_root is None or not args.dataset_root.is_dir():
            print(
                "ERROR: set --source (image dir), or --dataset-root / XIANGQI_DATASET_ROOT "
                f"(e.g. {default_kaggle_root})",
                file=sys.stderr,
            )
            sys.exit(1)

    return args


def collect_images(source: Path, limit: int) -> list[Path]:
    if source.is_file():
        return [source] if source.suffix.lower() in {".jpg", ".jpeg", ".png", ".bmp", ".webp"} else []
    exts = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}
    files: list[Path] = []
    for p in sorted(source.iterdir()):
        if p.is_file() and p.suffix.lower() in exts:
            files.append(p)
            if len(files) >= limit:
                break
    return files


def collect_from_board_types(
    dataset_root: Path,
    subsets: list[str],
    per_type: int,
) -> list[tuple[Path, str]]:
    """Return (image_path, subset_label) for up to ``per_type`` images per subset."""
    pairs: list[tuple[Path, str]] = []
    for name in subsets:
        img_dir = dataset_root / name / "images"
        if not img_dir.is_dir():
            print(f"WARNING: skip missing folder {img_dir}", file=sys.stderr)
            continue
        for p in collect_images(img_dir, per_type):
            pairs.append((p, name))
    return pairs


def main() -> None:
    args = parse_args()
    try:
        from ultralytics import YOLO
    except ImportError:
        os.system(f"{sys.executable} -m pip install -q ultralytics")
        from ultralytics import YOLO

    if args.source is not None:
        paths = collect_images(args.source, args.num)
        labels: list[str | None] = [None] * len(paths)
    else:
        pairs = collect_from_board_types(args.dataset_root, args.subsets, args.per_type)
        paths = [p for p, _ in pairs]
        labels = [lab for _, lab in pairs]

    if not paths:
        raise SystemExit("No images found (check --source or --dataset-root / --subsets)")

    model = YOLO(str(args.model))
    results = model.predict(
        source=[str(p) for p in paths],
        conf=args.conf,
        imgsz=args.imgsz,
        verbose=False,
    )

    n = len(results)
    ncols = min(3, n)
    nrows = (n + ncols - 1) // ncols
    fig, axes = plt.subplots(nrows, ncols, figsize=(4.2 * ncols, 4.2 * nrows))
    axes_flat = np.atleast_1d(axes).ravel()

    for i, r in enumerate(results):
        ax = axes_flat[i]
        bgr = r.plot()
        ax.imshow(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))
        fname = Path(r.path).name[:28]
        prefix = f"{labels[i]} | " if i < len(labels) and labels[i] else ""
        ax.set_title(f"{prefix}{fname}", fontsize=8)
        ax.axis("off")
        # caption: class counts
        if r.boxes is not None and len(r.boxes):
            names = r.names
            ids = r.boxes.cls.int().tolist()
            from collections import Counter

            c = Counter(ids)
            short = ", ".join(f"{names[k]}:{v}" for k, v in sorted(c.items())[:8])
            if len(c) > 8:
                short += "…"
            ax.set_xlabel(short, fontsize=6)

    for j in range(len(results), len(axes_flat)):
        axes_flat[j].axis("off")

    if args.source is not None:
        subtitle = f"source={args.source}"
    else:
        subtitle = (
            f"dataset-root={args.dataset_root}  per-type={args.per_type}  subsets={args.subsets}"
        )
    fig.suptitle(
        f"model={args.model.name}  conf≥{args.conf}  n={n}\n{subtitle}",
        fontsize=10,
    )
    plt.tight_layout()
    if args.save:
        plt.savefig(args.save, dpi=140, bbox_inches="tight")
        print(f"Saved {args.save}")
    plt.show()


if __name__ == "__main__":
    main()
