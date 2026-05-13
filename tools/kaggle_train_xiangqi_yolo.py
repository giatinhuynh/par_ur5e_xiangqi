#!/usr/bin/env python3
"""
One-shot YOLOv8 training for xiangqi piece detection (Kaggle or local).

- Stages images/labels from chess_dataset-style folders (Wooden, Acrylic, Stainless Steel).
- Remaps label class IDs to match xiangqi_vision/piece_detector.py (14 classes).

Kaggle usage:
  1. Add the chess-dataset as notebook Input (path baked in below).
  2. Run (no --input needed if the dataset mounts at the default path):
       !python tools/kaggle_train_xiangqi_yolo.py
     Override with: --input /kaggle/input/.../chess_dataset

Local usage:
  python tools/kaggle_train_xiangqi_yolo.py \\
    --input /path/to/par_ur5e_xiangqi/chess_dataset \\
    --work ./kaggle_work

Output: best.pt copied to work dir root as {run_name}_best.pt; logs and PNG debug plots.

Colab / Jupyter: kernel flags such as ``-f .../kernel-....json`` are stripped automatically so
``%run tools/kaggle_train_xiangqi_yolo.py`` works without errors.
"""

from __future__ import annotations

import argparse
import logging
import os
import random
import shutil
import sys
from pathlib import Path

# Vision-aligned class names (order = class id 0..13) — must match piece_detector.py
CLASS_NAMES = [
    "red_general",
    "red_advisor",
    "red_elephant",
    "red_horse",
    "red_chariot",
    "red_cannon",
    "red_soldier",
    "black_general",
    "black_advisor",
    "black_elephant",
    "black_horse",
    "black_chariot",
    "black_cannon",
    "black_soldier",
]

# Dataset label id -> vision id (aligned with VOC names in chess_dataset)
REMAP = {
    8: 0,  # red_shuai -> red_general
    7: 1,  # red_shi -> red_advisor
    2: 2,  # red_xiang -> red_elephant
    5: 3,  # red_ma -> red_horse
    0: 4,  # red_ju -> red_chariot
    4: 5,  # red_pao -> red_cannon
    1: 6,  # red_bing -> red_soldier
    12: 7,  # black_jiang -> black_general
    11: 8,  # black_shi -> black_advisor
    10: 9,  # black_xiang -> black_elephant
    6: 10,  # black_ma -> black_horse
    13: 11,  # black_ju -> black_chariot
    9: 12,  # black_pao -> black_cannon
    3: 13,  # black_zu -> black_soldier
}

IMG_EXTS = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}

# Kaggle: default dataset root when attached as Input (override with --input or XIANGQI_DATASET_ROOT).
DEFAULT_KAGGLE_CHESS_DATASET = Path(
    "/kaggle/input/datasets/hunhcgiatn/chess-dataset/chess_dataset"
)


def _argv_for_cli() -> list[str]:
    """Drop flags Jupyter / Colab / IPython inject into sys.argv (e.g. -f kernel-....json)."""
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
    p = argparse.ArgumentParser(description="Train YOLOv8 xiangqi detector (Kaggle/local)")
    p.add_argument(
        "--input",
        type=Path,
        default=None,
        help=(
            "Root containing Wooden/, Acrylic/, Stainless Steel/. "
            "If omitted: XIANGQI_DATASET_ROOT, else this path if it exists: "
            f"{DEFAULT_KAGGLE_CHESS_DATASET}"
        ),
    )
    p.add_argument(
        "--work",
        type=Path,
        default=Path(os.environ.get("KAGGLE_WORKING_DIR", "/kaggle/working")) / "xiangqi_yolo",
        help="Working directory for staged dataset, runs, plots",
    )
    p.add_argument(
        "--subsets",
        nargs="+",
        default=["Wooden", "Acrylic", "Stainless Steel"],
        help='Material folders to include (use "All Types" alone if no per-material folders)',
    )
    p.add_argument("--run-name", default="xiangqi_kaggle_v1")
    p.add_argument("--val-fraction", type=float, default=0.15)
    p.add_argument("--seed", type=int, default=42)
    p.add_argument("--model", default="yolov8n.pt")
    p.add_argument("--epochs", type=int, default=100)
    p.add_argument("--imgsz", type=int, default=640)
    p.add_argument("--batch", type=int, default=16)
    p.add_argument("--patience", type=int, default=25)
    p.add_argument("--workers", type=int, default=2)
    p.add_argument("--skip-train", action="store_true", help="Only stage data + viz (no training)")
    p.add_argument("--skip-plots", action="store_true", help="Skip matplotlib debug figures")
    args, unknown = p.parse_known_args(_argv_for_cli())
    if unknown:
        print(f"WARNING: ignoring unrecognized arguments: {unknown!r}", file=sys.stderr)
    return args


def remap_label_text(text: str, log: logging.Logger) -> str:
    out_lines = []
    for line in text.strip().splitlines():
        line = line.strip()
        if not line:
            continue
        parts = line.split()
        old_id = int(parts[0])
        new_id = REMAP.get(old_id)
        if new_id is None:
            log.warning("Unknown class id %s — line skipped: %s", old_id, line[:80])
            continue
        out_lines.append(f"{new_id} " + " ".join(parts[1:]))
    return "\n".join(out_lines) + ("\n" if out_lines else "")


def collect_images(input_root: Path, subsets: list[str], log: logging.Logger) -> list[Path]:
    all_images: list[Path] = []
    for sub in subsets:
        img_dir = input_root / sub / "images"
        if not img_dir.is_dir():
            log.warning("Missing folder (skipped): %s", img_dir)
            continue
        for f in img_dir.iterdir():
            if f.suffix.lower() not in IMG_EXTS:
                continue
            lbl = input_root / sub / "labels" / (f.stem + ".txt")
            if lbl.is_file():
                all_images.append(f)
            else:
                log.warning("No label for %s/%s", sub, f.name)
    return all_images


def stage_dataset(
    input_root: Path,
    train_root: Path,
    images: list[Path],
    val_fraction: float,
    seed: int,
    log: logging.Logger,
) -> tuple[list[Path], list[Path]]:
    random.seed(seed)
    random.shuffle(images)
    n_val = max(1, int(len(images) * val_fraction))
    val_set = set(images[:n_val])
    train_set = [p for p in images if p not in val_set]

    for split in ("train", "val"):
        (train_root / split / "images").mkdir(parents=True, exist_ok=True)
        (train_root / split / "labels").mkdir(parents=True, exist_ok=True)

    def stage(src_img: Path, split: str) -> None:
        sub = src_img.parent.parent.name
        src_lbl = input_root / sub / "labels" / (src_img.stem + ".txt")
        raw = src_lbl.read_text(encoding="utf-8", errors="ignore")
        remapped = remap_label_text(raw, log)
        if not remapped.strip():
            log.warning("Empty label after remap for %s — skipped", src_img.name)
            return
        dst_img = train_root / split / "images" / f"{sub}__{src_img.name}"
        dst_lbl = train_root / split / "labels" / (dst_img.stem + ".txt")
        shutil.copy2(src_img, dst_img)
        dst_lbl.write_text(remapped, encoding="utf-8")

    for p in train_set:
        stage(p, "train")
    for p in val_set:
        stage(p, "val")

    log.info("Staged train: %d | val: %d", len(train_set), len(val_set))
    return train_set, list(val_set)


def plot_staged_samples(train_root: Path, work: Path, log: logging.Logger) -> None:
    import cv2
    import matplotlib.pyplot as plt

    def yolo_plot_bgr(img_bgr, label_path: Path):
        h, w = img_bgr.shape[:2]
        tl = img_bgr.copy()
        if not label_path.is_file():
            return tl
        for line in label_path.read_text(encoding="utf-8").strip().splitlines():
            parts = line.split()
            if len(parts) < 5:
                continue
            cid = int(parts[0])
            cx, cy, bw, bh = map(float, parts[1:5])
            cx *= w
            cy *= h
            bw *= w
            bh *= h
            x1 = int(cx - bw / 2)
            y1 = int(cy - bh / 2)
            x2 = int(cx + bw / 2)
            y2 = int(cy + bh / 2)
            col = (0, 255, 0) if cid < 7 else (0, 128, 255)
            cv2.rectangle(tl, (x1, y1), (x2, y2), col, 2)
            name = CLASS_NAMES[cid] if 0 <= cid < len(CLASS_NAMES) else str(cid)
            cv2.putText(
                tl,
                name,
                (x1, max(15, y1 - 4)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.35,
                col,
                1,
                cv2.LINE_AA,
            )
        return tl

    sample_dir = train_root / "train" / "images"
    samples = sorted(sample_dir.glob("*"))[:6]
    if not samples:
        log.warning("No staged train images to plot")
        return
    fig, axes = plt.subplots(2, 3, figsize=(14, 9))
    for ax, sp in zip(axes.ravel(), samples):
        im = cv2.imread(str(sp))
        if im is None:
            continue
        lb = train_root / "train" / "labels" / (sp.stem + ".txt")
        vis = yolo_plot_bgr(im, lb)
        ax.imshow(cv2.cvtColor(vis, cv2.COLOR_BGR2RGB))
        ax.set_title(sp.name[:40], fontsize=8)
        ax.axis("off")
    plt.suptitle("Staged TRAIN samples (remapped labels)", fontsize=12)
    plt.tight_layout()
    out = work / "debug_staged_train_samples.png"
    plt.savefig(out, dpi=120)
    plt.close()
    log.info("Saved %s", out)


def plot_training_curves(run_dir: Path, work: Path, log: logging.Logger) -> None:
    import matplotlib.pyplot as plt

    try:
        import pandas as pd
    except ImportError:
        log.warning("pandas not installed — skip results.csv curves")
        return

    csv_path = run_dir / "results.csv"
    if not csv_path.is_file():
        log.warning("No results.csv at %s", csv_path)
        return
    df = pd.read_csv(csv_path)
    fig, ax = plt.subplots(1, 2, figsize=(12, 4))
    if "train/box_loss" in df.columns:
        ax[0].plot(df["epoch"], df["train/box_loss"], label="train box")
        if "val/box_loss" in df.columns:
            ax[0].plot(df["epoch"], df["val/box_loss"], label="val box")
        ax[0].legend()
        ax[0].set_title("Box loss")
        ax[0].set_xlabel("epoch")
    if "metrics/mAP50(B)" in df.columns:
        ax[1].plot(df["epoch"], df["metrics/mAP50(B)"], label="mAP50")
        ax[1].legend()
        ax[1].set_title("mAP50")
        ax[1].set_xlabel("epoch")
    plt.tight_layout()
    out = work / "debug_training_curves.png"
    plt.savefig(out, dpi=120)
    plt.close()
    log.info("Saved %s", out)


def main() -> int:
    args = parse_args()
    input_root: Path | None = args.input
    if input_root is None:
        env_root = os.environ.get("XIANGQI_DATASET_ROOT")
        if env_root:
            input_root = Path(env_root)
    if input_root is None and DEFAULT_KAGGLE_CHESS_DATASET.is_dir():
        input_root = DEFAULT_KAGGLE_CHESS_DATASET
    if input_root is None or not input_root.is_dir():
        log = logging.getLogger("train")
        logging.basicConfig(level=logging.INFO, format="%(asctime)s | %(levelname)s | %(message)s")
        log.error(
            "No dataset root found. Use --input /path/to/chess_dataset, or set XIANGQI_DATASET_ROOT, "
            "or attach the dataset at %s",
            DEFAULT_KAGGLE_CHESS_DATASET,
        )
        return 1

    work: Path = args.work
    work.mkdir(parents=True, exist_ok=True)
    train_root = work / "dataset"

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s | %(levelname)s | %(message)s",
        handlers=[logging.StreamHandler(sys.stdout)],
    )
    log = logging.getLogger("train")

    log.info("Input root: %s", input_root.resolve())
    log.info("Work dir: %s", work.resolve())

    if train_root.exists():
        shutil.rmtree(train_root)
    train_root.mkdir(parents=True)

    all_images = collect_images(input_root, args.subsets, log)
    if len(all_images) < 10:
        log.error("Too few images with labels: %d", len(all_images))
        return 1
    log.info("Total image/label pairs: %d", len(all_images))

    stage_dataset(input_root, train_root, all_images, args.val_fraction, args.seed, log)

    import yaml

    data_yaml = {
        "path": str(train_root.resolve()),
        "train": "train/images",
        "val": "val/images",
        "nc": len(CLASS_NAMES),
        "names": CLASS_NAMES,
    }
    yaml_path = train_root / "data.yaml"
    with open(yaml_path, "w", encoding="utf-8") as f:
        yaml.dump(data_yaml, f, default_flow_style=False, allow_unicode=False)
    log.info("Wrote %s", yaml_path)

    if not args.skip_plots:
        plot_staged_samples(train_root, work, log)

    if args.skip_train:
        log.info("--skip-train: done after staging")
        return 0

    try:
        from ultralytics import YOLO
    except ImportError:
        log.info("Installing ultralytics …")
        os.system(f"{sys.executable} -m pip install -q ultralytics pyyaml matplotlib opencv-python-headless pandas")
        from ultralytics import YOLO

    log.info(
        "Training: model=%s epochs=%d imgsz=%d batch=%d",
        args.model,
        args.epochs,
        args.imgsz,
        args.batch,
    )
    model = YOLO(args.model)
    model.train(
        data=str(yaml_path),
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        patience=args.patience,
        workers=args.workers,
        project=str(work / "runs"),
        name=args.run_name,
        exist_ok=True,
        pretrained=True,
        verbose=True,
        degrees=5.0,
        translate=0.1,
        scale=0.35,
        fliplr=0.5,
        flipud=0.0,
        mosaic=1.0,
        hsv_h=0.015,
        hsv_s=0.6,
        hsv_v=0.4,
        plots=True,
        save=True,
        save_period=10,
    )

    run_dir = work / "runs" / args.run_name
    weights_best = run_dir / "weights" / "best.pt"
    if not weights_best.is_file():
        log.error("Missing %s", weights_best)
        return 1
    log.info("Best weights: %s", weights_best)

    if not args.skip_plots:
        plot_training_curves(run_dir, work, log)

    best = YOLO(str(weights_best))
    # Pin project/name so Ultralytics does not write under cwd (e.g. /kaggle/working/runs/detect/val).
    best.val(
        data=str(yaml_path),
        split="val",
        plots=True,
        verbose=True,
        project=str(run_dir),
        name="post_train_val",
        exist_ok=True,
    )
    log.info("Post-train val plots: %s", run_dir / "post_train_val")

    # Predict on a few val images; save under work/val_predictions
    import cv2
    import matplotlib.pyplot as plt

    val_imgs = sorted((train_root / "val" / "images").glob("*"))[:8]
    if val_imgs:
        pred_dir = work / "val_predictions"
        pred_dir.mkdir(parents=True, exist_ok=True)
        pred_results = best.predict(
            source=[str(p) for p in val_imgs],
            conf=0.25,
            save=True,
            project=str(pred_dir),
            name="batch",
            exist_ok=True,
        )
        for r in pred_results[:8]:
            im = r.plot()
            fig, ax = plt.subplots(figsize=(6, 6))
            ax.imshow(cv2.cvtColor(im, cv2.COLOR_BGR2RGB))
            ax.set_title(Path(r.path).name[:50], fontsize=8)
            ax.axis("off")
            plt.tight_layout()
            fig_path = pred_dir / f"preview_{Path(r.path).stem}.png"
            plt.savefig(fig_path, dpi=100)
            plt.close()
        log.info("Saved val prediction previews under %s", pred_dir)

    final_pt = work / f"{args.run_name}_best.pt"
    shutil.copy2(weights_best, final_pt)
    log.info("Copy for download / deploy: %s", final_pt.resolve())
    print("\n=== DONE ===")
    print(f"Use this weights file with vision_node model_path: {final_pt}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
