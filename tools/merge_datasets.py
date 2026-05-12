#!/usr/bin/env python3
"""
merge_datasets.py
Merges the downloaded Roboflow base dataset and the lab-captured + annotated dataset
into a single combined dataset ready for YOLOv8 training.

Usage:
    python3 tools/merge_datasets.py \
        --base xiangqi-pieces-1 \
        --lab  lab_dataset \
        --out  ~/xiangqi_dataset
"""

import argparse
import os
import shutil
import yaml
from pathlib import Path


TARGET_CLASSES = [
    'r_general', 'r_advisor', 'r_elephant', 'r_horse',
    'r_chariot', 'r_cannon', 'r_soldier',
    'b_general', 'b_advisor', 'b_elephant', 'b_horse',
    'b_chariot', 'b_cannon', 'b_soldier',
]


def load_class_names(yaml_path: Path):
    with open(yaml_path) as f:
        info = yaml.safe_load(f)
    return info.get('names', [])


def build_remap(src_names):
    """
    Build a mapping from src class ID to target class ID.
    Heuristic: match on the piece type substring (general/advisor/elephant/horse/chariot/cannon/soldier)
    and colour prefix (r_/b_).
    """
    remap = {}
    for tgt_id, tgt in enumerate(TARGET_CLASSES):
        tgt_colour = tgt[0]   # 'r' or 'b'
        tgt_type   = tgt.split('_', 1)[1]
        for src_id, src in enumerate(src_names):
            src_lower = src.lower().replace('-', '_')
            # Colour match
            colour_match = (
                ('red' in src_lower or src_lower.startswith('r_')) and tgt_colour == 'r'
            ) or (
                ('black' in src_lower or src_lower.startswith('b_')) and tgt_colour == 'b'
            )
            # Type match (alias handling)
            type_aliases = {
                'general': ['general', 'king', 'marshal', 'commander'],
                'advisor': ['advisor', 'guard', 'assistant'],
                'elephant': ['elephant', 'bishop'],
                'horse': ['horse', 'knight'],
                'chariot': ['chariot', 'rook', 'car'],
                'cannon': ['cannon'],
                'soldier': ['soldier', 'pawn'],
            }
            type_match = any(alias in src_lower for alias in type_aliases.get(tgt_type, [tgt_type]))
            if colour_match and type_match:
                remap[src_id] = tgt_id
                break
    return remap


def remap_label_file(src_path: Path, dst_path: Path, remap: dict):
    lines = src_path.read_text().strip().split('\n') if src_path.exists() else []
    new_lines = []
    for line in lines:
        if not line.strip():
            continue
        parts = line.split()
        old_id = int(parts[0])
        new_id = remap.get(old_id, old_id)
        new_lines.append(f"{new_id} {' '.join(parts[1:])}")
    dst_path.write_text('\n'.join(new_lines) + ('\n' if new_lines else ''))


def copy_split(src_dir: Path, dst_dir: Path, remap: dict, prefix: str = ''):
    img_src = src_dir / 'images'
    lbl_src = src_dir / 'labels'
    img_dst = dst_dir / 'images'
    lbl_dst = dst_dir / 'labels'
    img_dst.mkdir(parents=True, exist_ok=True)
    lbl_dst.mkdir(parents=True, exist_ok=True)

    if not img_src.exists():
        return 0

    count = 0
    for img_file in img_src.iterdir():
        if img_file.suffix.lower() not in ('.jpg', '.jpeg', '.png'):
            continue
        dst_name = f'{prefix}{img_file.name}' if prefix else img_file.name
        shutil.copy(img_file, img_dst / dst_name)

        lbl_file = lbl_src / (img_file.stem + '.txt')
        dst_lbl = lbl_dst / (Path(dst_name).stem + '.txt')
        remap_label_file(lbl_file, dst_lbl, remap)
        count += 1

    return count


def write_data_yaml(output: Path):
    cfg = {
        'path': str(output.resolve()),
        'train': 'train/images',
        'val':   'val/images',
        'test':  'test/images',
        'nc': len(TARGET_CLASSES),
        'names': TARGET_CLASSES,
    }
    with open(output / 'data.yaml', 'w') as f:
        yaml.dump(cfg, f, default_flow_style=False, allow_unicode=True)
    print(f"data.yaml written to {output / 'data.yaml'}")


def main():
    parser = argparse.ArgumentParser(description='Merge Roboflow + lab datasets for YOLOv8 training')
    parser.add_argument('--base', default='xiangqi-pieces-1', help='Path to Roboflow base dataset')
    parser.add_argument('--lab',  default='lab_dataset',       help='Path to lab-captured dataset')
    parser.add_argument('--out',  default='~/xiangqi_dataset', help='Output combined dataset path')
    args = parser.parse_args()

    base = Path(args.base)
    lab  = Path(args.lab)
    out  = Path(os.path.expanduser(args.out))
    out.mkdir(parents=True, exist_ok=True)

    total = {'train': 0, 'val': 0}

    # --- Base dataset ---
    if base.exists():
        base_yaml = base / 'data.yaml'
        base_names = load_class_names(base_yaml) if base_yaml.exists() else TARGET_CLASSES
        base_remap = build_remap(base_names) if base_names != TARGET_CLASSES else {i: i for i in range(len(TARGET_CLASSES))}
        print(f"Base dataset class remap: {base_remap}")

        for src_split, dst_split in [('train', 'train'), ('valid', 'val'), ('test', 'test')]:
            src_dir = base / src_split
            if src_dir.exists():
                n = copy_split(src_dir, out / dst_split, base_remap, prefix='')
                total[dst_split] = total.get(dst_split, 0) + n
                print(f"  Base {src_split}: {n} images → {dst_split}")
    else:
        print(f"WARNING: Base dataset not found at {base}. Skipping.")

    # --- Lab dataset ---
    if lab.exists():
        lab_yaml = lab / 'data.yaml'
        lab_names = load_class_names(lab_yaml) if lab_yaml.exists() else TARGET_CLASSES
        lab_remap = build_remap(lab_names) if lab_names != TARGET_CLASSES else {i: i for i in range(len(TARGET_CLASSES))}

        for src_split, dst_split in [('train', 'train'), ('valid', 'val'), ('val', 'val')]:
            src_dir = lab / src_split
            if src_dir.exists():
                n = copy_split(src_dir, out / dst_split, lab_remap, prefix='lab_')
                total[dst_split] = total.get(dst_split, 0) + n
                print(f"  Lab {src_split}: {n} images → {dst_split}")
    else:
        print(f"INFO: Lab dataset not found at {lab}. Only base dataset will be used.")

    write_data_yaml(out)

    print('\n=== Merge complete ===')
    for split, n in total.items():
        print(f'  {split}: {n} images')


if __name__ == '__main__':
    main()
