#!/usr/bin/env python3
"""
inspect_predictions.py
Runs the trained YOLOv8 model on validation images and displays bounding box predictions
for manual inspection.

Usage:
    python3 tools/inspect_predictions.py \
        --model runs/detect/xiangqi_v1/weights/best.pt \
        --images ~/xiangqi_dataset/val/images \
        --conf 0.4 \
        --n 30
"""

import argparse
import glob
import os
import random
import cv2
import numpy as np

try:
    from ultralytics import YOLO
except ImportError:
    print("ERROR: ultralytics not installed. Run: pip install ultralytics")
    raise

CLASS_NAMES = [
    'r_general', 'r_advisor', 'r_elephant', 'r_horse',
    'r_chariot', 'r_cannon', 'r_soldier',
    'b_general', 'b_advisor', 'b_elephant', 'b_horse',
    'b_chariot', 'b_cannon', 'b_soldier',
]

CLASS_COLOURS = {
    i: (0, 80, 200) if i < 7 else (0, 0, 0)   # Red = blue-ish (BGR); Black = black
    for i in range(14)
}


def draw_predictions(img: np.ndarray, results) -> np.ndarray:
    vis = img.copy()
    boxes = results[0].boxes
    if boxes is None:
        return vis
    for box in boxes:
        x1, y1, x2, y2 = map(int, box.xyxy[0])
        cls_id = int(box.cls[0])
        conf   = float(box.conf[0])
        label  = f'{CLASS_NAMES[cls_id]} {conf:.2f}'
        colour = CLASS_COLOURS.get(cls_id, (128, 128, 128))
        cv2.rectangle(vis, (x1, y1), (x2, y2), colour, 2)
        cv2.putText(vis, label, (x1, max(y1 - 5, 0)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, colour, 2)
    return vis


def load_ground_truth(label_path: str, img_w: int, img_h: int):
    boxes = []
    if not os.path.exists(label_path):
        return boxes
    with open(label_path) as f:
        for line in f:
            parts = line.strip().split()
            if not parts:
                continue
            cls_id = int(parts[0])
            cx, cy, w, h = map(float, parts[1:5])
            x1 = int((cx - w/2) * img_w)
            y1 = int((cy - h/2) * img_h)
            x2 = int((cx + w/2) * img_w)
            y2 = int((cy + h/2) * img_h)
            boxes.append((cls_id, x1, y1, x2, y2))
    return boxes


def draw_gt(img: np.ndarray, boxes) -> np.ndarray:
    vis = img.copy()
    for cls_id, x1, y1, x2, y2 in boxes:
        label = CLASS_NAMES[cls_id]
        cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 200, 0), 1)  # GT in green
        cv2.putText(vis, f'GT:{label}', (x1, y2 + 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 180, 0), 1)
    return vis


def main():
    parser = argparse.ArgumentParser(description='Inspect YOLOv8 predictions on validation images')
    parser.add_argument('--model',  default='runs/detect/xiangqi_v1/weights/best.pt')
    parser.add_argument('--images', default=os.path.expanduser('~/xiangqi_dataset/val/images'))
    parser.add_argument('--conf',   type=float, default=0.4)
    parser.add_argument('--n',      type=int,   default=30, help='Number of images to inspect')
    parser.add_argument('--gt',     action='store_true', help='Also overlay ground-truth boxes')
    args = parser.parse_args()

    model = YOLO(args.model)
    print(f'Loaded: {args.model}')

    img_paths = sorted(glob.glob(os.path.join(args.images, '*.jpg')) +
                       glob.glob(os.path.join(args.images, '*.png')))
    random.shuffle(img_paths)
    img_paths = img_paths[:args.n]
    print(f'Inspecting {len(img_paths)} images. Press SPACE or any key to advance, Q to quit.')

    for i, img_path in enumerate(img_paths):
        img = cv2.imread(img_path)
        if img is None:
            continue

        results = model(img_path, conf=args.conf, verbose=False)
        vis = draw_predictions(img, results)

        if args.gt:
            label_path = img_path.replace('/images/', '/labels/').rsplit('.', 1)[0] + '.txt'
            gt_boxes = load_ground_truth(label_path, img.shape[1], img.shape[0])
            vis = draw_gt(vis, gt_boxes)

        n_det = len(results[0].boxes) if results[0].boxes else 0
        cv2.putText(vis, f'{i+1}/{len(img_paths)}: {os.path.basename(img_path)}  |  {n_det} detections',
                    (5, img.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 0), 2)

        cv2.imshow('Prediction Inspector', vis)
        key = cv2.waitKey(0) & 0xFF
        if key == ord('q'):
            break

    cv2.destroyAllWindows()
    print('Done.')


if __name__ == '__main__':
    main()
