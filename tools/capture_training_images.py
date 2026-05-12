#!/usr/bin/env python3
"""
capture_training_images.py
Captures raw images from the lab camera for YOLO training data collection.

Press SPACE to save a frame.
Press 'r' to show a reminder of capture configurations still needed.
Press 'q' to quit.

Saves to: ~/xiangqi_dataset/raw_captures/
"""

import cv2
import os
import time
from pathlib import Path

SAVE_DIR = Path.home() / 'xiangqi_dataset' / 'raw_captures'
SAVE_DIR.mkdir(parents=True, exist_ok=True)

CONFIGS = [
    ('full_start',     'Full starting position (all 32 pieces)',        10),
    ('mid_game',       'Mid-game position (10-20 pieces remaining)',    15),
    ('end_game',       'End-game position (few pieces left)',           10),
    ('graveyard',      'Captured pieces in graveyard zones',            5),
    ('single_type',    'Single piece type only (one row)',              10),
    ('bright_light',   'Under bright overhead light',                   5),
    ('dim_light',      'Under dim/window light',                        5),
    ('slight_tilt',    'Slight board rotation (±3°)',                   5),
    ('hand_in_frame',  'Human hand partially in frame (mid-move)',      5),
]

config_counts = {c[0]: 0 for c in CONFIGS}

def current_config_idx():
    for i, (key, _, target) in enumerate(CONFIGS):
        if config_counts[key] < target:
            return i
    return len(CONFIGS) - 1

def print_progress():
    print('\n--- Capture Progress ---')
    total = 0
    for key, desc, target in CONFIGS:
        n = config_counts[key]
        total += n
        bar = '█' * n + '░' * max(0, target - n)
        status = '✓' if n >= target else f'{n}/{target}'
        print(f'  [{status:>5}] {bar}  {desc}')
    print(f'  Total: {total} images\n')


def main():
    # Try to open camera (index 0 first, then 2 for RealSense on some systems)
    cap = None
    for cam_idx in [0, 2, 4]:
        cap = cv2.VideoCapture(cam_idx)
        if cap.isOpened():
            print(f'Camera opened at index {cam_idx}')
            break
        cap.release()

    if not cap or not cap.isOpened():
        print('ERROR: Could not open camera. Check USB connection.')
        return

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap.set(cv2.CAP_PROP_AUTOFOCUS, 1)

    count = len(list(SAVE_DIR.glob('*.jpg')))  # Resume from existing count
    print(f'Saving to: {SAVE_DIR}')
    print(f'Starting at index: {count}')
    print_progress()

    while True:
        ret, frame = cap.read()
        if not ret:
            print('ERROR: Failed to read frame.')
            break

        cfg_idx  = current_config_idx()
        cfg_key, cfg_desc, cfg_target = CONFIGS[cfg_idx]
        cfg_done = config_counts[cfg_key]

        overlay = frame.copy()
        # Semi-transparent status bar
        cv2.rectangle(overlay, (0, 0), (frame.shape[1], 60), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.6, frame, 0.4, 0, frame)

        cv2.putText(frame, f'Config: {cfg_desc}  [{cfg_done}/{cfg_target}]',
                    (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 120), 2)
        cv2.putText(frame, f'Total saved: {count}  |  SPACE=save  R=progress  Q=quit',
                    (10, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

        cv2.imshow('Xiangqi Training Data Capture', frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

        elif key == ord(' '):
            fname = SAVE_DIR / f'{cfg_key}_{int(time.time())}_{count:04d}.jpg'
            cv2.imwrite(str(fname), frame)
            config_counts[cfg_key] += 1
            count += 1
            print(f'  Saved [{cfg_key} {config_counts[cfg_key]}/{cfg_target}]: {fname.name}')
            if config_counts[cfg_key] >= cfg_target:
                print(f'  ✓ Config "{cfg_key}" complete!')
                print_progress()

        elif key == ord('r'):
            print_progress()

    cap.release()
    cv2.destroyAllWindows()
    print(f'\nCapture session complete. Total: {count} images in {SAVE_DIR}')
    print_progress()


if __name__ == '__main__':
    main()
