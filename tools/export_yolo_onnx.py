"""Export YOLO .pt weights to ONNX for faster CPU inference (2-5x speedup via onnxruntime).

Usage:
    python tools/export_yolo_onnx.py --model workspace/models/xiangqi_kaggle_v4_best.pt

The output .onnx file is written alongside the .pt file.
Update vision_config.yaml → model_path to point at the .onnx to activate it.
"""
import argparse
from pathlib import Path


def main() -> None:
    parser = argparse.ArgumentParser(description='Export Xiangqi YOLO weights to ONNX.')
    parser.add_argument('--model', required=True, help='Path to .pt weights file')
    parser.add_argument('--imgsz', type=int, default=640, help='Input image size (default 640)')
    args = parser.parse_args()

    pt_path = Path(args.model).resolve()
    if not pt_path.exists():
        raise FileNotFoundError(f'Model not found: {pt_path}')

    from ultralytics import YOLO
    model = YOLO(str(pt_path))
    out = model.export(format='onnx', imgsz=args.imgsz, half=False, dynamic=False)
    print(f'Exported to: {out}')
    print('Update vision_config.yaml → model_path to use the .onnx file.')


if __name__ == '__main__':
    main()
