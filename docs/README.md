# Documentation Index

## Physical Setup

| Document | Description |
|---|---|
| [board_printing_guide.md](board_printing_guide.md) | How to generate, print, and verify the custom Xiangqi board mat with ArUco fiducial markers |
| [pieces_guide.md](pieces_guide.md) | Procurement guide (buying standard pieces) and 3D printing specifications if needed |

## Vision & AI

| Document | Description |
|---|---|
| [vision_training_guide.md](vision_training_guide.md) | Complete step-by-step YOLOv8n training pipeline: dataset sourcing, lab image capture, labelling, training, validation, and deployment |

## Tools

Scripts in `../tools/`:

| Script | Purpose |
|---|---|
| `generate_board_svg.py` | Generates `board_mat_A2.svg` and `board_mat_A3.svg` for printing |
| `capture_training_images.py` | Guided image capture from lab camera with per-configuration counters |
| `merge_datasets.py` | Merges Roboflow base dataset + lab captures into a single YOLOv8 dataset |
| `inspect_predictions.py` | Visual inspection of trained model predictions on validation images |
