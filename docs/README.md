# Documentation Index

Full-stack setup (Docker, calibration, weights, launch) is in the repository root **[README.md](../README.md)**.

**Simulation (no robot):** step-by-step setup and dashboard instructions in **[README.md §8b](../README.md#8b-simulation-mode-setup-and-instructions)**; background and troubleshooting in **[sim_mode.md](sim_mode.md)**.

## Physical Setup

| Document | Description |
|---|---|
| [board_printing_guide.md](board_printing_guide.md) | How to generate, print, and verify the custom Xiangqi board mat with ArUco fiducial markers |
| [pieces_guide.md](pieces_guide.md) | Procurement guide (buying standard pieces) and 3D printing specifications if needed |

## Vision & AI

| Document | Description |
|---|---|
| [sim_mode.md](sim_mode.md) | Simulation vs hardware, ROS behaviour, config, verification scripts, troubleshooting |
| [vision_training_guide.md](vision_training_guide.md) | Complete step-by-step YOLOv8n training pipeline: dataset sourcing, lab image capture, labelling, training, validation, and deployment |

## Lab stack integration

| Document | Description |
|---|---|
| [ur5e_env_alignment.md](ur5e_env_alignment.md) | Verified ROS interfaces (`/par_moveit/waypoint_move`, `/rg2/*`), Compose/workspace layout, aliases, packages split vs UR5e_Env |

## Assessment (course project)

| Document | Description |
|---|---|
| [assignment_rubric_checklist.md](assignment_rubric_checklist.md) | §4.8 requirement mapping, report expectations (§6), shared themes (§2), readiness checklist vs rubric — companion to root [`assignment.md`](../assignment.md) |

## Tools

Scripts in `../tools/`:

| Script | Purpose |
|---|---|
| `generate_board_svg.py` | Generates `board_mat_A2.svg` and `board_mat_A3.svg` for printing |
| `kaggle_train_xiangqi_yolo.py` | One-shot YOLOv8 train (Kaggle/local): remap chess_dataset labels → `piece_detector` class IDs, stage train/val, train, plots |
| `kaggle_preview_detections.py` | Grid preview of detections; default samples **Wooden**, **Acrylic**, **Stainless Steel**, and **All Types** under the dataset root |
| `capture_training_images.py` | Guided image capture from lab camera with per-configuration counters |
| `merge_datasets.py` | Merges Roboflow base dataset + lab captures into a single YOLOv8 dataset |
| `inspect_predictions.py` | Visual inspection of trained model predictions on validation images |
