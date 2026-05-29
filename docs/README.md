# Documentation index

**Full system design, architecture, interfaces, and setup:** [repository README](../README.md).

## Physical setup

| Document | Description |
|----------|-------------|
| [board_printing_guide.md](board_printing_guide.md) | Board mat SVG, printing, ArUco layout |
| [pieces_guide.md](pieces_guide.md) | Piece procurement / 3D print notes |
| `board_geometry_*.yaml` | Grid spacing for `tools/generate_board_svg.py` |

## Vision and simulation

| Document | Description |
|----------|-------------|
| [vision_training_guide.md](vision_training_guide.md) | YOLOv8 dataset, training, deployment, class map |
| [sim_mode.md](sim_mode.md) | Simulation vs hardware, config, troubleshooting |

## Lab integration

| Document | Description |
|----------|-------------|
| [ur5e_env_alignment.md](ur5e_env_alignment.md) | UR5e_Env vs this repo: MoveIt, RG2, aliases |
| [lab_docker_deps.md](lab_docker_deps.md) | Docker / dependency notes |
| [ur5evxlabdoc.md](../ur5evxlabdoc.md) | VXLab IPs, pendant, driver commands |

## Host tools (`../tools/`)

| Script | Purpose |
|--------|---------|
| `generate_board_svg.py` | `docs/board_mat_A2.svg`, `board_mat_A3.svg`, … |
| `kaggle_train_xiangqi_yolo.py` | Train / export YOLOv8 |
| `kaggle_preview_detections.py` | Detection preview grid |
| `capture_training_images.py` | Lab camera capture for dataset |
| `merge_datasets.py` | Merge Roboflow + lab images |
| `inspect_predictions.py` | Validation visual inspection |
| `wsl_docker_sim_run.sh` | Docker sim + dashboard on port 5000 |
| `wsl_docker_rebuild.sh` | Rebuild image and start sim |
| `verify_api_pyffish.py` | Dashboard endgame vs pyffish |
| `docker_verify_fsf_pyffish.py` | FSF legal-move sanity in image build |

Per-package logic: `workspace/src/*/README.md`.
