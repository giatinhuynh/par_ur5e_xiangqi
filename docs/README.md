# Documentation index

**Setup and launch:** [repository README](../README.md) (Docker, calibration, drivers, sim §8b).

**System design:** [.cursor/plans/xiangqi_robot_system_plan_64c0c1b9.plan.md](../.cursor/plans/xiangqi_robot_system_plan_64c0c1b9.plan.md).

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
| [sim_mode.md](sim_mode.md) | Simulation architecture, config, troubleshooting |

## Lab integration

| Document | Description |
|----------|-------------|
| [ur5e_env_alignment.md](ur5e_env_alignment.md) | UR5e_Env vs this repo: MoveIt, RG2, aliases, workspace layout |
| [lab_docker_deps.md](lab_docker_deps.md) | Docker / dependency notes |
| [ur5evxlabdoc.md](../ur5evxlabdoc.md) | VXLab IPs, pendant, driver commands |

## Course / report

| Document | Description |
|----------|-------------|
| [assignment_rubric_checklist.md](assignment_rubric_checklist.md) | §4.8 mapping, report checklist, rubric readiness |
| [assignment.md](../assignment.md) | Official brief |

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
