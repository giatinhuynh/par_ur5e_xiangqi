# YOLO weights (package `share`)

Files here install to `share/xiangqi_vision/models/` after `colcon build`.

| File | In git | Notes |
|------|--------|--------|
| `xiangqi_kaggle_v1_best.pt` | yes | Default for **sim** / offline when no lab `workspace/models/` copy |

**Lab hardware** uses `xiangqi_kaggle_v4_best.pt` via `vision_config.yaml` (see [workspace/models/README.md](../../../../workspace/models/README.md)).

**Optional download:** set `yolo_download_url` on `vision_node` or use `tools/fetch_yolo_weights.py`. See `weights_util.py`.

Training: [docs/vision_training_guide.md](../../../../docs/vision_training_guide.md).
