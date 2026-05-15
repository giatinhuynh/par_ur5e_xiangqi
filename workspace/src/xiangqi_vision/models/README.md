# YOLO weights (`*.pt`)

Place your trained Xiangqi piece detector here as **`xiangqi_kaggle_v1_best.pt`** (or set `model_path` in `vision_config.yaml`). After `colcon build`, files in this directory are installed to `share/xiangqi_vision/models/` and picked up automatically when the workspace path does not exist.

**Download at runtime (optional):** set ROS parameter `yolo_download_url` on `vision_node`, or environment variable **`XIANGQI_YOLO_DOWNLOAD_URL`**, to an `http(s)` URL pointing at a `.pt` file. On startup the node downloads into the package models directory (see `xiangqi_vision/weights_util.py`).

**One-shot fetch from the shell:**

```bash
python3 tools/fetch_yolo_weights.py --url 'https://example.com/your_weights.pt' --dest workspace/src/xiangqi_vision/models/xiangqi_kaggle_v1_best.pt
```

Training and export are described in `docs/vision_training_guide.md`.
