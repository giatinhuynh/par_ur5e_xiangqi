# Lab YOLO weights (`workspace/models/`)

Mounted in Docker as `/home/rosuser/workspace/models/`. Referenced by `xiangqi_bringup/config/vision_config.yaml`.

| File | In git | Notes |
|------|--------|--------|
| `xiangqi_kaggle_v1_best.pt` | yes | Legacy / backup |
| `xiangqi_kaggle_v2_best.pt` | yes | Legacy |
| `xiangqi_kaggle_v4_best.pt` | yes | **Current default** in `vision_config.yaml` |
| `xiangqi_kaggle_v3_best.pt` | no | Too large for GitHub (>100 MB) — keep locally or fetch via URL |

Copy to the lab dev box:

```bash
scp workspace/models/xiangqi_kaggle_v4_best.pt vxlab@10.234.7.84:/home/rosuser/workspace/models/
```

Or: `tools/fetch_yolo_weights.py` + `yolo_download_url` in `vision_config.yaml`.

Package-bundled weights (v1): [../src/xiangqi_vision/models/README.md](../src/xiangqi_vision/models/README.md).
