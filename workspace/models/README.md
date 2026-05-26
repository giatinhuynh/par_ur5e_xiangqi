# Lab YOLO weights (`workspace/models/`)

`vision_config.yaml` loads **`xiangqi_kaggle_v3_best.pt`** from this folder on the robot PC (`/home/rosuser/workspace/models/`).

| File | In git | Notes |
|------|--------|--------|
| `xiangqi_kaggle_v1_best.pt` | yes (~6 MB) | Legacy |
| `xiangqi_kaggle_v2_best.pt` | yes (~21 MB) | Legacy |
| `xiangqi_kaggle_v3_best.pt` | **no** (~130 MB) | Exceeds GitHub 100 MB limit — **not pushed** |

## Deploy v3 to the lab

Copy from your machine (after training or from Kaggle):

```bash
scp workspace/models/xiangqi_kaggle_v3_best.pt vxlab@10.234.7.84:/home/rosuser/workspace/models/
```

Or use `tools/fetch_yolo_weights.py` with a download URL and set `yolo_download_url` in `vision_config.yaml`.
