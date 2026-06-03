# Lab Docker: persistent Xiangqi dependencies

The stock UR5e_Env image (`ros:humble`) has ROS + arm drivers but **not** Ultralytics, ONNX runtime, Flask, Fairy-Stockfish, or pyffish. Those live in this repo’s [Dockerfile](../Dockerfile) (`ultralytics`, `onnx`, `onnxruntime` for optional CPU `.onnx` inference).

## Recommended: `./docker-xiangqi.sh` in `~/UR5e_Env` (replaces `./docker-start.sh`)

One-time install on the lab host:

```bash
~/par_ur5e_xiangqi/tools/ur5e_env/install-docker-xiangqi.sh
```

Daily use:

```bash
cd ~/UR5e_Env
./docker-xiangqi.sh
./docker-attach.sh
```

Or without installing the wrapper:

```bash
~/par_ur5e_xiangqi/tools/docker-start-xiangqi.sh
cd ~/UR5e_Env && ./docker-attach.sh
```

This script:

1. Runs `docker-compose up -d` in `~/UR5e_Env` (same env vars as upstream `docker-start.sh`)
2. Uses **`ur5e_xiangqi:latest`** automatically if that image exists; otherwise starts **`ros:humble`**
3. Runs **`lab_ensure_deps.sh`** (idempotent — skips if deps already OK)

Force stock image + pip deps only:

```bash
USE_XIANGQI_IMAGE=0 ~/par_ur5e_xiangqi/tools/docker-start-xiangqi.sh
```

## What survives a container reset?

| Persists | Lost on new container from `./docker-build.sh` |
|----------|--------------------------------------------------|
| `~/UR5e_Env/workspace/` on the host (packages, calibration, `colcon` install) | Pip installs in container unless baked into image |

`./docker-build.sh` in UR5e_Env **recreates the image** → run **`docker-start-xiangqi.sh`** again (deps step is fast if marker/imports already OK).

If you see `KeyError: 'ContainerConfig'` from old `docker-compose` 1.29, use **`./docker-xiangqi.sh`** (it uses **`docker compose` v2** and removes a stale `ros2` container before start). Or manually: `docker rm -f ros2 && ./docker-xiangqi.sh`.

## Option B — Bake deps into the image (fastest daily start)

```bash
~/par_ur5e_xiangqi/tools/lab_build_xiangqi_image.sh
~/par_ur5e_xiangqi/tools/docker-start-xiangqi.sh   # auto-picks ur5e_xiangqi:latest
```

No manual `lab_ensure_deps.sh` needed in most cases when the extended image exists.

## Manual dep install only

```bash
~/par_ur5e_xiangqi/tools/lab_ensure_deps.sh
```

## ONNX weights (optional, faster CPU inference)

**Automatic:** `./docker-xiangqi.sh` runs `lab_ensure_deps.sh`, which calls `lab_export_yolo_onnx.sh` when
`/home/rosuser/workspace/models/xiangqi_kaggle_v4_best.pt` exists and `.onnx` is missing or older than `.pt`.

```bash
# Manual export only
~/par_ur5e_xiangqi/tools/lab_export_yolo_onnx.sh
FORCE_ONNX_EXPORT=1 ~/par_ur5e_xiangqi/tools/lab_export_yolo_onnx.sh
SKIP_ONNX_EXPORT=1 ./docker-xiangqi.sh   # deps yes, export no
```

Point `vision_config.yaml` at the `.onnx` and set `yolo_imgsz: 640`. Marker **`xiangqi_deps_installed_v2`** includes `onnxruntime`; older `v1` containers re-run `lab_ensure_deps.sh` on next `./docker-xiangqi.sh`.

**Not in `ros2 launch`:** export takes minutes and blocks startup — keep it in the docker start scripts only.

## When to re-run

| Event | Action |
|-------|--------|
| Daily start | **`docker-start-xiangqi.sh`** |
| `./docker-build.sh` (new `ros:humble`) | **`docker-start-xiangqi.sh`** (re-installs deps if needed) |
| Rebuilt `ur5e_xiangqi:latest` | **`docker-start-xiangqi.sh`** or rebuild image from updated Dockerfile |
