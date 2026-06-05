#!/bin/bash
# Export YOLO .pt -> .onnx inside the running UR5e_Env container (CPU, idempotent).
#
# Run on the lab host (after lab_ensure_deps / ./docker-xiangqi.sh):
#   ~/par_ur5e_xiangqi/tools/lab_export_yolo_onnx.sh
#
# Env:
#   CONTAINER=ros2
#   YOLO_PT=/home/rosuser/workspace/models/xiangqi_kaggle_v4_best.pt
#   YOLO_EXPORT_IMGSZ=640
#   FORCE_ONNX_EXPORT=1   re-export even if .onnx is newer than .pt
#   SKIP_ONNX_EXPORT=1    no-op
set -euo pipefail

CONTAINER="${CONTAINER:-ros2}"
# Use default when unset or empty (docker-start may pass YOLO_PT=).
YOLO_PT="${YOLO_PT-/home/rosuser/workspace/models/xiangqi_kaggle_v4_best.pt}"
YOLO_EXPORT_IMGSZ="${YOLO_EXPORT_IMGSZ:-640}"

if [[ "${SKIP_ONNX_EXPORT:-}" == "1" ]]; then
  echo "SKIP_ONNX_EXPORT=1 — not exporting ONNX."
  exit 0
fi

if ! docker ps --format '{{.Names}}' | grep -qx "$CONTAINER"; then
  echo "Container '$CONTAINER' is not running."
  exit 1
fi

docker exec -u rosuser "$CONTAINER" bash -lc "
set -euo pipefail
PT='${YOLO_PT}'
IMGSZ='${YOLO_EXPORT_IMGSZ}'
FORCE='${FORCE_ONNX_EXPORT:-}'
ONNX=\"\${PT%.pt}.onnx\"

if [[ ! -f \"\$PT\" ]]; then
  echo \"SKIP: YOLO weights not found at \$PT (sync workspace/models first)\"
  exit 0
fi

python3 -c 'import onnxruntime' 2>/dev/null || {
  echo 'ERROR: onnxruntime not installed — run lab_ensure_deps.sh first'
  exit 1
}

if [[ -f \"\$ONNX\" && \"\$FORCE\" != \"1\" && ! \"\$PT\" -nt \"\$ONNX\" ]]; then
  echo \"OK: ONNX already present: \$ONNX\"
  exit 0
fi

echo \"==> Exporting ONNX (imgsz=\$IMGSZ) from \$PT ...\"
echo \"    (first run may take a few minutes)\"
yolo export model=\"\$PT\" format=onnx imgsz=\"\$IMGSZ\"

if [[ -f \"\$ONNX\" ]]; then
  echo \"OK: wrote \$ONNX\"
  ls -lh \"\$ONNX\"
else
  echo \"WARN: export finished but \$ONNX not found — check yolo output path\"
  exit 1
fi
"
