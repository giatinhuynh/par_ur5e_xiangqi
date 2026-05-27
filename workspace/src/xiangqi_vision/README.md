# xiangqi_vision

**Tier 1 (reactive) perception**: board geometry, piece classification, human turn detection, and calibration tooling.

Important integration note: vision does not command arm motion by itself. In the current system, scan timing is coordinated by planner/game-manager so the wrist camera is moved to a top-down **scan pose** before key scans/watch phases. The scan-pose service in manipulation derives board-centred x/y from `board_calibration.yaml` when available, with parameter fallback if calibration is missing.

---

## How does vision “know” the board?

The system does **not** assume a fixed camera pose or a manually drawn ROI. It finds the board **every frame** (or each detection tick) using **fiducial markers** printed on your mat, then **rectifies** the image so the 9×10 grid lines up with a predictable pixel layout.

### 1. Physical setup: markers + grid

The board SVG generator ([`tools/generate_board_svg.py`](../../../tools/generate_board_svg.py)) places **four ArUco markers** on the **outer corners of the printed sheet** (not on individual squares). In code (`board_detector.py`), dictionary **`DICT_4X4_50`** is used, with this ID layout:

| Marker ID | Sheet corner | Board corner (conceptually) |
|-----------|--------------|-----------------------------|
| **0** | Top-left of sheet | Near file `a`, rank `9` (black side) |
| **1** | Top-right | Near file `i`, rank `9` |
| **2** | Bottom-right | Near file `i`, rank `0` (red / robot side) |
| **3** | Bottom-left | Near file `a`, rank `0` |

The **playing grid** (lines and river) sits **inside** the quadrilateral formed by those four markers. **`grid_spacing_mm`** in calibration / geometry YAML must match the real distance between **adjacent intersections** on the mat so that, after warping, pixel spacing matches physical spacing in the normalized image.

### 2. Finding the board in the camera image (each frame)

1. **ArUco detection** - OpenCV’s `ArucoDetector` scans the RGB frame for markers with IDs 0–3.
2. **Need all four** - If fewer than four are found (occlusion, glare, motion blur, mat too small in frame), `BoardDetector.detect()` **fails** for that frame: no homography, no YOLO input, `BoardState` is not updated from that tick.
3. **Homography** - For each visible marker, the detector uses the marker’s centre in **image pixels** as a corner sample. Those four source points are matched to four **fixed destination corners** in a synthetic “top-down” bitmap of size **800×890** with a **44 px margin** (`board_detector.py`). `cv2.findHomography(..., RANSAC)` estimates the 3×3 matrix **H** that maps the slanted view → flat board plane.
4. **Warp** - `cv2.warpPerspective` applies **H** to the full image, producing a **normalized board image**: as if the camera looked straight down at the board. YOLO always runs on this warp, so piece appearance is consistent and cells line up with the internal grid math.

So: **the board is “known” because the four ArUco corners anchor a geometric transform**; the grid is not searched blindly in the raw image.

### 3. From warped pixels to `(file, rank)` and `BoardState.grid`

- **`PieceDetector`** runs YOLO on the warped image. Each detection’s centre is snapped to the nearest intersection via **`board_layout.pixel_to_grid`**, which matches the **inset** 4×A3 grid on the printed mat (not uniform spacing from the 44 px warp margin alone).
- The flat **`int8[90]`** array uses index **`rank * 9 + file`**, rank `0` = red side. Piece types use **±1…±7** (see `BoardState.msg`).

If YOLO is missing, the node can still publish a grid of zeros after a successful warp (depending on configuration), but normally you need a weights file.

### 4. What the calibration YAML adds (robot vs image)

**Two different problems:**

| Problem | Solved by |
|---------|-----------|
| Where is the board **in the image**? | **Runtime ArUco** + homography + warp (above). |
| Where is each square **in the robot base frame**? | **`board_to_base_tf`** + **`grid_spacing_mm`** saved by **`calibration_tool`** (teach pendant at four board corners). |

`BoardCalibration` in YAML can also store a **captured homography** from the calibration session; the current `BoardDetector.detect()` path **recomputes H from live ArUco** rather than relying on a stale single-shot homography for localization. The **saved** homography is mainly consistent with the calibration workflow; the critical persistent fields for manipulation are **`board_to_base_tf`**, **`grid_spacing_mm`**, and **`board_origin_mm`**.

### 5. Turn detection (when has the human moved?)

Separate from “where is the board”: **`TurnDetector`** compares successive **grids** after the game manager sets **`/xiangqi/start_watching`**. It waits for a change from the reference grid, then requires the new grid to stay **stable** for **`stability_frames`** ticks (~stability_frames / poll_rate seconds) so a hand hovering over the board does not trigger a false “move done”. **`/xiangqi/human_ready`** bypasses that for demos.

With a wrist camera, the manager requests `/xiangqi/move_to_scan_pose` before publishing `/xiangqi/start_watching` so turn detection runs from a consistent top-down viewpoint. If scan-pose motion fails or is unavailable, watching still starts (best-effort fallback), but detection robustness may degrade.

---

## Game board vs robot: is the structure “correct”?

**Short answer:** The **game rules and grid topology** follow **standard Xiangqi** (9 files × 10 ranks, FEN/pyffish, coordinate moves like `h0g2`). Vision and the game manager use the **same** flat grid convention (`rank * 9 + file`, **rank 0 = Red / robot side**). The arm does **not** load the SVG; it only knows intersections through **calibration** (four taught points + **`grid_spacing_mm`**). If calibration and printing match the real mat, the robot targets the **right intersections**; if not, you get systematic errors even when the AI’s moves are legally correct.

### What the robot is *not* doing

- It does **not** parse the SVG or “see” river/palace lines as semantic features for motion.
- Motion planning assumes a **uniform rectangular lattice**: every intersection is a point separated by **`grid_spacing_mm`** along two axes in a **board frame**, then rotated/translated into **`base_link`** via **`board_to_base_tf`** (`calibration_tool.py`, Kabsch-style fit from four corners).
- **Palace diagonals** and the **river** matter for **pyffish** (legal moves) but are **not** extra geometry in `MoveTranslator`-only **which square index** is being targeted.

So the “board structure” the robot knows is **the discrete grid**, aligned with the **same indices** as chess software, not a CAD clone of the printed artwork.

### Chain that must stay consistent

1. **Rules / AI** - `pyffish` + FEN use standard Xiangqi layout; `game_manager_node._fen_to_grid` builds the same `int8[90]` layout as vision’s `BoardState.grid` for move inference.
2. **Vision** - ArUco warp maps the **physical sheet** to an image where **small y ≈ rank 0 (Red)** and **large y ≈ rank 9 (Black)**, matching the marker→destination corner pairing in `BoardDetector` (see §1). YOLO cells map to **file 0…8, rank 0…9** with that orientation.
3. **Manipulation** - `MoveTranslator` turns coordinate moves into **(file, rank)** indices, then **`grid_to_world`** using **`board_to_base_tf`** and **`grid_spacing_mm`**. The four teach-in corners must be the **actual** intersections **`(0,0), (8,0), (8,9), (0,9)`** in the same coordinate sense as the code (`calibration_tool.CALIBRATION_CORNERS`), in **order**.

### What you must get right in the lab

| Requirement | Why |
|-------------|-----|
| **`grid_spacing_mm`** equals real intersection spacing on the **printed** mat (e.g. from `docs/board_geometry_*.yaml` for that print) | Otherwise the arm’s grid is stretched/shrunk vs reality. |
| Mat is printed/placed so **ArUco IDs 0–3** sit on the corners **assumed** by `board_detector.py` (no accidental 180° rotation vs the generator) | Otherwise vision’s file/rank axes can flip relative to the physical board. |
| Teach pendant touches the **true** corner intersections for calibration, in the **declared order** | Wrong points or order breaks `board_to_base_tf` even if spacing is right. |

If any of these drift, you can still have **legal moves in software** while the gripper goes to the **wrong tile**. Validation: compare a few taught TCP positions with `MoveTranslator` output, and check vision debug overlay vs the physical board.

---

## `vision_node` - main loop

1. **Subscribe** to camera RGB (`camera_topic`, default `/camera/color/image_raw`) with a sensor-style QoS.
2. **Timer** fires at `poll_rate_hz` (default ~3 Hz). On each tick, copy the latest frame and run `_process_image`.
3. **`_process_image` pipeline**
   - **`BoardDetector.detect`**: ArUco → **H** → on success, **`warp_board`**.
   - **`PieceDetector`** (Ultralytics YOLO): run on the **warped** image; map each box to `(file, rank)`; build **`int8[90]`**.
   - Publish **`BoardState`** on `/xiangqi/board_state` and debug BGR on `/xiangqi/debug_image`.
4. **Turn detection** - `TurnDetector.update` after each new grid (see above).
5. **`GetBoardState`** (`get_board_state`) - returns latest cached state or forces a fresh `_process_image` when `force_rescan` is true.

In the full stack, move verification is also scan-pose gated: the planner runs `GoToScanPose` (best-effort) before calling board verification, so `GetBoardState(force_rescan=true)` is requested after arm reposition whenever possible.

If calibration or model files are missing, the node logs warnings; without ArUco or YOLO, perception degrades accordingly.

---

## `BoardDetector` / `BoardCalibration` (summary)

- **`detect(image)`** - ArUco IDs 0–3 → homography **H** + debug overlay.
- **`warp_board(image, H)`** - Top-down 800×890 view for detection.
- **`BoardCalibration`** - Load/save YAML: optional stored **H**, **`board_to_base_tf`**, spacing, origin (see `board_detector.py`).

---

## `PieceDetector`

- YOLO class IDs → piece codes; aggregation into one occupant per cell (`CLASS_MAP` in `piece_detector.py`).
- Normalized board size **must** stay aligned with `BoardDetector` for pixel→grid consistency.

---

## `TurnDetector`

- FSM: `IDLE` → `WATCHING` → `CHANGE_DETECTED` → stable confirm; optional keyboard trigger.

---

## `calibration_tool`

Interactive tool: capture frame (**SPACE**) with all four ArUco visible → **H**; teach TCP at four **grid** corners → solve **`board_to_base_tf`** → write `board_calibration.yaml`. See repository **`docs/board_printing_guide.md`** and **`docs/vision_training_guide.md`**.

---

## Lighting and preprocessing experiments

**ArUco** already runs CLAHE on a grayscale copy when markers are hard to see (`board_detector._preprocess`). **YOLO** runs on the warped board; you can tune that input separately.

### A/B several presets (recommended first)

With the camera and arm at **scan pose**, run the experiment node (does not change live `vision_node` output):

```bash
source /home/rosuser/workspace/install/setup.bash
ros2 run xiangqi_vision vision_preprocess_experiment --ros-args \
  -p model_path:=/home/rosuser/workspace/models/xiangqi_kaggle_v3_best.pt \
  -p calibration_file:=/home/rosuser/workspace/config/board_calibration.yaml \
  -p compare_presets:="['none','clahe','lab_default','high_contrast','saturation']"
```

View the tiled comparison (same QoS style as `/xiangqi/debug_image`):

```bash
ros2 run rqt_image_view rqt_image_view /xiangqi/preprocess_experiment_image
```

Check that frames are publishing (first tick can take **10–30 s** on CPU with 4 presets):

```bash
ros2 topic hz /xiangqi/preprocess_experiment_image
```

If rqt is blank but `debug_image` works: rebuild `xiangqi_vision` (experiment downscales large tiles for rqt), wait for a log line `Published experiment frame #1`, or watch the status image (`No camera frames` / `ArUco failed`).

Each row is **warped | preprocessed | YOLO boxes**, with piece count and mean confidence. Pick the preset that looks best, then enable it on `vision_node` (below).

### Enable preprocessing on `vision_node`

In `xiangqi_bringup/config/vision_config.yaml` (or at runtime):

```yaml
piece_preprocess_enabled: true
piece_preprocess_preset: "lab_default"   # or clahe, gamma_bright, etc.
debug_show_preprocess: true            # debug topic: warped | preprocessed | detections
```

Runtime tuning without restart:

```bash
ros2 param set /vision_node piece_preprocess_enabled true
ros2 param set /vision_node piece_preprocess_preset clahe
ros2 param set /vision_node debug_show_preprocess true
```

Fine-grained overrides (only applied when different from defaults): `piece_gamma`, `piece_brightness`, `piece_contrast`, `piece_use_clahe`, `piece_clahe_clip_limit`, `piece_use_denoise`, `piece_use_sharpen`, `piece_saturation_scale`, `piece_use_white_balance`.

**Presets:** `none`, `bright`, `bright_sharp`, `clahe`, `clahe_strong`, `gamma_bright`, `gamma_dark`, `high_contrast`, `denoise`, `sharpen`, `saturation`, `lab_default`.

### Dim image + soft / blurry pieces (good starting point)

Try **`bright_sharp`** first (brighten → CLAHE on L channel → mild saturation → unsharp mask):

```bash
# Compare against raw and lighter-only on the experiment topic
ros2 run xiangqi_vision vision_preprocess_experiment --ros-args \
  -p model_path:=/home/rosuser/workspace/models/xiangqi_kaggle_v3_best.pt \
  -p calibration_file:=/home/rosuser/workspace/config/board_calibration.yaml \
  -p compare_presets:="['none','bright','bright_sharp','sharpen']"

# Then enable on live vision_node
ros2 param set /vision_node piece_preprocess_enabled true
ros2 param set /vision_node piece_preprocess_preset bright_sharp
ros2 param set /vision_node debug_show_preprocess true
```

Or in `vision_config.yaml`:

```yaml
piece_preprocess_enabled: true
piece_preprocess_preset: "bright_sharp"
debug_show_preprocess: true
```

**Still too dim?** Nudge brighter without a rebuild:

```bash
ros2 param set /vision_node piece_gamma 0.65          # lower = brighter (default in bright_sharp: 0.72)
ros2 param set /vision_node piece_brightness 35      # add flat lift (0–255 scale)
```

**Still soft?** Add sharpen on top of any preset:

```bash
ros2 param set /vision_node piece_use_sharpen true
```

**Too noisy after sharpen?** Do not use `denoise` with `bright_sharp` unless the image is grainy—denoise can make piece edges look softer. Prefer physical focus/lighting first.

| Goal | Preset / params |
|------|-----------------|
| Only brighter | `bright` or `gamma_bright` |
| Brighter + crisper piece edges | `bright_sharp` |
| Local shadow on mat | `clahe` or `lab_default` |
| Fine-tune live | `piece_gamma`, `piece_brightness`, `piece_use_sharpen` |

### Physical lighting tips

- Reduce direct glare on white ArUco print (diffuse overhead light, slight angle).
- Keep exposure consistent at scan pose; re-check after moving lights.
- If ArUco fails but YOLO is fine (or vice versa), tune ArUco (mat visibility) and piece presets separately.
