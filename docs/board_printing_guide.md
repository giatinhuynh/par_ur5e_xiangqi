# Board Mat Printing Guide

## Overview

The robot uses a **custom-printed board mat** rather than a commercial Xiangqi set. This provides:
- Exact, known grid dimensions (critical for coordinate calibration)
- Built-in **ArUco fiducial markers** at each corner for automatic camera calibration
- Precise control over piece intersection positions

---

## Step 1: Generate the Board SVG

The board mat is generated programmatically by a Python script to guarantee exact dimensions.

### Requirements

```bash
pip install numpy opencv-contrib-python
```

### Run the generator

```bash
cd /path/to/par_ur5e_xiangqi
python3 tools/generate_board_svg.py
```

This produces two files in `docs/`:
- `board_mat_A2.svg` — recommended for best accuracy
- `board_mat_A3.svg` — if A2 printing is not available

---

## Step 2: Board Dimensions

The A2 board (`594 mm × 420 mm` landscape) uses the following geometry:

| Parameter | Value |
|---|---|
| Page size | A2 landscape: 594 × 420 mm |
| Board grid area | ~484 × 318 mm (centred) |
| File spacing (column gap) | ~60.5 mm |
| Rank spacing (row gap) | ~35.3 mm |
| Piece intersection diameter (target) | ≤ 30 mm circle fits within cell |
| ArUco marker size | 22 mm × 22 mm |
| ArUco dictionary | `DICT_4X4_50` |
| ArUco IDs | 0 (top-left), 1 (top-right), 2 (bottom-right), 3 (bottom-left) |

> **Important:** The grid spacing values above are what get stored in `board_calibration.yaml` as `grid_spacing_mm`. After printing, measure the actual spacing with a ruler and update the YAML if it differs (e.g., due to printer scaling).

### Board orientation

```
  Black side (Human) — rank 9
  ┌─────────────────────────────┐
  │  ArUco 0      ArUco 1       │
  │  a  b  c  d  e  f  g  h  i │  ← files
  │  9                        9 │
  │  8                        8 │
  │         楚 河 / 漢 界       │  ← river
  │  4                        4 │
  │  3                        3 │
  │  2                        2 │
  │  1                        1 │
  │  0                        0 │  ← ranks
  │  ArUco 3      ArUco 2       │
  └─────────────────────────────┘
  Red side (Robot) — rank 0
```

The robot arm base is positioned on the **Red (rank 0) side**.

---

## Step 3: Print Settings

### Recommended: A2 at a print shop (most accurate)

1. Export `board_mat_A2.svg` to PDF using Inkscape or a browser:
   - **Inkscape**: File → Export → PDF, ensure no scaling
   - **Browser**: Open SVG → Print → Save as PDF, set page to A2, margins to zero
2. Send the PDF to a print shop with instructions:
   - **Print size: A2 (594 × 420 mm) — do NOT scale or fit to page**
   - Paper: 160–200 gsm matte coated card stock (heavier = more stable on table)
   - Colour or B&W (B&W sufficient; colour helps distinguish Red vs Black labels)

### Alternative: A3 at lab printer

1. Use `board_mat_A3.svg`
2. Print to A3 at **100% scale, no fit-to-page**
3. Disable any auto-scaling option in the printer dialogue

> **Warning:** Many printers add a small margin by default (~3–5 mm). If the printer adds margins, the physical grid spacing will not match the expected values. Always verify with a ruler after printing and update `board_calibration.yaml` accordingly.

---

## Step 4: Lamination (Strongly Recommended)

Laminate the printed mat before use:

- Use a **cold lamination pouch** (A2 or A3 size) to avoid heat warping
- Cold laminate or ask the print shop to laminate
- Benefits: prevents creasing, protects from suction gripper contact, stays flat on table
- After lamination, the mat should lie perfectly flat — if it curls, place it under a heavy book overnight

---

## Step 5: Secure to the Table

The mat must not move during the game:
- Apply **2–4 small strips of non-permanent double-sided tape** on the underside corners
- Alternatively, use a thin foam mat underneath for grip
- **Do not use tape over the ArUco markers** — the camera must see them clearly
- The mat should remain perfectly stationary; when the RG2 gripper lifts a piece, a slight lateral reaction force is transferred through the piece to the board, so the mat must be firmly secured

---

## Step 6: Measure and Verify

After printing and laminating, verify the geometry before running the calibration tool:

1. Use a ruler to measure the distance between adjacent file lines (e.g., from file `a` to file `b`)
2. Measure between adjacent rank lines (e.g., rank 0 to rank 1)
3. If the measured values differ from the expected values by more than ±1 mm, update `board_calibration.yaml`:

```yaml
grid_spacing_mm: 60.5   # <-- update this with your measured value
```

4. Also verify that the ArUco markers are clearly visible (sharp black squares, no bleeding):
   - Hold the mat under the RealSense camera
   - Run `ros2 run xiangqi_vision vision_node` and check `/xiangqi/debug_image` in RViz
   - All 4 ArUco markers should be detected simultaneously (green outlines visible in the debug image)

---

## Step 7: Graveyard Zones

The graveyard zones are printed on the mat on the **left and right sides** of the board (already included in the SVG). These are the areas where captured pieces are placed by the robot arm.

The graveyard positions are configured in `game_config.yaml`:
```yaml
# Graveyard positions are in robot base_link frame (metres)
# They are set during or after board calibration
```

After calibration, physically mark the graveyard slots with a pencil grid (each slot ~35 × 35 mm). The RG2 gripper needs to release each piece at a well-defined position so captured pieces do not pile up or fall outside the mat — clear slot markings help verify correct placement during testing.

> **Graveyard clearance note:** Make sure captured piece positions are at least 40 mm from the board edge lines, so the robot arm can approach from above and lower the RG2 without the gripper body clipping the board mat.

---

## Checklist Before First Use

- [ ] Mat printed at correct scale (verify with ruler)
- [ ] Mat laminated and flat
- [ ] Mat secured to table in robot workspace
- [ ] All 4 ArUco markers detected by camera (check debug image)
- [ ] Grid spacing recorded in `board_calibration.yaml`
- [ ] Calibration tool run to compute homography and board-to-robot transform
