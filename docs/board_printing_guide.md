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

The generator (`tools/generate_board_svg.py`) lays out a **standard 9×10 Xiangqi grid** (same topology as any regulation board: [Wikipedia — Xiangqi](https://en.wikipedia.org/wiki/Xiangqi)): nine files, ten ranks, river, palaces, cannon/pawn marks. **Cells are square** (same step along files and ranks); the grid is centred in the printable interior. **ArUco markers** sit in the **paper corners** with a small inset; a **clear band** separates them from the play area so labels do not overlap markers. The spec line is **centred above the bottom markers**, not on top of them.

Re-run `python3 tools/generate_board_svg.py` after editing the script; nominal sizes below match the current generator output.

### A2 (recommended)

| Parameter | Value (nominal) |
|---|---|
| Page size | 594 × 420 mm (landscape) |
| Corner ArUco size | ~27 mm (scales with page short side) |
| Cell size (square) | **~38.0 mm** (file step = rank step) |
| ArUco dictionary | `DICT_4X4_50` |
| ArUco IDs | 0 TL, 1 TR, 2 BR, 3 BL |

### A3 (smaller print)

| Parameter | Value (nominal) |
|---|---|
| Page size | 420 × 297 mm (landscape) |
| Corner ArUco size | ~26 mm |
| Cell size (square) | **~24.7 mm** |

A3 cells are smaller — use **smaller pieces** or prefer A2 for ~30 mm discs. Each SVG includes a one-line **footer** (centred, grey) with page size, cell mm, and ArUco info.

> **Calibration:** `board_calibration.yaml` uses a single `grid_spacing_mm` — for this mat, set it to the **measured** distance between adjacent intersections (file or rank; they should match). After printing, measure with a ruler and update if the printer scaled the sheet.

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
