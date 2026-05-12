# Xiangqi Pieces Guide: Procurement and 3D Printing

## Overview

The lab's UR5e is fitted with an **OnRobot RG2 two-finger parallel gripper** (max open width 110 mm, max force 40 N). This means pieces must be **graspable from the side** — the two fingers close around the circumference of the piece. The key requirement is a **consistent, known diameter** so `grasp_width` in `manipulation_config.yaml` can be tuned once and left fixed.

This guide covers two options for obtaining chess pieces:
1. **Buy standard pieces** (recommended — faster and reliable with RG2)
2. **3D print custom pieces** (if standard pieces are unavailable or a specific diameter is needed)

---

## Option A: Buy Standard Pieces (Recommended)

### Why buy instead of 3D print

- **Consistent diameter** is all that matters for the RG2 — the gripper grabs the side, not the top
- Available from Asian grocery stores, eBay, AliExpress (~$10–20 for a full set)
- No print time, no finishing required

### What to look for

| Property | Requirement |
|---|---|
| Piece diameter | **28–36 mm (30–32 mm optimal)** — critical for setting `grasp_width` |
| Piece height | 8–15 mm — taller pieces are easier to grasp side-on |
| Side surface | Smooth cylinder with no deep grooves — RG2 fingers must seat cleanly |
| Piece style | Traditional round with Chinese characters |
| Colour | White/beige body, red characters (Red set) and black characters (Black set) |
| Material | Hard plastic preferred; wood is fine if the cylinder is uniform |

**Search terms:** "Chinese chess set plastic", "Xiangqi pieces 32mm", "象棋 plastic pieces"

### Recommended test before committing

Manually close the RG2 fingers around one piece (use the EyeBox web interface or `ros2 action send_goal /rg2/set_width`) to the target grasp width. Lift the piece 50 mm, hold 3 seconds, lower and release. Pass rate must be 10/10. If slipping occurs, reduce `grasp_width` by 1–2 mm or increase `grasp_force` slightly.

---

## Option B: 3D Print Custom Pieces

3D printed pieces give full control over diameter, height, and top surface flatness. This is useful if:
- Standard pieces are unavailable locally
- You want a specific colour/contrast for easier detection
- You want a perfectly flat top for **vision** (the VXLab setup uses side grasps with the RG2, not vacuum)

### Recommended print specifications

| Parameter | Value | Notes |
|---|---|---|
| Diameter | 32 mm | Fits well within A2 board cell spacing (~60 mm) |
| Height | 10–12 mm | Enough barrel height for stable **side** grasp with RG2 |
| Top face | Flat, no chamfer | Helps YOLO / top-down vision; RG2 still grips the **side** |
| Bottom face | Flat or slightly concave | Stable on board surface |
| Wall thickness | ≥ 1.5 mm | Rigid enough to not deform when fingers squeeze |
| Number of pieces | 16 Red + 16 Black = 32 total | Standard Xiangqi set |

### Piece type quantities

| Piece | Red | Black |
|---|---|---|
| General (将/帅) | 1 | 1 |
| Advisor (士/仕) | 2 | 2 |
| Elephant (象/相) | 2 | 2 |
| Horse (马) | 2 | 2 |
| Chariot (车) | 2 | 2 |
| Cannon (炮) | 2 | 2 |
| Soldier (卒/兵) | 5 | 5 |
| **Total** | **16** | **16** |

### 3D model design (Fusion 360 / FreeCAD)

Design each piece as a simple cylinder — the RG2 grabs the **side** of the piece, so the critical surface is the cylindrical barrel, not the top face:

```
Cross section (side view):
  ─────────────────  ← flat top (32 mm diameter)
  │               │  12 mm height  ← taller is easier to grasp side-on
  │  smooth barrel│  no ridges on side
  ─────────────────  ← flat bottom with 0.5 mm chamfer to prevent rocking
  
Diameter: 32 mm (must be consistent across all 32 pieces)
```

1. Create a cylinder: diameter 32 mm, height 12 mm
2. Add a small chamfer (0.5 mm) on the **bottom edge only** to prevent rocking
3. Keep the **side surface smooth** — this is where the RG2 fingers make contact
4. Optionally emboss the piece name character on the **top face** for visual identification by the camera (does not affect grip)
5. Export as `.stl`

Alternatively, download free Xiangqi piece STL files from:
- [Thingiverse: search "Xiangqi" or "Chinese chess pieces"](https://www.thingiverse.com/search?q=xiangqi)
- [Printables: search "Xiangqi"](https://www.printables.com/search/models?q=xiangqi)

> **If downloading existing models**: verify the top face is flat. Many decorative models have raised characters on the top — if so, orient them with the character side down and print a flat disc lid over them, or sand the top flat.

### Slicer settings

| Setting | Recommended value | Reason |
|---|---|---|
| Layer height | 0.15–0.20 mm | Good surface finish without excessive print time |
| Infill | 40–60% | Rigid, not hollow (hollow pieces rock on the board) |
| Perimeters / shells | 4 | Strong outer wall for grip |
| Top layers | 5 | Ensures perfectly flat top surface |
| Print orientation | Flat side down | Maximises top surface quality |
| Supports | None needed | Simple disc geometry |
| Estimated time per piece | ~20–35 minutes | ~11–19 hours total for 32 pieces |

### Colour strategy

Two options for visual distinction:

**Option 1: Filament colour (best for detection)**
- Print all Red pieces in **red PLA** (e.g., "Ender Red" or "Silk Red")
- Print all Black pieces in **black PLA**
- Use the `piece_detector.py` colour segmentation as a confidence booster

**Option 2: White base + paint/labels**
- Print all pieces in **white PLA**
- Apply red/black circular sticker labels (30 mm stickers available at office supply stores) for colour distinction
- Label with piece identity for human readability (not required by the detection model)

### Post-processing

1. **Remove the piece from the bed** gently with a spatula
2. **Sand the cylindrical side** (if needed): use 220-grit sandpaper wrapped around a tube. The side surface should feel smooth — this is where the RG2 fingers grip.
3. **Test grip**: use the RG2 to grasp the piece at the configured `grasp_width`. The piece should not rotate or slip when lifted. If it slips, reduce `grasp_width` by 1–2 mm.
4. If printing in a rough filament (e.g., PETG), optionally apply a thin coat of clear lacquer to the side barrel to reduce surface friction variation.

---

## RG2 Grip Compatibility Test

Before buying or printing a full set, do a **grip test** with one prototype piece:

1. Place the piece on the board mat in the robot workspace
2. Move the arm to directly above the piece using the teach pendant or `WaypointMove`
3. Send a `GripperSetWidth` goal: `target_width = 70.0, target_force = 10.0` (open wide)
4. Descend the arm to piece height
5. Send `GripperSetWidth`: `target_width = 28.0, target_force = 15.0` (grasp)
6. Lift 100 mm, hold 3 seconds
7. Lower back to board, send `GripperSetWidth`: `target_width = 50.0` (release)
8. Repeat 10 times — pass rate must be 10/10

Quick ROS command to test gripper manually:
```bash
ros2 action send_goal /rg2/set_width onrobot_rg2_msgs/action/GripperSetWidth \
    "{target_width: 28.0, target_force: 15.0}"
```

If grip fails or piece slips:
- Reduce `grasp_width` by 1–2 mm (more compression)
- Increase `grasp_force` to 20 N
- Check that the piece side surface is smooth and the diameter is consistent

---

## Colour Calibration for Detection

Once you have pieces (bought or printed), capture reference images for the YOLO model training:

- Place pieces on the board mat under the lab camera
- Note the approximate HSV range for each colour under lab lighting:

| Colour | Approximate HSV range (OpenCV scale) |
|---|---|
| Red pieces | H: 0–10 or 170–180, S: 100–255, V: 50–255 |
| Black pieces | H: 0–180, S: 0–50, V: 0–80 |
| Board mat background | H: 20–35, S: 50–150, V: 150–220 |

These ranges can be used for the HSV-based hand presence detection in `vision_node.py`.

---

## Summary: Buy vs Print

| Criterion | Buy Standard | 3D Print |
|---|---|---|
| Time to ready | 1–5 days (shipping) | 11–20 hours print time |
| Cost | $10–20 AUD | ~$2–5 filament |
| RG2 grip reliability | ★★★★★ | ★★★★☆ (with side sanding) |
| Detection consistency | ★★★★☆ | ★★★★★ (controlled colour) |
| Diameter consistency | ★★★★☆ (measure each piece) | ★★★★★ (CAD-exact) |
| Recommendation | **Primary choice** | Backup / if pieces unavailable |

> **After choosing pieces**, measure the actual diameter of 3–5 pieces with digital calipers and update `grasp_width` in `manipulation_config.yaml` to `(measured_diameter - 2)` mm.
