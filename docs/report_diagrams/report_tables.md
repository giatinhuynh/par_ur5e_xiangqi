# Report tables (fill with your measured data)

Copy into the **Results** section. Replace `—` with values from lab runs. These complement the diagrams in [`README.md`](README.md).

---

## Table 1. YOLO piece detection accuracy (Experiment 1)

| Piece class | Precision | Recall | F1 | Notes |
|-------------|-----------|--------|-----|-------|
| Red General (K) | — | — | — | |
| Red Advisor (A) | — | — | — | |
| … (14 classes) | | | | |
| **Macro avg** | — | — | — | N = ___ snapshots |

*Caption: Per-class detection metrics on the lab board under nominal lighting. Ground truth from manual labels or high-confidence consensus scans.*

---

## Table 2. Human move inference reliability (Experiment 2)

| Metric | Value |
|--------|-------|
| Trials (human moves) | M = — |
| Correct move inferred | — (—%) |
| Required Confirm move fallback | — (—%) |
| False illegal-move alerts | — |
| Ambiguous (no unique legal match) | — |

---

## Table 3. Manipulation placement error (Experiment 3)

| Board region | Trials | Grasp success | Mean XY error (mm) | Max XY error (mm) |
|--------------|--------|---------------|--------------------|--------------------|
| Corners | — | —% | — | — |
| Centre | — | —% | — | — |
| Graveyard capture | — | —% | — | — |
| **Overall** | K = — | —% | — | — |

---

## Table 4. End-to-end game performance (Experiment 4)

| Game # | Mode | Moves | Avg robot cycle (s) | Vision failures | Operator interventions |
|--------|------|-------|---------------------|-----------------|------------------------|
| 1 | AI vs Human | — | — | — | — |
| … | | | | | |
| **Mean** | | | — | — | — |

---

## Table 5. AI engine comparison (UG extended work)

| Engine | Mean depth / ply | Mean think time (s) | Mean eval swing (cp) | Win rate vs other (sim) |
|--------|------------------|---------------------|----------------------|-------------------------|
| Fairy-Stockfish (skill 20) | — | 5.0 (cap) | — | — |
| Custom minimax | — | 5.0 (cap) | — | — |

*Run engine-vs-engine or fixed-position suites in simulation; cite Fairy-Stockfish and your implementation separately in references.*

---

## Table 6. Original work vs dependencies (for demo + report)

| Component | Original implementation | Third-party / lab |
|-----------|-------------------------|-----------------|
| Piece detector | Integration, training pipeline, class map | Ultralytics YOLOv8 |
| Occupancy layer | ResNet training + fusion logic | PyTorch, torchvision |
| Game rules / FEN | Human move inference, FSM | pyffish |
| AI search | Minimax + evaluation | Fairy-Stockfish (baseline) |
| Motion | BT, move_translator, calibration | MoveIt, UR drivers, RG2 driver |
| Board localization | ArUco homography + teach-in | OpenCV |
