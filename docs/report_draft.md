# Autonomous Xiangqi on a UR5e Cobot — Final Report Draft

> **Format target:** ≤10 pages, single column, ≥11pt, ≥1.5cm margins.  
> **Figures:** export from [`.cursor/plans/xiangqi_report_diagrams.plan.md`](../.cursor/plans/xiangqi_report_diagrams.plan.md).  
> **Tables:** fill numbers in [`report_diagrams/report_tables.md`](report_diagrams/report_tables.md).  
> Replace `[PHOTO]` with a lab photograph of the physical setup.

---

## 1. Introduction

This project implements a fully autonomous system in which a Universal Robots UR5e cobot with an OnRobot RG2 gripper plays Chinese Chess (Xiangqi) against a human opponent or a second AI engine. The robot perceives the board with an arm-mounted Intel RealSense camera, plans moves with selectable chess engines, and executes pick-and-place manipulation including capture-to-graveyard sequences. The task satisfies the course requirement for a pick-and-place application with an explicit high-level task-planning component (§4.8).

The software is organised as a **three-tier robot architecture** on ROS 2 Humble, deployed in the VXLab Docker workflow. Figure 1 shows the physical and software layout. The contribution of this work is not merely integration of off-the-shelf components: we developed custom perception fusion, human-move inference, board calibration, a behaviour-tree motion sequencer, a custom minimax engine, and typed ROS 2 interfaces that enforce tier boundaries.

**Figure 1.** Physical system setup *(see report diagrams plan, Fig 1)*.

---

## 2. Related Work

**Vision for chess robots.** Star-Robot's chinese-chess-robot project demonstrated YOLO-based piece detection on Xiangqi boards with high mAP; we adopt a similar detection approach with YOLOv8 fine-tuned on lab and public datasets [1].

**Game engines.** Fairy-Stockfish provides UCI search with NNUE evaluation for the Xiangqi variant [2]. We use it as a strong baseline and implement a second, original minimax engine for the undergraduate extended-algorithm requirement.

**Robot architectures.** Classical Sense-Plan-Act and subsumption architectures are widely taught; hierarchical three-tier designs separate deliberation, sequencing, and reactive control [3]. Our architecture choice is justified in Figure 2.

**Lab stack.** Motion execution builds on the VXLab UR5e_Env integration of MoveIt 2, UR drivers, and RG2 Modbus control [4].

---

## 3. Methodology

### 3.1 Robot software architecture

We selected a **three-tier hierarchical architecture** (Figure 2) because Xiangqi requires (i) long-horizon deliberation over FEN and search, (ii) explicit multi-step motion with capture and verification, and (iii) continuous perception and actuator control. Sense-Plan-Act lacks an explicit sequencing layer for capture→move→scan→verify; pure subsumption cannot host chess-scale search without contrived layering.

Figure 3 shows the implemented ROS 2 node graph. **Tier 3 (deliberative):** `game_manager_node` holds authoritative game state, fuses perception for human moves, and dispatches robot moves; `ai_engine_node` answers `GetBestMove`. **Tier 2 (sequencing):** `task_planner_node` runs a `py_trees` behaviour tree only after `AiMoveCommand`. **Tier 1 (reactive):** `vision_node`, `manipulation_node`, `gripper_controller_node`, and `safety_monitor_node` handle sensing and actuation.

Tier boundaries are **structurally enforced** (Figures 13, 20): the game manager never calls manipulation actions directly; it publishes `AiMoveCommand` to the planner, which is the sole publisher of `AiCommandAck` and `AiExecutionResult`. The monotonic `dispatch_id` field prevents stale planner responses from being applied to a newer move.

**Figures 2–3, 13, 20.** Architecture rationale, node graph, dispatch protocol, tier enforcement.

### 3.2 ROS 2 infrastructure (UG extended work)

The `xiangqi_msgs` package defines **15 custom interfaces**: eight messages, five services, and two actions (Figure 19). These types enforce typed information flow between tiers. The `xiangqi_bringup` package provides `xiangqi_system.launch.py` (hardware) and `xiangqi_sim.launch.py`, plus seven YAML configuration files—no parameters are hardcoded in node source.

We extended the VXLab Docker image with Fairy-Stockfish, pyffish, Ultralytics/ONNX Runtime, Flask, and py_trees (Figure 15). This yields reproducible deployment without modifying the base lab image.

**Figures 15, 17, 19.** Deployment stack, package map, message infrastructure.

### 3.3 Vision and board calibration

The vision pipeline (Figure 4) warps each camera frame with ArUco markers (IDs 0–3 at sheet corners) to a normalized 800×890 top-down image. **YOLOv8 ONNX** (`xiangqi_kaggle_v4_best.onnx`) classifies piece type and colour per intersection. A **ResNet-34 occupancy network** publishes a fast binary grid on `/xiangqi/occupancy_state` for reliable empty-cell detection.

`game_manager_node` **fuses** YOLO and occupancy (Figure 5): a cell is occupied if either sensor agrees; type comes from YOLO where occupancy is set. Multi-round collection (six occupancy + three YOLO frames) precedes human-move inference. Temporal stabilizers (`GridStabilizer`, `TurnDetector`) reduce jitter.

Board-to-robot registration uses `calibration_tool` (Figure 11): teach scan pose, capture homography, teach corner/midpoint joint poses for bilinear interpolation, and teach graveyard zones. Output is `board_calibration.yaml`, shared by vision and `move_translator`.

Pre-game **Scan Board** (Figure 14) runs three `GetBoardState` rounds with majority voting; the resulting `prescan_fen` seeds the game manager so play can start from arbitrary legal positions.

**Figures 4–5, 11, 14.** Vision pipeline, fusion, calibration, Scan Board.

### 3.4 Game logic and human move inference

The game manager FSM is shown in Figure 6. After the robot moves, the arm returns to scan pose and vision enters watching mode. Human completion is detected by grid stability or by dashboard **Confirm move** (`/xiangqi/human_ready`) when lighting is poor.

Human-move inference is **original work**: the manager diffs fused grids against the authoritative FEN and matches the change to a **pyffish** legal move (`human_move_grid_tolerance: 12`). Illegal or ambiguous changes enter `PENDING_ILLEGAL` (Figures 9, 18) until the operator confirms or overrides via the dashboard.

During robot motion, occupancy-only watching detects human interference on the board.

**Figures 6, 9, 18.** FSM, human-turn sequence, illegal-move flow.

### 3.5 Task planning and manipulation

Task planning uses a **behaviour tree** (Figure 8) ticking at 10 Hz: setup coordinates from `move_translator`, optional graveyard capture, main pick-and-place, scan pose, verify board against `expected_fen` (up to five retries), then finalize or report failure.

`manipulation_node` implements `PickAndPlace` with eight phases (Figure 10). Board cells use **4-patch bilinear joint interpolation** from taught calibration; vertical segments use Cartesian `/par_moveit/waypoint_move` when needed; the RG2 gripper uses `/rg2/set_width`. At startup, manipulation calls `GetBoardTransform` from vision for ArUco-derived board frame.

Figure 7 shows the end-to-end message sequence for one robot move.

**Figures 7–8, 10.** Robot move sequence, behaviour tree, pick-and-place phases.

### 3.6 AI engines (UG dual algorithm)

Two engines share the `GetBestMove` service and are hot-swappable (Figures 16, 21):

| Engine | Role | Key parameters |
|--------|------|----------------|
| Fairy-Stockfish | Third-party UCI baseline | Skill 1–20, 5 s time limit |
| Custom minimax | Original implementation | Iterative deepening, α–β, depth ~6, 5 s budget |

The minimax engine (`minimax_engine.py`, `evaluation.py`) uses pyffish for legality and receives **full move history** to penalise repetition—draw-by-repetition that Fairy-Stockfish handles internally. Both engines were evaluated in recorded sessions (Table 5).

**Figures 16, 21; Table 5.**

### 3.7 Evaluation methodology

We designed four experiments (Figure 12):

1. **Detection accuracy** — N board snapshots, per-class precision/recall (Table 1).  
2. **Move inference** — M human moves, correct inference rate and fallback usage (Table 2).  
3. **Manipulation** — K pick-and-place trials, grasp success and XY error (Table 3).  
4. **End-to-end games** — complete sessions, cycle time and interventions (Table 4).

Evidence includes dashboard logs, demo video, and MS Teams Project-Evidence folder per submission instructions.

**Figure 12; Tables 1–4.**

---

## 4. Results

*[Fill with measured data before submission. Example structure below.]*

### 4.1 Detection and move inference

Table 1 summarises YOLO detection performance under lab lighting. Occupancy fusion improved empty-cell reliability compared to YOLO alone, reducing false piece counts in move differencing.

Table 2 reports human-move inference over M physical moves. Confirm-move fallback was required in approximately [X]% of trials when `stability_frames` could not be met within the timeout.

### 4.2 Manipulation

Table 3 shows placement error is largest at board corners ([X] mm mean), consistent with joint-interpolation sensitivity at patch boundaries. Grasp success was [X]% over K trials with lab pieces (20 mm diameter, `grasp_width` 18 mm).

### 4.3 End-to-end games and engine comparison

Table 4 summarises complete games. Mean robot move cycle time was [X] s (scan + pick-and-place + verify).

Table 5 compares engines. In one AI-vs-AI session, Fairy-Stockfish (Red, skill 20) defeated custom minimax (Black, 5 s) after 33 plies. In AI-vs-Human, minimax defeated a non-expert human after 26 plies—confirming legal, strategic play.

---

## 5. Analysis and Evaluation

### 5.1 Strengths

- **Complete autonomous loop** on hardware: vision → AI → behaviour tree → verify → human turn.  
- **Explicit three-tier architecture** with typed ROS interfaces and `dispatch_id` safety.  
- **Original components:** fusion-based move inference, calibration pipeline, minimax engine, BT orchestration.  
- **Operator tools:** Scan Board, Confirm move, illegal-move confirmation, e-stop, web dashboard.  
- **Simulation mode** for development without arm drivers.

### 5.2 Limitations

- **YOLO jitter** under variable lighting causes dashboard flicker and occasional verify failures (`verify_grid_tolerance` mitigates but does not eliminate).  
- **Small XY placement offsets** on some cells require calibration re-teach or spacing verification (`grid_spacing_mm` 61.25 on 4×A3 mat).  
- **Human turn latency:** multi-round scan collection trades accuracy for speed (~6 occupancy frames).  
- **Engine strength gap:** custom minimax at depth 6 is weaker than Fairy-Stockfish NNUE at equal time.  
- **Confirm move** remains a practical fallback—not fully hands-free autonomy in all lighting.

### 5.3 Delineation from dependencies

Table 6 (in `report_tables.md`) lists what we implemented versus Ultralytics YOLO, Fairy-Stockfish, pyffish, OpenCV ArUco, MoveIt, and VXLab drivers. Our original work centres on fusion logic, game FSM, calibration/teach-in, behaviour tree, minimax, and `xiangqi_msgs` infrastructure.

---

## 6. Conclusion

We presented an autonomous Xiangqi-playing UR5e system with a justified three-tier ROS 2 architecture, dual perception, behaviour-tree task planning, and two interchangeable AI engines. The system plays full games on hardware with web-based monitoring and rule enforcement. Future work includes quantitative tuning of vision under lighting, placement calibration refinement, and expanded evaluation datasets for the Results section.

---

## References

[1] Star-Robot, *chinese-chess-robot*, GitHub.  
[2] Fairy-Stockfish contributors, *Fairy-Stockfish*, GitHub.  
[3] R. C. Arkin, *Behavior-Based Robotics*; course materials on SPA and three-tier architectures.  
[4] Kibibibit, *UR5e_Env*, VXLab Docker and driver stack.  
[5] Ultralytics, *YOLOv8* documentation.  
[6] pyffish authors, Python bindings for fairy-stockfish variants.

*[Add full BibTeX/APA entries and any papers cited in Related Work.]*

---

## Appendix A — Log of AI tool use

| Date | Tool | Purpose |
|------|------|---------|
| 2026-06 | Cursor / Claude | Code assistance, report diagram generation, documentation drafts |

*[Expand per course requirements: prompts, sections assisted, human verification steps.]*

---

## Appendix B — Figure checklist for PDF assembly

| Fig | Title | Report section |
|-----|-------|----------------|
| 1 | Physical setup | §1 |
| 2 | Architecture choice | §3.1 |
| 3 | Three-tier ROS graph | §3.1 |
| 4 | Vision pipeline | §3.3 |
| 5 | Perception fusion | §3.3 |
| 6 | Game FSM | §3.4 |
| 7 | Robot move sequence | §3.5 |
| 8 | Behaviour tree | §3.5 |
| 9 | Human turn sequence | §3.4 |
| 10 | Pick-and-place phases | §3.5 |
| 11 | Calibration workflow | §3.3 |
| 12 | Evaluation design | §3.7 |
| 15 | Deployment stack | §3.2 |
| 16 | Minimax engine | §3.6 |
| 19 | ROS msg infrastructure | §3.2 |
| 20 | Tier enforcement | §3.1 |
| 21 | Dual AI hot-swap | §3.6 |

Optional: Figs 13, 14, 17, 18 if page budget allows.
