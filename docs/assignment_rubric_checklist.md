# Course alignment, report expectations, and rubric checklist

This note expands on [`assignment.md`](../assignment.md) for **§4.8 Pick-and-Place tasks with Cobot UR5e Arm** as implemented here (autonomous Xiangqi). Use it while writing the report and preparing demos; the [repository README](../README.md) stays focused on setup and architecture.

---

## §4.8 Project requirements — mapping to this repo

| # | Requirement | Where it is addressed |
|---|-------------|------------------------|
| 1 | Pick-and-place task **with task planning** | Full game loop: BT (`xiangqi_planner`) sequences perception, AI move, captures, and verification |
| 2 | Configure UR5e software stack | Docker extension + integration with VXLab `par_moveit_config`, `onrobot_rg2_driver`, RealSense |
| 3 | Appropriate **robot software architecture** | Three-tier architecture (see README mermaid diagram); justify in report vs SPA / subsumption |
| 4 | Vision / object recognition | ArUco board warp + YOLOv8 pieces (`xiangqi_vision`) |
| 5 | Manipulation | MoveIt via `WaypointMove` + RG2 actions (`xiangqi_manipulation/manipulation_node.py`) |
| 6 | Task-planning algorithm | `py_trees` / `py_trees_ros` behaviour tree |
| 7 | Demonstrate on UR5e | Live demo + [`docs/README.md`](README.md) and root README runbook |

---

## §2 Shared themes (all groups)

- **Investigation & research** — Related work + citations (YOLO line, Fairy-Stockfish, lab stack, architecture literature).
- **Original implementation** — e.g. board calibration + homography/teach-in, human move detection by differencing + pyffish validation, custom minimax engine, BT orchestration (describe scope vs off-the-shelf ROS).
- **Robot software architecture** — Name tiers, show how nodes enforce information flow; reuse the README mermaid figure in the report.
- **Autonomy** — Vision-based turn stability + optional keyboard/human-ready fallback (`/xiangqi/human_ready`); document any manual steps remaining.
- **Open-ended exploration** — Stronger play (NNUE, more training data), graveyard logic, hand detection, richer safety, etc.
- **Demonstration & analysis** — Strengths/limitations with **evidence** (logs, metrics, short clips, confusion-style tables).

---

## §6 Required report contents (from brief)

- **Methodology** of development and work.
- **(UG)** Extended infrastructure, architecture, and/or implementation (Docker, packages, architecture enforcement, **multiple algorithms**: Fairy-Stockfish vs minimax).
- **Analysis** of capabilities + **evidence** supporting the analysis.
- **(PG)** Experimental evaluation **methodology** + **results** (quantitative/qualitative; limitations made explicit — “publishable” standard per brief).
- **Full references** (literature + software dependencies).
- **Suggested section flow:** Introduction → Related Work → Methodology → Results → Analysis & Evaluation → Conclusion.
- **Format:** ≤10 pages, single column, ≥11pt, ≥1.5cm margins; figures/tables legible at 100% zoom and in B&W print.
- **Rubric (UG report):** methodology/analysis (incl. technical ROS description + architecture justification), extended work block, **writing + referencing**, and **log of AI tool use** — confirm exact wording on Canvas.

---

## Readiness for “full marks” (honest checklist)

**Full course marks are not only code.** They include Week 12 progress, final live demo (implementation + evaluation discussion + distinguishing your work from dependencies), individual contribution, approved **risk assessment**, and the **written report** with evidence. The rubric in [`assignment.md`](../assignment.md) is qualitative (“Excellent / Good / …”); there is no automatic “100% if the repo builds.”

| Area | Implementation in repo | For top band you still need |
|------|-------------------------|-----------------------------|
| §4.8 task + planning | BT + game manager + vision + manipulation | Polished **live** demo on hardware; quick recovery if drivers drop |
| Vision | YOLO + calibration + turn detection | **Report evidence**: accuracy under lab lighting, failure cases |
| Manipulation | Real MoveIt/RG2 integration (with sim fallback) | Tuned grasp heights/widths on **your** pieces; repeatability notes |
| Two algorithms (UG) | Fairy-Stockfish + minimax | Short **comparison** in report (strength/depth/time or game outcomes) |
| Architecture | Clear tiering and launch graph | **Justification** vs other architectures; tie text to the diagram |
| Original work | Several defensible components | Explicitly **delineate** what you built vs Ultralytics/MoveIt/FS |
| Evaluation demo | Dashboard + debug topics | Rubric asks for **discussion of evaluation** with strengths/weaknesses |
| PG-only (§3.3) | Plan lists experiments; not automated in repo | Designed experiment, **repeatable metrics**, results section |

**Bottom line:** The codebase is **substantially aligned** with an “Excellent”-tier *implementation* story for §4.8, but **claiming full marks** still requires **demo + report + admin deliverables** (risk assessment, AI log, citations, PG experiment if applicable) to match the Canvas rubric.
