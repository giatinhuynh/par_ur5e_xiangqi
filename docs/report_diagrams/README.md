# Report diagrams — figure index and export guide

**Preview all figures rendered inline (Cursor plan style):** [`.cursor/plans/xiangqi_report_diagrams.plan.md`](../../.cursor/plans/xiangqi_report_diagrams.plan.md) — open in Cursor to see every Mermaid diagram like the system plan.

Mermaid sources for the **COSC2781 final report** (≤10 pages). Each figure maps to a report section and rubric element (architecture justification, methodology, original work, evaluation).

## How to export for PDF (legible at 100% zoom, B&W print)

**Pre-exported files (ready to insert):**

| Format | Location |
|--------|----------|
| PNG (1600 px) | [`exports/`](exports/) — `fig01_…` through `fig21_…` |
| SVG (vector) | [`exports/svg/`](exports/svg/) |
| ZIP | [`report_figures_png.zip`](report_figures_png.zip), [`report_figures_svg.zip`](report_figures_svg.zip) |

Re-export after editing `.mmd` sources:

```bash
cd docs/report_diagrams && ./export_all.sh
```

Uses system Google Chrome on macOS (`PUPPETEER_EXECUTABLE_PATH`). Manual alternative: [mermaid.live](https://mermaid.live).

---

## Recommended figures for a 10-page report

Use **8–12 figures** total (tables count toward the page limit). Priority order:

| Fig | File | Report section | Rubric / purpose |
|-----|------|----------------|------------------|
| **1** | `fig01_physical_setup.mmd` | Introduction / Methodology | Physical system context |
| **2** | `fig02_architecture_choice.mmd` | Methodology — architecture | **UG:** justify three-tier vs SPA / subsumption |
| **3** | `fig03_three_tier_ros.mmd` | Methodology — architecture | **Core figure** — node tiers and data flow |
| **4** | `fig04_vision_pipeline.mmd` | Methodology — vision | ArUco → YOLO + occupancy |
| **5** | `fig05_perception_fusion.mmd` | Methodology — original work | Dual-sensor fusion in game manager |
| **6** | `fig06_game_fsm.mmd` | Methodology — game logic | Game manager FSM |
| **7** | `fig07_robot_move_sequence.mmd` | Methodology — integration | End-to-end robot move |
| **8** | `fig08_behaviour_tree.mmd` | Methodology — task planning | **§4.8 task planning** requirement |
| **9** | `fig09_human_turn_sequence.mmd` | Methodology | Human turn + illegal-move path |
| **10** | `fig10_pick_place_phases.mmd` | Methodology — manipulation | Pick-and-place phases |
| **11** | `fig11_calibration_workflow.mmd` | Methodology — calibration | Original calibration pipeline |
| **12** | `fig12_evaluation_design.mmd` | Results / Analysis | **Demo evaluation** discussion |
| 13 | `fig13_dispatch_protocol.mmd` | Methodology (appendix OK) | `dispatch_id` protocol detail |
| 14 | `fig14_scan_board_flow.mmd` | Methodology | Scan Board / prescan FEN |
| 15 | `fig15_deployment_stack.mmd` | Methodology — infrastructure | **UG:** ROS 2 / Docker infrastructure |
| 16 | `fig16_minimax_search.mmd` | Methodology — algorithms | **UG:** second algorithm |
| 17 | `fig17_ros_package_map.mmd` | Methodology (optional) | Package layout |
| 18 | `fig18_illegal_move_flow.mmd` | Analysis (optional) | Safety / rule enforcement |

---

## Suggested page placement (single-column report)

```
§1 Introduction          → Fig 1 (photo of setup + Fig 1 diagram, or diagram only)
§2 Related Work          → (text + citations; optional small comparison table)
§3 Methodology
  3.1 Architecture       → Fig 2, Fig 3
  3.2 Infrastructure      → Fig 15, Fig 17
  3.3 Vision              → Fig 4, Fig 5, Fig 11
  3.4 Game & AI           → Fig 6, Fig 16
  3.5 Task planning       → Fig 8, Fig 7
  3.6 Manipulation        → Fig 10
  3.7 Human interaction   → Fig 9, Fig 14
§4 Results               → Tables (detection %, move times) + Fig 12
§5 Analysis & Evaluation → Fig 12 (reuse), limitations bullet list
§6 Conclusion            → (no new figures)
```

---

## Caption templates (copy into report)

Use consistent numbering: **Figure 1.** … **Table 1.** …

Examples:

- *Figure 3. Three-tier robot software architecture implemented in ROS 2 Humble. Deliberative nodes manage game state and AI; the sequencing layer executes behaviour-tree motion; reactive nodes handle perception, manipulation, and safety.*
- *Figure 8. Behaviour tree executed by `task_planner_node` after each `AiMoveCommand`. Verify step retries up to five times before reporting board verification failure.*

Full captions are in [`all_diagrams.md`](all_diagrams.md).

---

## What is **not** a diagram (but rubric still needs it)

| Deliverable | Where |
|-------------|--------|
| Detection accuracy **table** | Results §4 |
| FSF vs minimax comparison **table** | Results / UG extended work |
| Move cycle time **table** | Results |
| Approved **risk assessment** | Separate admin submission |
| **AI tool use log** | Report appendix or dedicated section |
| Demo **video / photos** | ZIP or MS Teams Project-Evidence |

See [`assignment_rubric_checklist.md`](../assignment_rubric_checklist.md) for the honest full-marks checklist.
