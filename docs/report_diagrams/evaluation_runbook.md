# Evaluation data collection — lab runbook

Run these during a lab session to fill [`report_tables.md`](report_tables.md) and the Results section of [`report_draft.md`](../report_draft.md). Due: **Sunday June 21, 2026**.

## Before you start

```bash
arm_drivers          # terminal 1
moveit_config_driver # terminal 2
ros2 launch xiangqi_bringup xiangqi_system.launch.py
```

Open dashboard `http://10.234.7.84:5000/` (or lab host). Record screen for MS Teams **Project-Evidence**.

---

## Experiment 1 — Detection accuracy (Table 1)

**Goal:** N ≥ 20 board snapshots with ground-truth FEN.

1. Place 10–15 distinct board positions (opening, midgame, captures, sparse endgame).
2. For each position: **Scan Board** twice; if FEN matches, label as ground truth.
3. Export vision output:
   ```bash
   ros2 service call /vision_node/get_board_state xiangqi_msgs/srv/GetBoardState "{force_rescan: true}"
   ```
4. Compare `grid` to FEN-derived grid (or manual cell checklist on printed sheet).
5. Tally per-class TP/FP/FN for 14 piece types → precision/recall in spreadsheet.

**Evidence:** screenshots of `debug_image` topic; confusion matrix table in report.

---

## Experiment 2 — Move inference (Table 2)

**Goal:** M ≥ 20 human moves in AI vs Human mode.

| Trial | Move inferred correctly? | Used Confirm move? | Illegal alert? | Notes |
|-------|--------------------------|--------------------|----------------|-------|
| 1 | Y/N | Y/N | Y/N | |

Procedure per move:
1. Human plays one legal move on physical board.
2. Wait for auto detection OR press Confirm move.
3. Note whether dashboard FEN matches intended move.

**Metrics:** correct rate = correct/M; fallback rate = Confirm/M.

---

## Experiment 3 — Manipulation (Table 3)

**Goal:** K ≥ 20 pick-and-place operations.

Use test harness or full games:

```bash
ros2 run xiangqi_manipulation test_moveit_move --move e5 e7
```

Record for corners (a0, i9), centre (e5), and graveyard captures:
- Grasp succeeded? (piece lifted, not dropped)
- Measure XY error: mark intended intersection on mat, measure piece centre offset with ruler (mm).

Also log from BT: `BOARD_VERIFY_FAILED` count per session.

---

## Experiment 4 — End-to-end games (Table 4)

**Goal:** ≥ 3 complete games (mix AI vs Human and AI vs AI).

Per game log from dashboard / `move_history`:
- Total plies
- Mean time per **robot** move (wall clock from command to next human turn)
- Count of Confirm move, verify failures, e-stop, illegal alerts

**Demo clip:** one full AI vs Human game (≤5 min highlight reel for submission).

---

## Experiment 5 — Engine comparison (Table 5)

In **simulation** (faster iteration):

```bash
tools/wsl_docker_sim_run.sh   # or xiangqi_sim.launch.py
```

1. AI vs AI: FSF (red, skill 20) vs minimax (black), 5 s each — record result and plies.
2. Repeat 5–10 games; note win rate.
3. Optional: fixed FEN suite — same positions, compare move choice and eval cp.

---

## Quick ros2 logging

```bash
# Game status trace
ros2 topic echo /xiangqi/game_status --once

# Move history
ros2 topic echo /xiangqi/move_history

# Save a bag for one game (optional)
ros2 bag record /xiangqi/board_state /xiangqi/game_status /xiangqi/move_history /xiangqi/ai_execution_result
```

---

## Submission checklist

- [ ] Tables 1–5 filled with real numbers  
- [ ] Figures 1–12 exported from [report diagrams plan](../../.cursor/plans/xiangqi_report_diagrams.plan.md)  
- [ ] Lab photo for Figure 1  
- [ ] Demo video on MS Teams if ZIP > 100 MB  
- [ ] `report_draft.md` → PDF (≤10 pages)  
- [ ] AI tool log appendix  
- [ ] Approved risk assessment  
- [ ] Group ZIP: `<group_name>.zip` with full software + PDF + evidence
