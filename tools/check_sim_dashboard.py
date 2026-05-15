#!/usr/bin/env python3
"""Quick sim dashboard check: new game, poll state, print PASS/FAIL."""
import json
import sys
import time
import urllib.error
import urllib.request

BASE = "http://127.0.0.1:15001"


def get_state():
    with urllib.request.urlopen(f"{BASE}/api/state", timeout=5) as r:
        return json.load(r)


def post(path, body=None):
    data = json.dumps(body or {}).encode()
    req = urllib.request.Request(
        f"{BASE}{path}",
        data=data,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    with urllib.request.urlopen(req, timeout=5) as r:
        return json.load(r)


def main():
    for _ in range(25):
        try:
            urllib.request.urlopen(f"{BASE}/", timeout=2)
            break
        except urllib.error.URLError:
            time.sleep(3)
    else:
        print("FAIL: dashboard not reachable")
        return 1

    s0 = get_state()
    print("--- Initial ---")
    print("simulation_mode:", s0.get("simulation_mode"))
    print("pieces:", sum(1 for x in s0.get("board_grid", []) if x))
    print("move_count:", s0.get("move_count"))

    post("/api/set_mode", {"mode": "ai_vs_ai"})
    post("/api/new_game", {})
    print("--- New game AI vs AI ---")
    time.sleep(12)

    s1 = get_state()
    pieces1 = sum(1 for x in s1.get("board_grid", []) if x)
    grid0 = s0.get("board_grid", [])
    grid1 = s1.get("board_grid", [])
    diff = sum(1 for a, b in zip(grid0, grid1) if a != b)
    hist = s1.get("move_history") or []

    print("--- After 12s ---")
    print("pieces:", pieces1)
    print("move_count:", s1.get("move_count"))
    print("game_status:", s1.get("game_status"))
    print("fen_prefix:", (s1.get("fen") or "")[:55])
    print("history_len:", len(hist))
    if hist:
        print("last_moves:", [h.get("move") for h in hist[-3:]])
    print("grid_cells_changed:", diff)

    ok = (
        pieces1 >= 8
        and s1.get("move_count", 0) > 0
        and diff > 0
        and urllib.request.urlopen(f"{BASE}/", timeout=2).status == 200
    )
    print("TEST_RESULT:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
