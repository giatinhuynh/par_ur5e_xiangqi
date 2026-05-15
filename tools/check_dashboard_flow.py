#!/usr/bin/env python3
"""Extended dashboard flow test: mode lock, new game, AI vs AI progress, optional result fields."""
import json
import sys
import time
import urllib.error
import urllib.request

BASE = sys.argv[1] if len(sys.argv) > 1 else "http://127.0.0.1:15001"


def get_state():
    with urllib.request.urlopen(f"{BASE}/api/state", timeout=8) as r:
        return json.load(r)


def post(path, body=None, expect_ok=True):
    data = json.dumps(body or {}).encode()
    req = urllib.request.Request(
        f"{BASE}{path}",
        data=data,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    try:
        with urllib.request.urlopen(req, timeout=8) as r:
            code = r.status
            out = json.load(r)
    except urllib.error.HTTPError as e:
        code = e.code
        try:
            out = json.loads(e.read().decode())
        except Exception:
            out = {"ok": False, "error": str(e)}
    if expect_ok and code >= 400:
        raise RuntimeError(f"{path} HTTP {code}: {out}")
    if expect_ok and out.get("ok") is False:
        raise RuntimeError(f"{path} failed: {out}")
    return out, code


def wait_dashboard(secs=90):
    for _ in range(secs // 3):
        try:
            urllib.request.urlopen(f"{BASE}/", timeout=3)
            return True
        except urllib.error.URLError:
            time.sleep(3)
    return False


def main():
    results = []
    failed = False

    def check(name, cond, detail=""):
        results.append((name, cond, detail))
        if not cond:
            nonlocal failed
            failed = True

    if not wait_dashboard():
        print("FAIL: dashboard not reachable at", BASE)
        return 1

    # Home page loads
    with urllib.request.urlopen(f"{BASE}/", timeout=5) as r:
        html = r.read().decode("utf-8", errors="replace")
    check("home_200", r.status == 200)
    check("html_flow_banner", "flow-banner" in html)
    check("html_result_banner", "game-result-banner" in html)

    s0 = get_state()
    check("has_simulation_mode", "simulation_mode" in s0)
    check("has_game_result_fields", "game_result" in s0 and "game_result_reason" in s0)
    gs0 = (s0.get("game_status") or "idle").lower()
    print("Initial:", {
        "game_status": gs0,
        "game_mode": s0.get("game_mode"),
        "move_count": s0.get("move_count"),
        "pieces": sum(1 for x in s0.get("board_grid", []) if x),
    })

    if gs0 not in ("idle", "game_over"):
        out, code = post("/api/set_mode", {"mode": "ai_vs_human"}, expect_ok=False)
        check("mode_locked_409", code == 409, f"code={code} body={out}")
        post("/api/new_game", {})
        for _ in range(15):
            time.sleep(2)
            s0 = get_state()
            if (s0.get("game_status") or "").lower() in ("idle", "game_over"):
                break
        gs0 = (s0.get("game_status") or "idle").lower()

    if gs0 in ("idle", "game_over"):
        post("/api/set_mode", {"mode": "ai_vs_ai"})
    post("/api/new_game", {})
    s_play = s0
    for _ in range(20):
        time.sleep(2)
        s_play = get_state()
        if s_play.get("move_count", 0) > 0:
            break
    mc = s_play.get("move_count", 0)
    gs = s_play.get("game_status", "")
    check("game_started_moves", mc > 0, f"move_count={mc}")
    check("status_in_progress", gs not in ("idle", ""), f"game_status={gs}")

    # Mode lock mid-game (skip if already verified at startup)
    if gs0 in ("idle", "game_over"):
        out, code = post("/api/set_mode", {"mode": "ai_vs_human"}, expect_ok=False)
        check("mode_locked_409", code == 409, f"code={code} body={out}")

    time.sleep(8)
    s1 = get_state()
    grid_changed = sum(
        1 for a, b in zip(s0.get("board_grid", []), s1.get("board_grid", [])) if a != b
    )
    check("board_updates", grid_changed > 0 or s1.get("move_count", 0) > mc,
          f"grid_diff={grid_changed} moves {mc}->{s1.get('move_count')}")
    check("pieces_on_board", sum(1 for x in s1.get("board_grid", []) if x) >= 8)

    print("After play:", {
        "move_count": s1.get("move_count"),
        "game_status": s1.get("game_status"),
        "game_result": s1.get("game_result"),
        "game_result_reason": s1.get("game_result_reason"),
        "history_len": len(s1.get("move_history") or []),
    })

    # After game over (if any) mode should unlock — poll briefly
    if s1.get("game_status") == "game_over":
        out2, code2 = post("/api/set_mode", {"mode": "ai_vs_human"}, expect_ok=False)
        check("mode_unlock_after_over", code2 == 200, f"code={code2}")

    print("\n=== Test report ===")
    for name, ok, detail in results:
        status = "PASS" if ok else "FAIL"
        line = f"{status}: {name}"
        if detail:
            line += f" ({detail})"
        print(line)

    overall = "PASS" if not failed else "FAIL"
    print("\nTEST_RESULT:", overall)
    return 0 if not failed else 1


if __name__ == "__main__":
    sys.exit(main())
