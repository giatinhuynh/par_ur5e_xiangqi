#!/usr/bin/env python3
"""Verify dashboard game result vs pyffish native end detection."""
from __future__ import annotations

import json
import sys
import urllib.request

try:
    import pyffish as sf
except ImportError:
    print("SKIP: pyffish not installed")
    sys.exit(0)

sf.set_option("VariantPath", "")
V = "xiangqi"
START = "rnbakabnr/9/1c5c1/p1p1p1p1p/9/9/P1P1P1P1P/1C5C1/9/RNBAKABNR w - - 0 1"


def classify(fen: str, moves: list[str]) -> tuple[str, str]:
    """Mirror game_manager_node._check_game_over."""
    is_red_turn = "w" in fen.split()[1]
    immediate, imm_val = sf.is_immediate_game_end(V, fen, [])
    if immediate:
        if imm_val > 0:
            r = "red_wins" if is_red_turn else "black_wins"
            return r, "checkmate"
        if imm_val < 0:
            r = "black_wins" if is_red_turn else "red_wins"
            return r, "checkmate"
        return "draw", "stalemate"

    optional, opt_val = sf.is_optional_game_end(V, START, moves)
    if optional:
        if opt_val > 0:
            r = "red_wins" if is_red_turn else "black_wins"
            return r, "perpetual_rule"
        if opt_val < 0:
            r = "black_wins" if is_red_turn else "red_wins"
            return r, "perpetual_rule"
        return "draw", "draw_by_repetition"

    try:
        red_insuf, blk_insuf = sf.has_insufficient_material(V, fen, [])
        if red_insuf and blk_insuf:
            return "draw", "insufficient_material"
    except AttributeError:
        pass

    return "ongoing", ""


def main() -> int:
    base = sys.argv[1] if len(sys.argv) > 1 else "http://127.0.0.1:5000"
    with urllib.request.urlopen(f"{base}/api/state", timeout=8) as r:
        s = json.load(r)

    fen = s.get("fen") or ""
    moves = [h.get("move", "") for h in (s.get("move_history") or []) if h.get("move")]
    dash_result = (s.get("game_result") or "ongoing").lower()
    dash_reason = (s.get("game_result_reason") or "").lower()

    print("Dashboard:")
    print(f"  move_count={s.get('move_count')} status={s.get('game_status')}")
    print(f"  result={dash_result} reason={dash_reason}")
    print(f"  history_len={len(moves)}")
    if moves:
        print(f"  last 8: {' '.join(moves[-8:])}")

    exp_result, exp_reason = classify(fen, moves)
    print("\nPyffish (game_manager rules):")
    print(f"  result={exp_result} reason={exp_reason}")

    imm, iv = sf.is_immediate_game_end(V, fen, [])
    opt, ov = sf.is_optional_game_end(V, START, moves)
    print(f"  is_immediate_game_end={imm} value={iv}")
    print(f"  is_optional_game_end={opt} value={ov}")

    ok = (exp_result == dash_result) and (
        not dash_reason or exp_reason == dash_reason or dash_reason in exp_reason
    )
    if dash_result == "ongoing":
        ok = exp_result == "ongoing"
    print("\nMATCH:" if ok else "\nMISMATCH:", ok)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
