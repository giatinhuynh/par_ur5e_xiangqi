#!/usr/bin/env python3
"""Build-time check: Fairy-Stockfish UCI bestmove must be in pyffish legal_moves."""
from __future__ import annotations

import subprocess
import sys

import pyffish as sf

sf.set_option("VariantPath", "")
V = "xiangqi"


def fsf_best(fen: str, movetime_ms: int = 400) -> str:
    script = (
        "uci\n"
        "setoption name UCI_Variant value xiangqi\n"
        "isready\n"
        f"position fen {fen}\n"
        f"go movetime {movetime_ms}\n"
        "quit\n"
    )
    out = subprocess.run(
        ["fairy-stockfish"],
        input=script,
        capture_output=True,
        text=True,
        timeout=15,
        check=False,
    )
    for line in out.stdout.splitlines():
        if line.startswith("bestmove"):
            parts = line.split()
            if len(parts) >= 2 and parts[1] != "(none)":
                return parts[1]
    return ""


def check_position(fen: str, label: str) -> None:
    legal = sf.legal_moves(V, fen, [])
    if not legal:
        raise SystemExit(f"{label}: no legal moves")
    mv = fsf_best(fen)
    if not mv:
        raise SystemExit(f"{label}: fairy-stockfish returned no bestmove")
    if mv not in legal:
        raise SystemExit(
            f"{label}: FSF {mv!r} not in pyffish legal set "
            f"(n={len(legal)}, sample={legal[:8]})"
        )
    print(f"OK {label}: {mv!r}")


def main() -> int:
    print(f"pyffish module: {sf.__file__}")
    start = sf.start_fen(V)
    check_position(start, "startpos")

    fen = start
    for ply in range(1, 4):
        legal = sf.legal_moves(V, fen, [])
        fen = sf.get_fen(V, fen, [legal[0]])
        check_position(fen, f"after {ply} plies")

    print("FSF/pyffish alignment check passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
