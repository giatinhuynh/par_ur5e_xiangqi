"""
Resolve AI engine moves to pyffish-legal coordinate notation.

Fairy-Stockfish (subprocess) and pyffish (library) should agree, but version skew or
rank-10 squares (e.g. a10a9) can make strict string matching fail. Match by parsed
from/to squares before falling back.
"""

from __future__ import annotations

from typing import Optional

try:
    import pyffish as sf
    PYFFISH_OK = True
except ImportError:
    PYFFISH_OK = False

VARIANT = 'xiangqi'


def parse_square(move: str, idx: int) -> tuple[tuple[str, int], int] | None:
    """Parse file+rank at move[idx:]. Ranks 1–10 (rank 10 uses two digits)."""
    if idx >= len(move):
        return None
    f = move[idx]
    if f < 'a' or f > 'i':
        return None
    if idx + 2 < len(move) and move[idx + 1] == '1' and move[idx + 2] == '0':
        return (f, 10), idx + 3
    if idx + 1 < len(move) and move[idx + 1].isdigit():
        return (f, int(move[idx + 1])), idx + 2
    return None


def parse_move(move: str) -> tuple[tuple[str, int], tuple[str, int]] | None:
    """Return ((from_file, from_rank), (to_file, to_rank)) or None."""
    if not move:
        return None
    a = parse_square(move, 0)
    if not a:
        return None
    (from_sq, next_i) = a
    b = parse_square(move, next_i)
    if not b:
        return None
    (to_sq, end_i) = b
    if end_i != len(move):
        return None
    return from_sq, to_sq


def square_to_index(file_ch: str, rank_1based: int) -> int:
    """Board grid index (rank 0 = UCI rank 1)."""
    return (rank_1based - 1) * 9 + (ord(file_ch) - 97)


def origin_grid_index(move: str) -> int | None:
    """Grid index of the move origin square, or None if unparsable."""
    parsed = parse_move(move)
    if not parsed:
        return None
    from_sq, _ = parsed
    return square_to_index(from_sq[0], from_sq[1])


def move_critical_indices(move: str) -> set[int]:
    """Grid indices for from/to squares of a coordinate move."""
    parsed = parse_move(move)
    if not parsed:
        return set()
    (from_sq, to_sq) = parsed
    return {
        square_to_index(from_sq[0], from_sq[1]),
        square_to_index(to_sq[0], to_sq[1]),
    }


def resolve_to_legal_move(
    fen: str,
    proposed: str,
    legal: Optional[list[str]] = None,
) -> tuple[str, bool]:
    """
    Map an engine move onto a pyffish-legal move.

    Returns:
        (resolved_move, was_exact) - was_exact True if proposed was already legal.
    """
    if not PYFFISH_OK:
        return proposed, True

    if legal is None:
        legal = sf.legal_moves(VARIANT, fen, [])

    if not legal:
        return proposed, False

    if proposed in legal:
        return proposed, True

    parsed = parse_move(proposed)
    if parsed:
        for m in legal:
            if parse_move(m) == parsed:
                return m, False

        from_sq, to_sq = parsed
        same_from = [m for m in legal if parse_move(m) and parse_move(m)[0] == from_sq]
        if len(same_from) == 1:
            return same_from[0], False
        for m in same_from:
            if parse_move(m) and parse_move(m)[1] == to_sq:
                return m, False

    # Last resort: try applying via pyffish (some builds accept moves not listed)
    try:
        sf.get_fen(VARIANT, fen, [proposed])
        if proposed not in legal:
            for m in legal:
                try:
                    if sf.get_fen(VARIANT, fen, [m]) == sf.get_fen(VARIANT, fen, [proposed]):
                        return m, False
                except Exception:
                    continue
    except Exception:
        pass

    return legal[0], False


def eval_to_red_perspective(fen: str, eval_cp: int) -> int:
    """UCI scores are for side-to-move; normalize to Red-positive centipawns."""
    parts = fen.split()
    if len(parts) > 1 and parts[1] == 'b':
        return -int(eval_cp)
    return int(eval_cp)
