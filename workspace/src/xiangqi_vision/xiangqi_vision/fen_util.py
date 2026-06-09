"""FEN ↔ grid helpers shared by vision (BoardState.fen) and verification."""

from __future__ import annotations

PIECE_CODES = {
    'K': 1, 'A': 2, 'B': 3, 'N': 4, 'R': 5, 'C': 6, 'P': 7,
    'k': 1, 'a': 2, 'b': 3, 'n': 4, 'r': 5, 'c': 6, 'p': 7,
}


def fen_to_grid(fen: str) -> list:
    """Parse Xiangqi FEN board part into int8[90] grid (rank 0 = Red home).

    FEN lists ranks top-to-bottom (rank 9 first, rank 0 last).
    Uppercase letters = Red pieces (positive codes), lowercase = Black (negative).
    """
    board_part = fen.split()[0]
    rows = board_part.split('/')  # rows[0]=rank9 (Black home), rows[9]=rank0 (Red home)
    grid = [0] * 90
    for i, row in enumerate(rows):
        rank = 9 - i
        f = 0
        for ch in row:
            if ch.isdigit():
                f += int(ch)
            else:
                code = PIECE_CODES.get(ch, 0)
                grid[rank * 9 + f] = code if ch.isupper() else -code
                f += 1
    return grid

PIECE_CHARS = {1: 'K', 2: 'A', 3: 'B', 4: 'N', 5: 'R', 6: 'C', 7: 'P'}

DEFAULT_FEN_TAIL = 'w - - 0 1'


def grid_to_fen(grid: list, original_fen: str = DEFAULT_FEN_TAIL) -> str:
    """Build a Xiangqi FEN from int8[90] grid (rank 0 = Red home)."""
    rows = []
    for rank in range(9, -1, -1):
        row = ''
        empty = 0
        for file in range(9):
            val = grid[rank * 9 + file]
            if val == 0:
                empty += 1
            else:
                if empty:
                    row += str(empty)
                    empty = 0
                ch = PIECE_CHARS.get(abs(val), '?')
                row += ch if val > 0 else ch.lower()
        if empty:
            row += str(empty)
        rows.append(row)
    board_part = '/'.join(rows)
    parts = (original_fen or DEFAULT_FEN_TAIL).split()
    if len(parts) >= 1:
        parts[0] = board_part
        return ' '.join(parts)
    return f'{board_part} {DEFAULT_FEN_TAIL}'
