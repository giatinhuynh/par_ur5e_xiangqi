"""
evaluation.py: Hand-crafted evaluation function for the custom Xiangqi minimax engine.

This is the core original implementation:
  - Material scores per piece type
  - Piece-square tables (9x10) for positional bonuses
  - King safety and palace defender bonuses
  - Mobility bonus (number of legal moves)

Score is from Red's perspective: positive = Red advantage, negative = Black advantage.
"""

import numpy as np
from typing import List

# Piece codes (match BoardState.grid encoding)
GENERAL  = 1
ADVISOR  = 2
ELEPHANT = 3
HORSE    = 4
CHARIOT  = 5
CANNON   = 6
SOLDIER  = 7

# Base material values (centipawns, Red perspective)
MATERIAL = {
    GENERAL:  100000,
    ADVISOR:    200,
    ELEPHANT:   200,
    HORSE:      400,
    CHARIOT:    900,
    CANNON:     450,
    SOLDIER:    100,
}

# ------------------------------------------------------------------
# Piece-square tables (9 files x 10 ranks, Red's view: rank 0 = red side)
# Values are BONUSES added to material score (centipawns).
# Tables are for Red pieces; mirrored for Black pieces.
# ------------------------------------------------------------------

# fmt: off

PST_GENERAL = [
    [  0,  0,  0,  5,  5,  5,  0,  0,  0],  # rank 0 (red palace)
    [  0,  0,  0,  5,  5,  5,  0,  0,  0],
    [  0,  0,  0,  5,  5,  5,  0,  0,  0],
    [  0,  0,  0,  0,  0,  0,  0,  0,  0],  # out of palace (invalid, penalised by rules)
    [  0,  0,  0,  0,  0,  0,  0,  0,  0],
    [  0,  0,  0,  0,  0,  0,  0,  0,  0],
    [  0,  0,  0,  0,  0,  0,  0,  0,  0],
    [  0,  0,  0,  0,  0,  0,  0,  0,  0],
    [  0,  0,  0,  0,  0,  0,  0,  0,  0],
    [  0,  0,  0,  0,  0,  0,  0,  0,  0],  # rank 9 (black palace)
]

PST_ADVISOR = [
    [  0,  0,  0,  0, 10,  0,  0,  0,  0],
    [  0,  0,  0, 10,  0, 10,  0,  0,  0],
    [  0,  0,  0,  0, 10,  0,  0,  0,  0],
    [  0,  0,  0,  0,  0,  0,  0,  0,  0],
    [  0,  0,  0,  0,  0,  0,  0,  0,  0],
    [  0,  0,  0,  0,  0,  0,  0,  0,  0],
    [  0,  0,  0,  0,  0,  0,  0,  0,  0],
    [  0,  0,  0,  0,  0,  0,  0,  0,  0],
    [  0,  0,  0,  0,  0,  0,  0,  0,  0],
    [  0,  0,  0,  0,  0,  0,  0,  0,  0],
]

PST_ELEPHANT = [
    [  0,  0, 20,  0,  0,  0, 20,  0,  0],
    [  0,  0,  0,  0,  0,  0,  0,  0,  0],
    [ 20,  0,  0,  0, 20,  0,  0,  0, 20],
    [  0,  0,  0,  0,  0,  0,  0,  0,  0],
    [  0,  0, 20,  0,  0,  0, 20,  0,  0],  # Cannot cross river -- pyffish enforces
    [  0,  0,  0,  0,  0,  0,  0,  0,  0],
    [  0,  0,  0,  0,  0,  0,  0,  0,  0],
    [  0,  0,  0,  0,  0,  0,  0,  0,  0],
    [  0,  0,  0,  0,  0,  0,  0,  0,  0],
    [  0,  0,  0,  0,  0,  0,  0,  0,  0],
]

PST_HORSE = [
    [  0,  4,  8,  8,  8,  8,  8,  4,  0],
    [  4,  8, 16, 20, 20, 20, 16,  8,  4],
    [  8, 16, 24, 28, 28, 28, 24, 16,  8],
    [  8, 20, 28, 32, 36, 32, 28, 20,  8],
    [  8, 20, 28, 36, 40, 36, 28, 20,  8],
    [  8, 20, 24, 32, 36, 32, 24, 20,  8],
    [  4, 12, 20, 24, 28, 24, 20, 12,  4],
    [  0,  4, 12, 16, 20, 16, 12,  4,  0],
    [ -4,  0,  8, 12, 12, 12,  8,  0, -4],
    [-12, -4,  4,  4,  8,  4,  4, -4,-12],
]

PST_CHARIOT = [
    [ 14, 14, 12, 18, 16, 18, 12, 14, 14],
    [ 16, 20, 18, 24, 26, 24, 18, 20, 16],
    [ 12, 12, 12, 18, 15, 18, 12, 12, 12],
    [ 12, 18, 16, 22, 22, 22, 16, 18, 12],
    [ 12, 14, 12, 18, 15, 18, 12, 14, 12],
    [ 12, 16, 14, 20, 20, 20, 14, 16, 12],
    [ 14, 14, 12, 18, 15, 18, 12, 14, 14],
    [ 16, 20, 18, 24, 24, 24, 18, 20, 16],
    [ 14, 17, 15, 19, 19, 19, 15, 17, 14],
    [ 14, 14, 12, 18, 16, 18, 12, 14, 14],
]

PST_CANNON = [
    [  6,  4,  0, -10, -12, -10,  0,  4,  6],
    [  2,  2,  0,  -4, -14,  -4,  0,  2,  2],
    [  2,  6,  4,   0,  -6,   0,  4,  6,  2],
    [  0,  0,  0,  -6, -16,  -6,  0,  0,  0],
    [ -2,  0,  4,   2,  -6,   2,  4,  0, -2],
    [  0,  0,  0,   2,   8,   2,  0,  0,  0],
    [  0,  0, -2,   4,  10,   4, -2,  0,  0],
    [  2,  2,  0,  -4,  -2,  -4,  0,  2,  2],
    [  2,  2,  0,  -4,  -2,  -4,  0,  2,  2],
    [  0,  0, -4,   0,   4,   0, -4,  0,  0],
]

PST_SOLDIER = [
    [  0,  0,  0,  0,  0,  0,  0,  0,  0],  # before crossing
    [  0,  0,  0,  0,  0,  0,  0,  0,  0],
    [  0,  0,  0,  0,  0,  0,  0,  0,  0],
    [  0,  0,  0,  0,  0,  0,  0,  0,  0],
    [  0,  0,  0,  0,  0,  0,  0,  0,  0],
    [  2,  0,  8,  0, 10,  0,  8,  0,  2],  # just crossed river
    [  4,  0, 12,  0, 16,  0, 12,  0,  4],
    [ 16, 36, 28, 36, 40, 36, 28, 36, 16],
    [ 36, 48, 44, 48, 52, 48, 44, 48, 36],
    [ 40, 52, 48, 52, 56, 52, 48, 52, 40],
]

# fmt: on

PST = {
    GENERAL:  PST_GENERAL,
    ADVISOR:  PST_ADVISOR,
    ELEPHANT: PST_ELEPHANT,
    HORSE:    PST_HORSE,
    CHARIOT:  PST_CHARIOT,
    CANNON:   PST_CANNON,
    SOLDIER:  PST_SOLDIER,
}

FILES = 9
RANKS = 10


def evaluate_board(grid: list, legal_moves_red: int, legal_moves_black: int) -> int:
    """
    Static evaluation of a board position.

    Args:
        grid: flat int8[90] array (positive = red, negative = black)
        legal_moves_red: number of legal moves available to Red
        legal_moves_black: number of legal moves available to Black

    Returns:
        Centipawn score from Red's perspective.
    """
    score = 0

    for idx, cell in enumerate(grid):
        if cell == 0:
            continue

        rank = idx // FILES
        file = idx % FILES
        piece_code = abs(cell)
        is_red = cell > 0

        material = MATERIAL.get(piece_code, 0)

        # PST lookup: Red uses rank as-is (rank 0 = red home), Black mirrors rank
        if is_red:
            pst_rank = rank
        else:
            pst_rank = (RANKS - 1) - rank

        pst_table = PST.get(piece_code)
        positional = pst_table[pst_rank][file] if pst_table else 0

        piece_score = material + positional

        if is_red:
            score += piece_score
        else:
            score -= piece_score

    # Mobility bonus: encourages active play
    score += 2 * (legal_moves_red - legal_moves_black)

    return score
