"""
minimax_engine.py: Custom Xiangqi AI using iterative-deepening alpha-beta search.

This is the core original algorithm implementation required by the assignment.
It uses pyffish for legal move generation (correct Xiangqi rules) and implements:
  - Minimax with alpha-beta pruning
  - Iterative deepening with time management
  - Move ordering (captures and checks first)
  - Hand-crafted evaluation function from evaluation.py
"""

from __future__ import annotations
import time
import math
from typing import Optional, Tuple, List

try:
    import pyffish as sf
    PYFFISH_OK = True
except ImportError:
    PYFFISH_OK = False

from .evaluation import evaluate_board, MATERIAL

VARIANT = 'xiangqi'
CHECKMATE_SCORE = 100_000
DRAW_SCORE = 0
INF = 10_000_000

# Xiangqi starting FEN
STARTING_FEN = 'rnbakabnr/9/1c5c1/p1p1p1p1p/9/9/P1P1P1P1P/1C5C1/9/RNBAKABNR w - - 0 1'


class MinimaxEngine:
    """
    Custom Xiangqi AI engine.
    Exposes the same interface as the Fairy-Stockfish wrapper.
    """

    def __init__(self):
        if not PYFFISH_OK:
            raise ImportError('pyffish not installed')
        sf.set_option('VariantPath', '')  # Use built-in variant definitions
        self._nodes_searched = 0
        self._start_time = 0.0
        self._time_limit = 5.0
        self._best_move_so_far: Optional[str] = None

    def get_best_move(
        self,
        fen: str,
        depth: int = 0,
        time_limit: float = 5.0,
    ) -> Tuple[str, str, int, int, float]:
        """
        Search for the best move.

        Returns:
            (best_move, ponder_move, depth_reached, evaluation_cp, thinking_time)
        """
        self._time_limit = time_limit if time_limit > 0 else 5.0
        self._start_time = time.monotonic()
        self._nodes_searched = 0
        self._best_move_so_far = None

        is_red_turn = 'w' in fen.split()[1] if len(fen.split()) > 1 else True
        moves_so_far: List[str] = []

        legal = sf.legal_moves(VARIANT, fen, moves_so_far)
        if not legal:
            return '', '', 0, 0, 0.0

        best_move = ''
        best_eval = -INF
        depth_reached = 0
        self._best_move_so_far = legal[0]

        max_depth = depth if depth > 0 else 6

        for d in range(1, max_depth + 1):
            if self._time_up():
                break
            try:
                move, eval_cp = self._root_search(fen, moves_so_far, d, is_red_turn)
                if move:
                    best_move = move
                    best_eval = eval_cp
                    depth_reached = d
                    self._best_move_so_far = move
            except TimeoutError:
                break

        if not best_move:
            best_move = self._best_move_so_far or legal[0]

        elapsed = time.monotonic() - self._start_time
        ponder = ''
        # Ponder is optional; skip under time pressure so we always return promptly.
        if best_move and depth > 0 and self._time_limit >= 2.0:
            try:
                next_fen = sf.get_fen(VARIANT, fen, [best_move])
                ponder, _ = self._root_search(next_fen, [], 1, not is_red_turn)
            except Exception:
                pass

        return best_move, ponder, depth_reached, int(best_eval), elapsed

    # ------------------------------------------------------------------
    # Internal search
    # ------------------------------------------------------------------

    def _root_search(
        self,
        fen: str,
        moves_so_far: List[str],
        depth: int,
        is_red: bool,
    ) -> Tuple[str, int]:
        legal_moves = sf.legal_moves(VARIANT, fen, moves_so_far)
        if not legal_moves:
            return '', (-CHECKMATE_SCORE if is_red else CHECKMATE_SCORE)

        ordered = self._order_moves(legal_moves, fen, moves_so_far)
        best_move = ordered[0]
        best_val = -INF

        for move in ordered:
            if self._time_up():
                raise TimeoutError
            child_fen = sf.get_fen(VARIANT, fen, moves_so_far + [move])
            val = -self._negamax(child_fen, [], depth - 1, -INF, INF, not is_red)
            if val > best_val:
                best_val = val
                best_move = move
                self._best_move_so_far = move

        return best_move, best_val if is_red else -best_val

    def _negamax(
        self,
        fen: str,
        moves_so_far: List[str],
        depth: int,
        alpha: int,
        beta: int,
        is_red: bool,
    ) -> int:
        """Negamax with alpha-beta pruning. Score is always from current player's perspective."""
        self._nodes_searched += 1
        if self._time_up():
            raise TimeoutError

        legal_moves = sf.legal_moves(VARIANT, fen, moves_so_far)

        # Terminal conditions
        # Terminal conditions
        if not legal_moves:
            # If no legal moves, it's either checkmate or stalemate.
            # We check the game result to see if someone won.
            res = sf.game_result(VARIANT, fen, moves_so_far)
            if res in ("1-0", "0-1"):
                return -CHECKMATE_SCORE + len(moves_so_far)
            return DRAW_SCORE  # Stalemate

        if depth == 0:
            return self._quiescence(fen, moves_so_far, alpha, beta, is_red)

        ordered = self._order_moves(legal_moves, fen, moves_so_far)
        best_val = -INF

        for move in ordered:
            if self._time_up():
                raise TimeoutError
            child_fen = sf.get_fen(VARIANT, fen, moves_so_far + [move])
            val = -self._negamax(child_fen, [], depth - 1, -beta, -alpha, not is_red)
            best_val = max(best_val, val)
            alpha = max(alpha, val)
            if alpha >= beta:
                break  # Beta cutoff

        return best_val

    def _quiescence(
        self,
        fen: str,
        moves_so_far: List[str],
        alpha: int,
        beta: int,
        is_red: bool,
    ) -> int:
        """Quiescence search: only examine captures to avoid horizon effect."""
        grid = self._fen_to_grid(fen)
        legal_moves = sf.legal_moves(VARIANT, fen, moves_so_far)

        legal_red = len(legal_moves) if is_red else 0
        legal_black = 0 if is_red else len(legal_moves)
        stand_pat = evaluate_board(grid, legal_red, legal_black)
        if not is_red:
            stand_pat = -stand_pat

        if stand_pat >= beta:
            return beta
        alpha = max(alpha, stand_pat)

        captures = [m for m in legal_moves if self._is_capture(m, fen, moves_so_far)]
        for move in captures:
            if self._time_up():
                raise TimeoutError
            child_fen = sf.get_fen(VARIANT, fen, moves_so_far + [move])
            val = -self._quiescence(child_fen, [], -beta, -alpha, not is_red)
            if val >= beta:
                return beta
            alpha = max(alpha, val)

        return alpha

    # ------------------------------------------------------------------
    # Move ordering
    # ------------------------------------------------------------------

    def _order_moves(self, moves: List[str], fen: str, moves_so_far: List[str]) -> List[str]:
        """Score moves for ordering: captures > checks > quiet."""
        scored = []
        for move in moves:
            score = 0
            if self._is_capture(move, fen, moves_so_far):
                score += 10000 + self._capture_gain(move, fen)
            scored.append((score, move))
        scored.sort(key=lambda x: -x[0])
        return [m for _, m in scored]

    def _is_capture(self, move: str, fen: str, moves_so_far: List[str]) -> bool:
        """Check if a move captures an opponent piece."""
        try:
            grid = self._fen_to_grid(fen)
            to_file = ord(move[2]) - ord('a')
            to_rank = int(move[3])
            idx = to_rank * 9 + to_file
            target = grid[idx] if idx < len(grid) else 0
            return target != 0
        except Exception:
            return False

    def _capture_gain(self, move: str, fen: str) -> int:
        """Estimate material gain from a capture (for move ordering)."""
        try:
            grid = self._fen_to_grid(fen)
            to_file = ord(move[2]) - ord('a')
            to_rank = int(move[3])
            idx = to_rank * 9 + to_file
            target = abs(grid[idx]) if idx < len(grid) else 0
            return MATERIAL.get(target, 0)
        except Exception:
            return 0

    # ------------------------------------------------------------------
    # Utilities
    # ------------------------------------------------------------------

    def _time_up(self) -> bool:
        return (time.monotonic() - self._start_time) >= self._time_limit

    @staticmethod
    def _fen_to_grid(fen: str) -> list:
        """
        Parse Xiangqi FEN into a flat int8[90] grid matching BoardState encoding.
        Red pieces = positive, Black = negative.
        """
        PIECE_CODES = {
            'K': 1, 'A': 2, 'B': 3, 'N': 4, 'R': 5, 'C': 6, 'P': 7,  # Red (uppercase)
            'k': 1, 'a': 2, 'b': 3, 'n': 4, 'r': 5, 'c': 6, 'p': 7,  # Black (lowercase)
        }
        grid = [0] * 90
        board_part = fen.split()[0]
        ranks = board_part.split('/')
        # FEN rank 0 = rank 9 (black side), rank 9 = rank 0 (red side)
        for fen_rank_idx, rank_str in enumerate(ranks):
            board_rank = 9 - fen_rank_idx  # Convert FEN rank index to board rank
            file_idx = 0
            for ch in rank_str:
                if ch.isdigit():
                    file_idx += int(ch)
                else:
                    code = PIECE_CODES.get(ch, 0)
                    if code:
                        idx = board_rank * 9 + file_idx
                        grid[idx] = code if ch.isupper() else -code
                    file_idx += 1
        return grid

    @property
    def nodes_searched(self) -> int:
        return self._nodes_searched
