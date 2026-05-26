"""
mcts_engine.py: Monte Carlo Tree Search engine for Xiangqi.

This engine uses pyffish for legal Xiangqi move generation and runs a
time-bounded UCT search. Rollouts are intentionally lightweight: random play
to a depth cap, then the existing hand-crafted evaluation function.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
import random
import time
from typing import Optional, Tuple

try:
    import pyffish as sf
    PYFFISH_OK = True
except ImportError:
    PYFFISH_OK = False

from .evaluation import evaluate_board
from .minimax_engine import MinimaxEngine
from .move_resolver import parse_move


VARIANT = 'xiangqi'
WIN_SCORE = 100_000
DEFAULT_ROLLOUT_DEPTH = 28
EXPLORATION = math.sqrt(2.0)


@dataclass
class _Node:
    fen: str
    move: str = ''
    parent: Optional['_Node'] = None
    untried_moves: list[str] = field(default_factory=list)
    children: list['_Node'] = field(default_factory=list)
    visits: int = 0
    value_sum: float = 0.0

    @property
    def mean_value(self) -> float:
        return self.value_sum / self.visits if self.visits else 0.0


class MCTSEngine:
    """Time-bounded Monte Carlo Tree Search engine with UCT selection."""

    def __init__(self, seed: int | None = None):
        if not PYFFISH_OK:
            raise ImportError('pyffish not installed')
        sf.set_option('VariantPath', '')
        self._rng = random.Random(seed)
        self._nodes_searched = 0
        self._start_time = 0.0
        self._time_limit = 5.0

    def get_best_move(
        self,
        fen: str,
        depth: int = 0,
        time_limit: float = 5.0,
    ) -> Tuple[str, str, int, int, float]:
        """
        Search for a move.

        ``depth`` is interpreted as the rollout depth cap for MCTS. When depth
        is zero, a sensible default rollout depth is used and the time limit is
        the main budget.
        """
        self._time_limit = max(float(time_limit or 5.0), 0.05)
        self._start_time = time.monotonic()
        self._nodes_searched = 0

        legal = sf.legal_moves(VARIANT, fen, [])
        if not legal:
            return '', '', 0, 0, 0.0

        root_red = self._side_to_move_is_red(fen)
        rollout_depth = int(depth) if depth > 0 else DEFAULT_ROLLOUT_DEPTH
        root = _Node(
            fen=fen,
            untried_moves=self._ordered_moves(legal, fen),
        )

        # Ensure every legal move is sampled at least once when the budget allows.
        while not self._time_up() and (root.untried_moves or root.visits < len(legal)):
            self._run_iteration(root, root_red, rollout_depth)

        while not self._time_up():
            self._run_iteration(root, root_red, rollout_depth)

        if root.children:
            best = max(root.children, key=lambda c: (c.visits, c.mean_value))
            best_move = best.move
            eval_root = best.mean_value
        else:
            best_move = legal[0]
            eval_root = 0.0

        elapsed = time.monotonic() - self._start_time
        # Side-to-move perspective (same as minimax); ai_engine_node applies eval_to_red_perspective.
        eval_cp = int(eval_root)
        return best_move, '', rollout_depth, eval_cp, elapsed

    def _run_iteration(self, root: _Node, root_red: bool, rollout_depth: int) -> None:
        node = self._select(root, root_red)
        if node.untried_moves:
            node = self._expand(node)
        value = self._rollout(node.fen, root_red, rollout_depth)
        self._backup(node, value)

    def _select(self, node: _Node, root_red: bool) -> _Node:
        while node.children and not node.untried_moves:
            maximizing = self._side_to_move_is_red(node.fen) == root_red
            node = max(node.children, key=lambda child: self._uct(child, maximizing))
        return node

    def _expand(self, node: _Node) -> _Node:
        move = node.untried_moves.pop(0)
        child_fen = sf.get_fen(VARIANT, node.fen, [move])
        child_moves = sf.legal_moves(VARIANT, child_fen, [])
        child = _Node(
            fen=child_fen,
            move=move,
            parent=node,
            untried_moves=self._ordered_moves(child_moves, child_fen),
        )
        node.children.append(child)
        self._nodes_searched += 1
        return child

    def _rollout(self, fen: str, root_red: bool, depth_cap: int) -> float:
        current = fen
        ply = 0
        while ply < depth_cap and not self._time_up():
            legal = sf.legal_moves(VARIANT, current, [])
            if not legal:
                return self._terminal_value(current, root_red)
            move = self._rollout_move(legal, current)
            current = sf.get_fen(VARIANT, current, [move])
            ply += 1

        red_eval = self._evaluate_fen(current)
        return float(red_eval if root_red else -red_eval)

    def _backup(self, node: _Node, value: float) -> None:
        while node is not None:
            node.visits += 1
            node.value_sum += value
            node = node.parent

    @staticmethod
    def _uct(child: _Node, maximizing: bool) -> float:
        if child.visits == 0:
            return float('inf')
        parent_visits = max(child.parent.visits if child.parent else 1, 1)
        exploit = child.mean_value if maximizing else -child.mean_value
        explore = EXPLORATION * math.sqrt(math.log(parent_visits) / child.visits)
        return exploit + explore

    def _rollout_move(self, moves: list[str], fen: str) -> str:
        captures = [m for m in moves if self._is_capture(m, fen)]
        if captures and self._rng.random() < 0.75:
            return self._rng.choice(captures)
        return self._rng.choice(moves)

    def _ordered_moves(self, moves: list[str], fen: str) -> list[str]:
        scored = []
        for move in moves:
            score = 1 if self._is_capture(move, fen) else 0
            scored.append((score, self._rng.random(), move))
        scored.sort(reverse=True)
        return [m for _, _, m in scored]

    def _terminal_value(self, fen: str, root_red: bool) -> float:
        try:
            result = sf.game_result(VARIANT, fen, [])
        except Exception:
            result = ''
        if result == '1-0':
            return WIN_SCORE if root_red else -WIN_SCORE
        if result == '0-1':
            return -WIN_SCORE if root_red else WIN_SCORE
        return 0.0

    @staticmethod
    def _evaluate_fen(fen: str) -> int:
        grid = MinimaxEngine._fen_to_grid(fen)
        legal = sf.legal_moves(VARIANT, fen, [])
        red_turn = MCTSEngine._side_to_move_is_red(fen)
        legal_red = len(legal) if red_turn else 0
        legal_black = 0 if red_turn else len(legal)
        return evaluate_board(grid, legal_red, legal_black)

    @staticmethod
    def _side_to_move_is_red(fen: str) -> bool:
        parts = fen.split()
        return len(parts) < 2 or parts[1] == 'w'

    @staticmethod
    def _is_capture(move: str, fen: str) -> bool:
        parsed = parse_move(move)
        if not parsed:
            return False
        try:
            grid = MinimaxEngine._fen_to_grid(fen)
            to_file = ord(parsed[1][0]) - ord('a')
            to_rank = parsed[1][1]
            idx = to_rank * 9 + to_file
            return 0 <= idx < len(grid) and grid[idx] != 0
        except Exception:
            return False

    def _time_up(self) -> bool:
        return (time.monotonic() - self._start_time) >= self._time_limit

    @property
    def nodes_searched(self) -> int:
        return self._nodes_searched
