"""
fairy_stockfish_engine.py: Wrapper around the Fairy-Stockfish binary via UCI protocol.

Communicates with the engine via subprocess stdin/stdout.
Supports xiangqi variant with optional NNUE weights.
"""

from __future__ import annotations
import subprocess
import threading
import time
import re
import os
from typing import Optional, Tuple

FAIRY_STOCKFISH_BIN = os.environ.get('FAIRY_STOCKFISH_BIN', 'fairy-stockfish')
NNUE_PATH = os.environ.get('XIANGQI_NNUE_PATH', '')

STARTING_FEN = 'rnbakabnr/9/1c5c1/p1p1p1p1p/9/9/P1P1P1P1P/1C5C1/9/RNBAKABNR w - - 0 1'


class FairyStockfishEngine:
    """UCI protocol wrapper for Fairy-Stockfish targeting the Xiangqi variant."""

    def __init__(self, skill_level: int = 20):
        self._proc: Optional[subprocess.Popen] = None
        self._lock = threading.Lock()
        self._skill_level = min(max(skill_level, 1), 20)
        self._ready = False
        self._start_engine()

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def _start_engine(self) -> None:
        try:
            self._proc = subprocess.Popen(
                [FAIRY_STOCKFISH_BIN],
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                text=True,
                bufsize=1,
            )
            self._send('uci')
            self._wait_for('uciok', timeout=10)
            self._send('setoption name UCI_Variant value xiangqi')
            self._send(f'setoption name Skill Level value {self._skill_level}')
            if NNUE_PATH and os.path.exists(NNUE_PATH):
                self._send(f'setoption name EvalFile value {NNUE_PATH}')
            self._send('isready')
            self._wait_for('readyok', timeout=15)
            self._ready = True
        except Exception as e:
            raise RuntimeError(f'Failed to start Fairy-Stockfish: {e}')

    def close(self) -> None:
        if self._proc and self._proc.poll() is None:
            try:
                self._send('quit')
                self._proc.wait(timeout=3)
            except Exception:
                self._proc.kill()

    def set_skill_level(self, level: int) -> None:
        self._skill_level = min(max(level, 1), 20)
        self._send(f'setoption name Skill Level value {self._skill_level}')
        self._send('isready')
        self._wait_for('readyok', timeout=5)

    # ------------------------------------------------------------------
    # Main interface
    # ------------------------------------------------------------------

    def get_best_move(
        self,
        fen: str,
        depth: int = 0,
        time_limit: float = 5.0,
    ) -> Tuple[str, str, int, int, float]:
        """
        Get best move for a position.

        Returns:
            (best_move, ponder_move, depth_reached, evaluation_cp, thinking_time)
        """
        if not self._ready:
            raise RuntimeError('Engine not ready')

        with self._lock:
            self._send('ucinewgame')
            self._send(f'position fen {fen}')

            if depth > 0:
                go_cmd = f'go depth {depth}'
            else:
                movetime_ms = int(time_limit * 1000)
                go_cmd = f'go movetime {movetime_ms}'

            self._send(go_cmd)
            t_start = time.monotonic()

            best_move = ''
            ponder_move = ''
            depth_reached = 0
            eval_cp = 0
            lines = []

            timeout = time_limit + 10
            deadline = t_start + timeout

            while time.monotonic() < deadline:
                line = self._readline(timeout=1.0)
                if line is None:
                    continue
                lines.append(line)

                # Parse info lines for depth and score
                if line.startswith('info'):
                    m = re.search(r'depth (\d+)', line)
                    if m:
                        depth_reached = int(m.group(1))
                    m = re.search(r'score cp (-?\d+)', line)
                    if m:
                        eval_cp = int(m.group(1))

                if line.startswith('bestmove'):
                    parts = line.split()
                    if len(parts) >= 2:
                        best_move = parts[1] if parts[1] != '(none)' else ''
                    if len(parts) >= 4 and parts[2] == 'ponder':
                        ponder_move = parts[3]
                    break

            elapsed = time.monotonic() - t_start
            return best_move, ponder_move, depth_reached, eval_cp, elapsed

    # ------------------------------------------------------------------
    # Low-level UCI communication
    # ------------------------------------------------------------------

    def _send(self, cmd: str) -> None:
        if self._proc and self._proc.poll() is None:
            self._proc.stdin.write(cmd + '\n')
            self._proc.stdin.flush()

    def _readline(self, timeout: float = 1.0) -> Optional[str]:
        """Read one line from the engine stdout with timeout."""
        import select
        if self._proc is None:
            return None
        ready, _, _ = select.select([self._proc.stdout], [], [], timeout)
        if ready:
            line = self._proc.stdout.readline()
            return line.rstrip('\n') if line else None
        return None

    def _wait_for(self, token: str, timeout: float = 10.0) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            line = self._readline(timeout=0.5)
            if line and token in line:
                return True
        return False

    def __del__(self):
        self.close()
