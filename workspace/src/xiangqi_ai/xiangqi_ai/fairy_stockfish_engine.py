"""
fairy_stockfish_engine.py: Wrapper around the Fairy-Stockfish binary via UCI protocol.

Communicates with the engine via subprocess stdin/stdout.
Supports xiangqi variant with optional NNUE weights.
"""

from __future__ import annotations
import os
import queue
import re
import subprocess
import threading
import time
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
        self._line_queue: queue.Queue[str | None] = queue.Queue()
        self._reader_thread: Optional[threading.Thread] = None
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
            self._reader_thread = threading.Thread(
                target=self._stdout_reader_loop, daemon=True
            )
            self._reader_thread.start()
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
        self._line_queue.put(None)

    def set_skill_level(self, level: int) -> None:
        self._skill_level = min(max(level, 1), 20)
        self._send(f'setoption name Skill Level value {self._skill_level}')
        self._send('isready')
        self._wait_for('readyok', timeout=5)

    # ------------------------------------------------------------------
    # Main interface
    # ------------------------------------------------------------------

    def _parse_info_score(self, line: str, eval_cp: int) -> int:
        m = re.search(r'score cp (-?\d+)', line)
        if m:
            return int(m.group(1))
        m = re.search(r'score mate (-?\d+)', line)
        if m:
            mate_in = int(m.group(1))
            return 30000 if mate_in > 0 else -30000
        return eval_cp

    def _run_search(
        self,
        fen: str,
        depth: int,
        time_limit: float,
    ) -> Tuple[str, str, int, int, float]:
        """Run UCI search; returns (best_move, ponder, depth, eval_cp, elapsed)."""
        if depth > 0:
            go_cmd = f'go depth {depth}'
            search_sec = max(time_limit, 5.0)
        else:
            movetime_ms = max(int(time_limit * 1000), 1)
            go_cmd = f'go movetime {movetime_ms}'
            search_sec = time_limit

        # Wait long enough for movetime/depth plus UCI info lines.
        timeout = search_sec + 15.0

        self._drain_queue()
        self._send(f'position fen {fen}')
        self._send(go_cmd)
        t_start = time.monotonic()

        best_move = ''
        ponder_move = ''
        depth_reached = 0
        eval_cp = 0
        deadline = t_start + timeout

        while time.monotonic() < deadline:
            try:
                line = self._line_queue.get(timeout=0.25)
            except queue.Empty:
                continue
            if line is None:
                break

            if line.startswith('info'):
                # Ignore "depth" on mate lines (e.g. depth 245 = mate distance, not search depth)
                if ' score mate ' not in line:
                    m = re.search(r'\bdepth (\d+)', line)
                    if m:
                        d = int(m.group(1))
                        if d <= 64:
                            depth_reached = max(depth_reached, d)
                eval_cp = self._parse_info_score(line, eval_cp)

            if line.startswith('bestmove'):
                parts = line.split()
                if len(parts) >= 2:
                    best_move = parts[1] if parts[1] != '(none)' else ''
                if len(parts) >= 4 and parts[2] == 'ponder':
                    ponder_move = parts[3]
                break

        elapsed = time.monotonic() - t_start
        return best_move, ponder_move, depth_reached, eval_cp, elapsed

    def evaluate_position(
        self,
        fen: str,
        movetime_ms: int = 120,
    ) -> Tuple[int, int]:
        """
        NNUE/static eval via shallow Fairy-Stockfish search (for display / comparison).
        Score is from side-to-move perspective (same as UCI).
        """
        if not self._ready:
            raise RuntimeError('Engine not ready')
        with self._lock:
            _, _, depth_reached, eval_cp, _ = self._run_search(
                fen, depth=0, time_limit=movetime_ms / 1000.0
            )
        return eval_cp, depth_reached

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
            return self._run_search(fen, depth, time_limit)

    # ------------------------------------------------------------------
    # Low-level UCI communication
    # ------------------------------------------------------------------

    def _stdout_reader_loop(self) -> None:
        proc = self._proc
        if proc is None or proc.stdout is None:
            self._line_queue.put(None)
            return
        try:
            for line in proc.stdout:
                self._line_queue.put(line.rstrip('\n'))
        except Exception:
            pass
        finally:
            self._line_queue.put(None)

    def _drain_queue(self) -> None:
        while True:
            try:
                line = self._line_queue.get_nowait()
            except queue.Empty:
                break
            if line is None:
                self._line_queue.put(None)
                break

    def _send(self, cmd: str) -> None:
        if self._proc and self._proc.poll() is None:
            self._proc.stdin.write(cmd + '\n')
            self._proc.stdin.flush()

    def _wait_for(self, token: str, timeout: float = 10.0) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                line = self._line_queue.get(timeout=0.25)
            except queue.Empty:
                continue
            if line is None:
                return False
            if token in line:
                return True
        return False

    def __del__(self):
        self.close()
