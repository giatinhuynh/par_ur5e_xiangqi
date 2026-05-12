"""
Turn detector: vision-based stability detection to determine when the human
has finished making their move.

Logic:
  - Continuously compare current board state against the reference state.
  - When a difference is found, start counting stable frames.
  - Once the board state has been stable (unchanged) for `stability_frames`
    consecutive frames, the human move is confirmed.
  - A keyboard fallback allows forcing a move detection event via ROS topic.
"""

from __future__ import annotations
import numpy as np
from enum import Enum, auto
from typing import Optional, Tuple


class TurnDetectorState(Enum):
    IDLE = auto()           # Not watching (robot's turn or game over)
    WATCHING = auto()       # Waiting for the human to move
    CHANGE_DETECTED = auto()  # Detected a board change, collecting stability evidence
    CONFIRMED = auto()      # Stable new state confirmed as valid human move


class TurnDetector:
    """
    Finite-state detector that watches the board for a stable human move.

    Designed to run inside the vision_node at ~3 Hz.
    """

    def __init__(self, stability_frames: int = 8, change_threshold: int = 1):
        self._stability_frames = stability_frames
        self._change_threshold = change_threshold   # Min occupied cells that differ to count as a change
        self._state = TurnDetectorState.IDLE
        self._reference_grid: Optional[np.ndarray] = None
        self._candidate_grid: Optional[np.ndarray] = None
        self._stability_count = 0
        self._keyboard_trigger = False

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    @property
    def state(self) -> TurnDetectorState:
        return self._state

    def start_watching(self, reference_grid: np.ndarray) -> None:
        """Begin watching for a human move from the given reference board state."""
        self._reference_grid = reference_grid.copy()
        self._candidate_grid = None
        self._stability_count = 0
        self._state = TurnDetectorState.WATCHING

    def stop_watching(self) -> None:
        self._state = TurnDetectorState.IDLE
        self._candidate_grid = None
        self._stability_count = 0

    def trigger_keyboard_fallback(self) -> None:
        """Force a move detection event (keyboard fallback)."""
        self._keyboard_trigger = True

    def update(self, current_grid: np.ndarray) -> Tuple[bool, Optional[np.ndarray]]:
        """
        Feed a new board observation into the detector.

        Returns:
            (move_confirmed, confirmed_grid)
            move_confirmed is True exactly once per move cycle.
        """
        if self._state == TurnDetectorState.IDLE:
            return False, None

        # Keyboard override
        if self._keyboard_trigger:
            self._keyboard_trigger = False
            self._state = TurnDetectorState.CONFIRMED
            result_grid = current_grid.copy()
            self.stop_watching()
            return True, result_grid

        if self._state == TurnDetectorState.CONFIRMED:
            return False, None

        changed = self._grids_differ(self._reference_grid, current_grid)

        if self._state == TurnDetectorState.WATCHING:
            if changed:
                self._candidate_grid = current_grid.copy()
                self._stability_count = 1
                self._state = TurnDetectorState.CHANGE_DETECTED
            return False, None

        if self._state == TurnDetectorState.CHANGE_DETECTED:
            if self._grids_differ(self._candidate_grid, current_grid):
                # Board still changing -- hand may still be on it
                self._candidate_grid = current_grid.copy()
                self._stability_count = 1
            else:
                self._stability_count += 1

            if self._stability_count >= self._stability_frames:
                if changed:  # Still different from reference (not undone)
                    confirmed = self._candidate_grid.copy()
                    self._state = TurnDetectorState.CONFIRMED
                    self.stop_watching()
                    return True, confirmed
                else:
                    # Board returned to reference state (move undone)
                    self._state = TurnDetectorState.WATCHING
                    self._stability_count = 0

        return False, None

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _grids_differ(self, a: np.ndarray, b: np.ndarray) -> bool:
        """Return True if grids differ in at least `change_threshold` occupied cells."""
        if a is None or b is None:
            return False
        diff = np.sum((a != 0) & (a != b))
        new_occupied = np.sum((b != 0) & (a == 0))
        return int(diff + new_occupied) >= self._change_threshold
