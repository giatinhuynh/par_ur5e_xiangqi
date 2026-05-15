"""Resolve board_calibration.yaml path with packaged sim fallback."""

from __future__ import annotations

import os

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:  # pragma: no cover
    get_package_share_directory = None  # type: ignore


def resolve_manipulation_calibration_path(primary: str, logger) -> str:
    primary = primary or ''
    if primary and os.path.isfile(primary):
        return primary
    if get_package_share_directory is None:
        return primary
    try:
        fb = os.path.join(
            get_package_share_directory('xiangqi_manipulation'),
            'config',
            'board_calibration_sim.yaml',
        )
        if os.path.isfile(fb):
            logger.warning(
                f'Calibration file not found at {primary!r} — using packaged sim defaults: {fb}'
            )
            return fb
    except Exception:
        pass
    return primary
