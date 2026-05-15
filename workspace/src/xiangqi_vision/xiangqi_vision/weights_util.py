"""Resolve YOLO weight path (workspace, package share, optional HTTP download)."""

from __future__ import annotations

import os
import urllib.request
from typing import Optional

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:  # pragma: no cover
    get_package_share_directory = None  # type: ignore


def _share_models_dir() -> Optional[str]:
    if get_package_share_directory is None:
        return None
    try:
        return os.path.join(get_package_share_directory('xiangqi_vision'), 'models')
    except Exception:
        return None


def resolve_calibration_path(primary: str, logger) -> str:
    """If primary file is missing, use packaged sim calibration (vision share)."""
    primary = primary or ''
    if primary and os.path.isfile(primary):
        return primary
    if get_package_share_directory is None:
        return primary
    try:
        fb = os.path.join(
            get_package_share_directory('xiangqi_vision'),
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


def _candidate_model_paths(primary: str) -> list[str]:
    out: list[str] = []
    p0 = (primary or '').strip()
    if p0:
        out.append(os.path.expanduser(p0))
    share_models = _share_models_dir()
    if not share_models:
        return out
    base_names = ['xiangqi_kaggle_v1_best.pt', 'xiangqi_default.pt']
    if p0:
        bn = os.path.basename(p0)
        ordered = [bn] + [n for n in base_names if n != bn]
    else:
        ordered = list(base_names)
    seen: set[str] = set()
    for n in ordered:
        path = os.path.join(share_models, n)
        if path not in seen:
            seen.add(path)
            out.append(path)
    return out


def _download(url: str, dest_path: str, logger) -> bool:
    parent = os.path.dirname(dest_path)
    if parent:
        os.makedirs(parent, exist_ok=True)
    logger.warning(f'Downloading YOLO weights from {url} -> {dest_path}')
    try:
        urllib.request.urlretrieve(url, dest_path)
        return os.path.isfile(dest_path) and os.path.getsize(dest_path) > 0
    except Exception as e:
        logger.error(f'YOLO weight download failed: {e}')
        return False


def resolve_yolo_model_path(
    primary: str,
    *,
    param_download_url: str,
    logger,
) -> Optional[str]:
    """
    Return an existing .pt path, or None.

    Order: primary path; package share models/ (canonical names); env XIANGQI_YOLO_DOWNLOAD_URL;
    param yolo_download_url (non-empty).
    """
    for p in _candidate_model_paths(primary):
        if os.path.isfile(p):
            return p

    share_models = _share_models_dir()
    env_url = (os.environ.get('XIANGQI_YOLO_DOWNLOAD_URL') or '').strip()
    url = (param_download_url or '').strip() or env_url
    if url and share_models:
        dest = os.path.join(share_models, 'xiangqi_kaggle_v1_best.pt')
        if _download(url, dest, logger):
            return dest
    return None
