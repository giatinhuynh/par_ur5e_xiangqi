"""
Configurable image preprocessing for vision experiments.

Applied to the warped board image before YOLO inference. ArUco detection uses its
own CLAHE pass in board_detector.py; tune piece_* parameters here for piece ID.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

import cv2
import numpy as np

# Named bundles for quick A/B tests (ros2 param piece_preprocess_preset).
PRESETS: dict[str, dict[str, float | bool | str]] = {
    'none': {},
    'clahe': {'use_clahe': True, 'clahe_clip_limit': 2.0},
    'clahe_strong': {'use_clahe': True, 'clahe_clip_limit': 4.0},
    'gamma_bright': {'gamma': 0.75},
    'gamma_dark': {'gamma': 1.35},
    'high_contrast': {'contrast': 1.4, 'brightness': 10},
    'denoise': {'use_denoise': True},
    'sharpen': {'use_sharpen': True},
    'saturation': {'saturation_scale': 1.35},
    'lab_default': {
        'use_clahe': True,
        'clahe_clip_limit': 2.5,
        'gamma': 0.9,
        'contrast': 1.15,
        'saturation_scale': 1.2,
    },
    # Dim wrist-cam scans: lift exposure, local contrast, then sharpen edges.
    'bright': {
        'gamma': 0.72,
        'brightness': 25,
        'contrast': 1.12,
        'saturation_scale': 1.15,
    },
    'bright_sharp': {
        'gamma': 0.72,
        'brightness': 25,
        'contrast': 1.12,
        'use_clahe': True,
        'clahe_clip_limit': 2.5,
        'saturation_scale': 1.15,
        'use_sharpen': True,
    },
}


@dataclass
class PreprocessConfig:
    enabled: bool = False
    preset: str = 'none'
    gamma: float = 1.0
    brightness: int = 0
    contrast: float = 1.0
    use_clahe: bool = False
    clahe_clip_limit: float = 2.0
    clahe_tile_size: int = 8
    use_denoise: bool = False
    denoise_strength: int = 5
    use_sharpen: bool = False
    saturation_scale: float = 1.0
    use_white_balance: bool = False

    @classmethod
    def from_preset(cls, preset: str, enabled: bool = True, **overrides) -> 'PreprocessConfig':
        base = dict(PRESETS.get(preset, {}))
        base.update(overrides)
        return cls(enabled=enabled and preset != 'none', preset=preset, **base)

    def effective(self) -> 'PreprocessConfig':
        """Merge preset defaults with explicit field overrides."""
        if not self.enabled or self.preset == 'none':
            return PreprocessConfig(enabled=False, preset='none')
        merged = PreprocessConfig.from_preset(self.preset, enabled=True)
        for field_name in (
            'gamma', 'brightness', 'contrast', 'use_clahe', 'clahe_clip_limit',
            'clahe_tile_size', 'use_denoise', 'denoise_strength', 'use_sharpen',
            'saturation_scale', 'use_white_balance',
        ):
            val = getattr(self, field_name)
            default = getattr(merged, field_name)
            if val != default:
                setattr(merged, field_name, val)
        merged.preset = self.preset
        return merged


def _apply_gamma(image: np.ndarray, gamma: float) -> np.ndarray:
    if abs(gamma - 1.0) < 1e-3:
        return image
    inv = 1.0 / max(gamma, 1e-3)
    table = np.array([((i / 255.0) ** inv) * 255 for i in range(256)], dtype=np.uint8)
    return cv2.LUT(image, table)


def _apply_brightness_contrast(image: np.ndarray, alpha: float, beta: int) -> np.ndarray:
    if abs(alpha - 1.0) < 1e-3 and beta == 0:
        return image
    return cv2.convertScaleAbs(image, alpha=alpha, beta=beta)


def _apply_clahe_bgr(image: np.ndarray, clip_limit: float, tile_size: int) -> np.ndarray:
    lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)
    clahe = cv2.createCLAHE(clipLimit=clip_limit, tileGridSize=(tile_size, tile_size))
    l = clahe.apply(l)
    return cv2.cvtColor(cv2.merge([l, a, b]), cv2.COLOR_LAB2BGR)


def _apply_saturation(image: np.ndarray, scale: float) -> np.ndarray:
    if abs(scale - 1.0) < 1e-3:
        return image
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV).astype(np.float32)
    hsv[:, :, 1] = np.clip(hsv[:, :, 1] * scale, 0, 255)
    return cv2.cvtColor(hsv.astype(np.uint8), cv2.COLOR_HSV2BGR)


def _apply_gray_world_wb(image: np.ndarray) -> np.ndarray:
    out = image.astype(np.float32)
    for c in range(3):
        mean_c = out[:, :, c].mean()
        if mean_c > 1e-3:
            out[:, :, c] *= 128.0 / mean_c
    return np.clip(out, 0, 255).astype(np.uint8)


def _apply_denoise(image: np.ndarray, strength: int) -> np.ndarray:
    h = max(3, strength | 1)
    return cv2.fastNlMeansDenoisingColored(image, None, h, h, 7, 21)


def _apply_sharpen(image: np.ndarray) -> np.ndarray:
    blurred = cv2.GaussianBlur(image, (0, 0), sigmaX=1.0)
    return cv2.addWeighted(image, 1.5, blurred, -0.5, 0)


def apply_piece_preprocess(image: np.ndarray, cfg: PreprocessConfig) -> np.ndarray:
    """Return a BGR uint8 image ready for YOLO."""
    eff = cfg.effective()
    if not eff.enabled:
        return image

    out = image.copy()
    steps: list[Callable[[np.ndarray], np.ndarray]] = []

    if eff.use_white_balance:
        steps.append(_apply_gray_world_wb)
    if eff.use_denoise:
        steps.append(lambda img: _apply_denoise(img, eff.denoise_strength))
    if eff.use_clahe:
        steps.append(
            lambda img: _apply_clahe_bgr(img, eff.clahe_clip_limit, eff.clahe_tile_size)
        )
    if abs(eff.saturation_scale - 1.0) >= 1e-3:
        scale = eff.saturation_scale
        steps.append(lambda img, s=scale: _apply_saturation(img, s))
    if abs(eff.gamma - 1.0) >= 1e-3:
        g = eff.gamma
        steps.append(lambda img, gv=g: _apply_gamma(img, gv))
    if abs(eff.contrast - 1.0) >= 1e-3 or eff.brightness != 0:
        a, b = eff.contrast, eff.brightness
        steps.append(lambda img, alpha=a, beta=b: _apply_brightness_contrast(img, alpha, beta))
    if eff.use_sharpen:
        steps.append(_apply_sharpen)

    for step in steps:
        out = step(out)
    return out


def stack_comparison(
    raw_warped: np.ndarray,
    processed: np.ndarray,
    annotated: np.ndarray,
    labels: tuple[str, str, str] = ('warped', 'preprocessed', 'YOLO'),
) -> np.ndarray:
    """Build a horizontal strip: raw | preprocessed | detections for debug publish."""
    panels = []
    for img, label in zip((raw_warped, processed, annotated), labels):
        h, w = annotated.shape[:2]
        panel = cv2.resize(img, (w, h), interpolation=cv2.INTER_AREA)
        cv2.putText(
            panel, label, (8, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 3, cv2.LINE_AA,
        )
        cv2.putText(
            panel, label, (8, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2, cv2.LINE_AA,
        )
        panels.append(panel)
    return np.hstack(panels)
