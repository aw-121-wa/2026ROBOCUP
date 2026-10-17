"""Task-specific ball sizes with shared camera, ROI and HSV calibration."""
from dataclasses import replace
import math
from pathlib import Path

import yaml

from .config import load_config


SIZE_FIELDS = {
    'reference_width', 'reference_height', 'size_min_scale', 'size_max_scale',
    'min_width', 'min_height', 'max_width', 'max_height',
}


def load_task_config(config_path, task):
    if task not in ('disc', 'pillar', 'stair'):
        raise ValueError(f'Unknown vision task: {task}')
    config_path = Path(config_path)
    base = load_config(config_path)
    profiles = yaml.safe_load(
        config_path.with_name('task_sizes.yaml').read_text(encoding='utf-8')
    )
    if not isinstance(profiles, dict):
        raise ValueError('task_sizes.yaml must contain task sections')
    overrides = profiles.get(task, {})
    if not isinstance(overrides, dict):
        raise ValueError(f'{task} size settings must be a mapping')
    unknown = set(overrides) - SIZE_FIELDS
    if unknown:
        raise ValueError(f'Unknown size settings: {unknown}')
    ball = replace(base.ball, **overrides)
    for field in SIZE_FIELDS:
        value = getattr(ball, field)
        if (isinstance(value, bool) or not isinstance(value, (int, float))
                or not math.isfinite(value) or value <= 0):
            raise ValueError(f'{field} must be a finite positive number')
    if ball.size_min_scale > ball.size_max_scale:
        raise ValueError('Invalid size scale range')
    if ball.min_width > ball.max_width or ball.min_height > ball.max_height:
        raise ValueError('Invalid size limits')
    return replace(base, ball=ball)
