"""Pillar-only geometry over the shared, on-device vision calibration."""
from dataclasses import dataclass
import math
from pathlib import Path

import yaml

from rdk_vision.config import RoiConfig, with_calibration


@dataclass
class PillarPositionTrigger:
    center_x: float
    center_y: float
    tolerance_x: float
    tolerance_y: float
    state: str = 'WAIT_POSITION'

    def update(self, result):
        self.state = 'WAIT_POSITION'
        if not result.valid or result.bbox is None:
            return None
        x, y, width, height = result.bbox
        if (abs(x + width / 2 - self.center_x) <= self.tolerance_x and
                abs(y + height / 2 - self.center_y) <= self.tolerance_y):
            self.state = 'AT_GRAB_POSITION'
            return 'TRIGGER_VALID'
        return None


def load_pillar_profile(base, path):
    profile = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    if (profile['frame_width'], profile['frame_height']) != (base.camera.width, base.camera.height):
        raise ValueError('pillar profile resolution does not match shared camera config')
    roi = RoiConfig(**profile['roi'])
    if any(type(value) is not int for value in vars(roi).values()):
        raise ValueError('pillar ROI must contain integer pixel coordinates')
    config = with_calibration(base, roi=roi)
    grab = profile['grab']
    values = [float(grab[key]) for key in ('center_x', 'center_y', 'tolerance_x', 'tolerance_y')]
    if not all(math.isfinite(value) for value in values):
        raise ValueError('pillar grab coordinates must be finite')
    cx, cy, tx, ty = values
    if (tx <= 0 or ty <= 0 or cx - tx < roi.x + roi.edge_margin or
            cx + tx > roi.x + roi.width - roi.edge_margin or
            cy - ty < roi.y + roi.edge_margin or
            cy + ty > roi.y + roi.height - roi.edge_margin):
        raise ValueError('pillar grab window must be positive and inside ROI')
    return config, PillarPositionTrigger(cx, cy, tx, ty)
