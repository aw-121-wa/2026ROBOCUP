from dataclasses import asdict, dataclass, replace
from pathlib import Path
from typing import Dict, List, Optional, Tuple, Union

import yaml

HsvRange = Tuple[int, int, int, int, int, int]


@dataclass(frozen=True)
class CameraConfig:
    device: str
    width: int
    height: int
    fps: int
    stale_ms: int
    startup_timeout_ms: int


@dataclass(frozen=True)
class SerialConfig:
    device: str
    baudrate: int
    reconnect_attempts: int
    reconnect_delay_ms: int


@dataclass(frozen=True)
class RoiConfig:
    x: int
    y: int
    width: int
    height: int
    edge_margin: int


@dataclass(frozen=True)
class BallConfig:
    min_mask_pixels: int
    min_contour_area: float
    min_width: int
    min_height: int
    max_width: int
    max_height: int
    reference_width: int
    reference_height: int
    aspect_ratio_min: float
    aspect_ratio_max: float
    size_min_scale: float
    size_max_scale: float
    fill_ratio_min: float
    fill_ratio_max: float


@dataclass(frozen=True)
class TemporalConfig:
    window_frames: int
    required_hits: int


@dataclass(frozen=True)
class VisionConfig:
    camera: CameraConfig
    serial: SerialConfig
    roi: RoiConfig
    colors: Dict[str, List[HsvRange]]
    ball: BallConfig
    temporal: TemporalConfig


def load_config(path: Union[str, Path]) -> VisionConfig:
    payload = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    camera = CameraConfig(**payload["camera"])
    serial = SerialConfig(**payload["serial"])
    roi = RoiConfig(**payload["roi"])
    colors = {
        name: [tuple(int(value) for value in item) for item in section["ranges"]]
        for name, section in payload["colors"].items()
    }
    ball = BallConfig(**payload["ball"])
    temporal = TemporalConfig(**payload["temporal"])
    config = VisionConfig(camera, serial, roi, colors, ball, temporal)
    _validate(config)
    return config


def save_config(config: VisionConfig, path: Union[str, Path]) -> None:
    _validate(config)
    payload = {
        "camera": asdict(config.camera),
        "serial": asdict(config.serial),
        "roi": asdict(config.roi),
        "colors": {
            name: {"ranges": [list(item) for item in ranges]}
            for name, ranges in config.colors.items()
        },
        "ball": asdict(config.ball),
        "temporal": asdict(config.temporal),
    }
    Path(path).write_text(
        yaml.safe_dump(payload, sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )


def _validate(config: VisionConfig) -> None:
    if config.camera.width <= 0 or config.camera.height <= 0 or config.camera.fps <= 0:
        raise ValueError("camera dimensions/fps must be positive")
    if config.camera.stale_ms <= 0 or config.camera.startup_timeout_ms <= 0:
        raise ValueError("camera timing values must be positive")

    roi = config.roi
    if roi.x < 0 or roi.y < 0 or roi.width <= 0 or roi.height <= 0:
        raise ValueError("ROI must be positive and inside frame")
    if roi.x + roi.width > config.camera.width or roi.y + roi.height > config.camera.height:
        raise ValueError("ROI exceeds camera frame")
    if roi.edge_margin < 0 or roi.edge_margin * 2 >= min(roi.width, roi.height):
        raise ValueError("ROI edge margin is invalid")

    if not (1 <= config.temporal.required_hits <= config.temporal.window_frames):
        raise ValueError("temporal required_hits must be within window_frames")

    if set(config.colors) != {"red", "blue"}:
        raise ValueError("colors must define exactly red and blue")
    for name, ranges in config.colors.items():
        if not ranges:
            raise ValueError(f"color {name} must have at least one HSV range")
        for low_h, low_s, low_v, high_h, high_s, high_v in ranges:
            if not (0 <= low_h <= high_h <= 179):
                raise ValueError(f"invalid Hue range for {name}")
            if not (0 <= low_s <= high_s <= 255 and 0 <= low_v <= high_v <= 255):
                raise ValueError(f"invalid S/V range for {name}")


def with_calibration(
    config: VisionConfig,
    *,
    roi: Optional[RoiConfig] = None,
    colors: Optional[Dict[str, List[HsvRange]]] = None,
    reference_width: Optional[int] = None,
    reference_height: Optional[int] = None,
) -> VisionConfig:
    ball = config.ball
    if reference_width is not None or reference_height is not None:
        ball = replace(
            ball,
            reference_width=(
                ball.reference_width if reference_width is None else reference_width
            ),
            reference_height=(
                ball.reference_height if reference_height is None else reference_height
            ),
        )
    updated = replace(
        config,
        roi=roi or config.roi,
        colors=colors or config.colors,
        ball=ball,
    )
    _validate(updated)
    return updated
