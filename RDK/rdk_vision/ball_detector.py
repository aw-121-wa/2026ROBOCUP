from dataclasses import dataclass
from typing import Optional, Tuple

import cv2
import numpy as np

from rdk_vision.config import VisionConfig


@dataclass(frozen=True)
class DetectionResult:
    valid: bool
    reason: str
    bbox: Optional[Tuple[int, int, int, int]] = None
    mask_pixels: int = 0
    contour_area: float = 0.0
    aspect_ratio: float = 0.0
    fill_ratio: float = 0.0


class BallDetector:
    def __init__(self, config: VisionConfig):
        self.config = config
        self._kernel = np.ones((3, 3), dtype=np.uint8)

    def build_mask(self, frame: np.ndarray, color: str) -> np.ndarray:
        if color not in self.config.colors:
            raise ValueError(f"unsupported color: {color}")
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        combined = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for low_h, low_s, low_v, high_h, high_s, high_v in self.config.colors[color]:
            low = np.array([low_h, low_s, low_v], dtype=np.uint8)
            high = np.array([high_h, high_s, high_v], dtype=np.uint8)
            combined = cv2.bitwise_or(combined, cv2.inRange(hsv, low, high))
        combined = cv2.morphologyEx(combined, cv2.MORPH_OPEN, self._kernel, iterations=1)
        combined = cv2.morphologyEx(combined, cv2.MORPH_CLOSE, self._kernel, iterations=1)
        return combined

    def detect(self, frame: np.ndarray, color: str) -> DetectionResult:
        if frame is None or frame.ndim != 3:
            return DetectionResult(False, "invalid_frame")

        roi = self.config.roi
        mask = self.build_mask(frame, color)
        roi_mask = mask[roi.y:roi.y + roi.height, roi.x:roi.x + roi.width]
        contours, _ = cv2.findContours(roi_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return DetectionResult(False, "no_candidate")

        valid_results = []
        rejected_results = []
        for contour in contours:
            result = self._evaluate_contour(roi_mask, contour)
            (valid_results if result.valid else rejected_results).append(result)

        if valid_results:
            return max(valid_results, key=lambda item: (item.contour_area, item.mask_pixels))
        return max(rejected_results, key=lambda item: (item.contour_area, item.mask_pixels))

    def _evaluate_contour(self, roi_mask: np.ndarray, contour: np.ndarray) -> DetectionResult:
        ball = self.config.ball
        roi = self.config.roi
        x, y, width, height = cv2.boundingRect(contour)
        contour_area = float(cv2.contourArea(contour))
        mask_pixels = int(cv2.countNonZero(roi_mask[y:y + height, x:x + width]))
        aspect_ratio = width / float(height)
        fill_ratio = mask_pixels / float(width * height)
        global_bbox = (roi.x + x, roi.y + y, width, height)

        def result(valid: bool, reason: str) -> DetectionResult:
            return DetectionResult(
                valid=valid,
                reason=reason,
                bbox=global_bbox,
                mask_pixels=mask_pixels,
                contour_area=contour_area,
                aspect_ratio=aspect_ratio,
                fill_ratio=fill_ratio,
            )

        if contour_area < ball.min_contour_area:
            return result(False, "contour_area")

        if (
            width < ball.min_width
            or height < ball.min_height
            or width > ball.max_width
            or height > ball.max_height
        ):
            return result(False, "size")

        margin = roi.edge_margin
        inside = (
            x >= margin
            and y >= margin
            and x + width <= roi.width - margin
            and y + height <= roi.height - margin
        )
        if not inside:
            return result(False, "edge_margin")

        if not (ball.aspect_ratio_min <= aspect_ratio <= ball.aspect_ratio_max):
            return result(False, "aspect_ratio")

        min_w = ball.reference_width * ball.size_min_scale
        max_w = ball.reference_width * ball.size_max_scale
        min_h = ball.reference_height * ball.size_min_scale
        max_h = ball.reference_height * ball.size_max_scale
        if not (min_w <= width <= max_w and min_h <= height <= max_h):
            return result(False, "size")

        if mask_pixels < ball.min_mask_pixels:
            return result(False, "mask_pixels")

        if not (ball.fill_ratio_min <= fill_ratio <= ball.fill_ratio_max):
            return result(False, "fill_ratio")

        return result(True, "ok")
