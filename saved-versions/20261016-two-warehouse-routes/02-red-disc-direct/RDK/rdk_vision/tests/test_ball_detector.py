import unittest

import cv2
import numpy as np

from rdk_vision.ball_detector import BallDetector
from rdk_vision.config import load_config


class BallDetectorTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cfg = load_config("rdk_vision/config.yaml")
        cls.detector = BallDetector(cls.cfg)
        cls.cx = cls.cfg.roi.x + cls.cfg.roi.width // 2
        cls.cy = cls.cfg.roi.y + cls.cfg.roi.height // 2
        cls.radius = max(12, min(cls.cfg.ball.reference_width, cls.cfg.ball.reference_height) // 2)

    def blank(self):
        return np.zeros((480, 640, 3), dtype=np.uint8)

    def test_red_circle_inside_roi_passes_red(self):
        frame = self.blank()
        cv2.circle(frame, (self.cx, self.cy), self.radius, (0, 0, 255), -1)
        result = self.detector.detect(frame, "red")
        self.assertTrue(result.valid, result.reason)
        self.assertIsNotNone(result.bbox)
        self.assertGreater(result.fill_ratio, 0.50)
        self.assertLess(result.fill_ratio, 0.90)

    def test_red_circle_does_not_pass_blue(self):
        frame = self.blank()
        cv2.circle(frame, (self.cx, self.cy), self.radius, (0, 0, 255), -1)
        self.assertFalse(self.detector.detect(frame, "blue").valid)

    def test_blue_circle_inside_roi_passes_blue(self):
        frame = self.blank()
        cv2.circle(frame, (self.cx, self.cy), self.radius, (255, 0, 0), -1)
        self.assertTrue(self.detector.detect(frame, "blue").valid)

    def test_partial_ball_touching_roi_edge_is_rejected(self):
        frame = self.blank()
        x = self.cfg.roi.x + self.radius - 2
        cv2.circle(frame, (x, self.cy), self.radius, (0, 0, 255), -1)
        result = self.detector.detect(frame, "red")
        self.assertFalse(result.valid)
        self.assertIn(result.reason, {"edge_margin", "size", "no_candidate"})

    def test_solid_square_is_rejected_by_fill_ratio(self):
        frame = self.blank()
        half_w = self.cfg.ball.reference_width // 2
        half_h = self.cfg.ball.reference_height // 2
        cv2.rectangle(
            frame,
            (self.cx - half_w, self.cy - half_h),
            (self.cx + half_w, self.cy + half_h),
            (0, 0, 255),
            -1,
        )
        result = self.detector.detect(frame, "red")
        self.assertFalse(result.valid)
        self.assertEqual(result.reason, "fill_ratio")

    def test_too_small_circle_is_rejected(self):
        frame = self.blank()
        small_radius = max(4, int(self.radius * 0.3))
        cv2.circle(frame, (self.cx, self.cy), small_radius, (0, 0, 255), -1)
        self.assertFalse(self.detector.detect(frame, "red").valid)


if __name__ == "__main__":
    unittest.main()
