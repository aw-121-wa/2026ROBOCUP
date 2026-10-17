import tempfile
import unittest
from pathlib import Path

from rdk_vision.config import load_config, save_config


VALID_YAML = """
camera:
  device: /dev/video0
  width: 640
  height: 480
  fps: 30
  stale_ms: 250
  startup_timeout_ms: 5000
serial:
  device: /dev/ttyS1
  baudrate: 115200
  reconnect_attempts: 3
  reconnect_delay_ms: 250
roi:
  x: 240
  y: 150
  width: 160
  height: 140
  edge_margin: 4
colors:
  red:
    ranges:
      - [0, 100, 60, 10, 255, 255]
      - [170, 100, 60, 179, 255, 255]
  blue:
    ranges:
      - [90, 80, 50, 135, 255, 255]
ball:
  min_mask_pixels: 700
  min_contour_area: 900
  min_width: 20
  min_height: 20
  max_width: 260
  max_height: 260
  reference_width: 50
  reference_height: 50
  aspect_ratio_min: 0.75
  aspect_ratio_max: 1.30
  size_min_scale: 0.65
  size_max_scale: 1.40
  fill_ratio_min: 0.50
  fill_ratio_max: 0.90
temporal:
  window_frames: 3
  required_hits: 2
"""


class ConfigTests(unittest.TestCase):
    def test_loads_v1_defaults(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "config.yaml"
            path.write_text(VALID_YAML, encoding="utf-8")
            cfg = load_config(path)
        self.assertEqual((cfg.camera.width, cfg.camera.height, cfg.camera.fps), (640, 480, 30))
        self.assertEqual(cfg.camera.stale_ms, 250)
        self.assertEqual((cfg.roi.x, cfg.roi.y, cfg.roi.width, cfg.roi.height), (240, 150, 160, 140))
        self.assertEqual((cfg.temporal.window_frames, cfg.temporal.required_hits), (3, 2))
        self.assertEqual(len(cfg.colors["red"]), 2)
        self.assertEqual(len(cfg.colors["blue"]), 1)

    def test_rejects_roi_outside_frame(self):
        bad = VALID_YAML.replace("x: 240", "x: 600")
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "bad.yaml"
            path.write_text(bad, encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "ROI"):
                load_config(path)

    def test_rejects_temporal_required_hits_larger_than_window(self):
        bad = VALID_YAML.replace("required_hits: 2", "required_hits: 4")
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "bad.yaml"
            path.write_text(bad, encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "temporal"):
                load_config(path)

    def test_with_calibration_updates_roi_and_reference_size(self):
        from rdk_vision.config import RoiConfig, with_calibration

        with tempfile.TemporaryDirectory() as temp_dir:
            source = Path(temp_dir) / "source.yaml"
            saved = Path(temp_dir) / "saved.yaml"
            source.write_text(VALID_YAML, encoding="utf-8")
            cfg = load_config(source)
            updated = with_calibration(
                cfg,
                roi=RoiConfig(250, 160, 150, 130, 4),
                reference_width=54,
                reference_height=52,
            )
            save_config(updated, saved)
            loaded = load_config(saved)

        self.assertEqual((loaded.roi.x, loaded.roi.y), (250, 160))
        self.assertEqual((loaded.ball.reference_width, loaded.ball.reference_height), (54, 52))

    def test_save_then_reload_preserves_calibration_values(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            source = Path(temp_dir) / "source.yaml"
            saved = Path(temp_dir) / "saved.yaml"
            source.write_text(VALID_YAML, encoding="utf-8")
            cfg = load_config(source)
            save_config(cfg, saved)
            loaded = load_config(saved)
        self.assertEqual(loaded.roi, cfg.roi)
        self.assertEqual(loaded.colors, cfg.colors)
        self.assertEqual(loaded.ball.reference_width, 50)


if __name__ == "__main__":
    unittest.main()
