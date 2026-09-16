from pathlib import Path
import unittest
import yaml


class LatestCalibrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        root = Path(__file__).resolve().parents[1]
        cls.cfg = yaml.safe_load((root / 'rdk_vision' / 'config.yaml').read_text(encoding='utf-8'))

    def test_latest_roi_is_packaged(self):
        self.assertEqual(
            self.cfg['roi'],
            {'x': 92, 'y': 29, 'width': 352, 'height': 244, 'edge_margin': 4},
        )

    def test_latest_ball_reference_is_packaged(self):
        ball = self.cfg['ball']
        self.assertEqual((ball['reference_width'], ball['reference_height']), (148, 150))
        self.assertEqual((ball['aspect_ratio_min'], ball['aspect_ratio_max']), (0.75, 1.3))
        self.assertEqual((ball['fill_ratio_min'], ball['fill_ratio_max']), (0.5, 0.9))

    def test_latest_red_hsv_is_packaged(self):
        self.assertEqual(
            self.cfg['colors']['red']['ranges'],
            [[0, 100, 60, 10, 255, 255], [170, 100, 60, 179, 255, 255]],
        )


if __name__ == '__main__':
    unittest.main()
