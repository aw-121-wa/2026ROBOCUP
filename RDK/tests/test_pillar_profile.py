import sys
import unittest
import tempfile
import cv2
import numpy as np
import yaml
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
sys.path.insert(0, str(ROOT))
from rdk_vision.config import load_config
from rdk_vision.ball_detector import DetectionResult, BallDetector
from pillar_profile import load_pillar_profile
from rdk_stm32_bridge import run_pillar_in_process


class PillarProfileTests(unittest.TestCase):
    def test_real_detector_accepts_ball_at_reference_position(self):
        base = load_config(ROOT / 'rdk_vision/config.yaml')
        config, trigger = load_pillar_profile(base, ROOT / 'rdk_vision/pillar.yaml')
        frame = np.zeros((480,640,3), dtype=np.uint8)
        cv2.circle(frame, (342,199), 74, (0,0,255), -1)
        result = BallDetector(config).detect(frame, 'red')
        self.assertTrue(result.valid, result.reason)
        self.assertEqual(trigger.update(result), 'TRIGGER_VALID')
        self.assertFalse(BallDetector(base).detect(frame, 'red').valid)

    def test_invalid_profile_fails_instead_of_using_disc_geometry(self):
        base = load_config(ROOT / 'rdk_vision/config.yaml')
        profile = yaml.safe_load((ROOT / 'rdk_vision/pillar.yaml').read_text())
        for key, value in [('center_x', 999), ('tolerance_x', 0), ('center_y', float('nan'))]:
            changed = dict(profile, grab=dict(profile['grab'], **{key:value}))
            with tempfile.TemporaryDirectory() as folder:
                path = Path(folder) / 'pillar.yaml'
                path.write_text(yaml.safe_dump(changed))
                with self.assertRaises(ValueError): load_pillar_profile(base, path)

    def test_independent_roi_preserves_shared_calibration(self):
        base = load_config(ROOT / 'rdk_vision/config.yaml')
        config, trigger = load_pillar_profile(base, ROOT / 'rdk_vision/pillar.yaml')
        self.assertNotEqual(config.roi, base.roi)
        self.assertEqual(config.colors, base.colors)
        self.assertEqual(config.ball, base.ball)
        self.assertEqual(config.camera, base.camera)
        self.assertEqual(trigger.update(DetectionResult(True, 'ok', (266,125,152,148))), 'TRIGGER_VALID')

    def test_no_early_or_edge_trigger_outside_grab_position(self):
        base = load_config(ROOT / 'rdk_vision/config.yaml')
        _, trigger = load_pillar_profile(base, ROOT / 'rdk_vision/pillar.yaml')
        self.assertIsNone(trigger.update(DetectionResult(True, 'ok', (300,125,152,148))))
        self.assertIsNone(trigger.update(DetectionResult(True, 'ok', (266,155,152,148))))
        self.assertIsNone(trigger.update(DetectionResult(False, 'edge_margin', (266,125,152,148))))

    def test_pillar_runner_uses_disc_vision_and_g104(self):
        with patch('rdk_stm32_bridge.run_disc_task', return_value=0) as run:
            self.assertEqual(run_pillar_in_process(ROOT), 0)
            args = run.call_args.args[0]
            self.assertEqual((args.prep_group, args.trigger_group), (103,104))
            self.assertEqual(args.trigger_x, 380)
            self.assertEqual(Path(args.config), ROOT / 'rdk_vision/config.yaml')
            trigger = run.call_args.kwargs['trigger']
            self.assertEqual(trigger.update(DetectionResult(True, 'ok', (266,125,152,148))), 'TRIGGER_VALID')
            self.assertIsNone(trigger.update(DetectionResult(False, 'edge_margin', (92,125,100,148))))
            self.assertEqual(run.call_args.kwargs['config'], load_config(ROOT / 'rdk_vision/config.yaml'))


if __name__ == '__main__':
    unittest.main()
