import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'tools'))
from rdk_vision.config import load_config
from rdk_vision.task_config import load_task_config
from rdk_stm32_bridge import run_disc_in_process, run_pillar_in_process


class TaskConfigTests(unittest.TestCase):
    def test_defaults_preserve_calibration_and_separate_stair_size(self):
        path = ROOT / 'rdk_vision/config.yaml'
        base = load_config(path)
        for task in ('disc', 'pillar'):
            self.assertEqual(load_task_config(path, task), base)
        stair = load_task_config(path, 'stair')
        self.assertEqual((stair.ball.reference_width, stair.ball.reference_height), (76, 77))
        self.assertEqual(stair.colors, base.colors)
        self.assertEqual(stair.roi, base.roi)

    def test_runners_select_independent_sizes(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            config_dir = root / 'rdk_vision'
            config_dir.mkdir()
            (config_dir / 'config.yaml').write_bytes((ROOT / 'rdk_vision/config.yaml').read_bytes())
            (config_dir / 'task_sizes.yaml').write_text(yaml.safe_dump({
                'disc': {'reference_width': 120},
                'pillar': {'reference_width': 90},
            }))
            with patch('rdk_stm32_bridge.run_disc_task', return_value=0) as run:
                run_disc_in_process(root, rfid_gate=None, on_action_complete=None)
                self.assertEqual(run.call_args.kwargs['config'].ball.reference_width, 120)
                independent = yaml.safe_load((root / 'rdk_vision/config.yaml').read_text())
                independent['ball']['reference_width'] = 95
                (root / 'rdk_vision/pillar_runtime.yaml').write_text(yaml.safe_dump(independent))
                run_pillar_in_process(root)
                self.assertEqual(run.call_args.kwargs['config'].ball.reference_width, 95)
                self.assertEqual(run.call_args.args[0].trigger_group, 104)

    def test_invalid_overrides_fail(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'config.yaml'
            path.write_bytes((ROOT / 'rdk_vision/config.yaml').read_bytes())
            for overrides in ({'reference_width': 0}, {'size_min_scale': 2},
                              {'size_max_scale': float('nan')}, {'roi': {}},
                              {'reference_width': True}):
                path.with_name('task_sizes.yaml').write_text(yaml.safe_dump({'stair': overrides}))
                with self.assertRaises(ValueError):
                    load_task_config(path, 'stair')


if __name__ == '__main__':
    unittest.main()
