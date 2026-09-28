import sys
import unittest
from pathlib import Path
from unittest.mock import Mock, patch
from dataclasses import replace

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / 'tools')]
from rdk_vision.config import load_config
from shared_task_camera import SharedTaskCamera
from stair_task import run_stair_point
from disc_rfid_gate import DiscRfidGate
from rdk_stm32_bridge import BridgeCore
from stair_task import recognize_point
from types import SimpleNamespace


class SharedCameraTests(unittest.TestCase):
    def test_group3_closes_stream_before_arm_action(self):
        events = []
        core = BridgeCore(lambda _: None, lambda **kw: 0,
                          close_camera=lambda: events.append('close'),
                          run_group=lambda group: events.append(group))
        core.handle('GROUP 3')
        self.assertTrue(core.wait_for_idle(1))
        self.assertEqual(events, ['close', 3])

    def test_idle_cancel_closes_retained_stream(self):
        close = Mock()
        core = BridgeCore(lambda _: None, lambda **kw: 0, close_camera=close)
        core.handle('PILLAR_CANCEL')
        core.tick()
        close.assert_called_once()

    def test_stair_exception_releases_stream(self):
        close = Mock()
        run = Mock(side_effect=TimeoutError('camera stalled'))
        lines = []
        core = BridgeCore(lines.append, lambda **kw: 0, run_stair=run, close_camera=close)
        core.handle('STAIR_CHECK 1')
        self.assertTrue(core.wait_for_idle(1))
        close.assert_called_once()
        self.assertEqual(lines[-1], 'STAIR_ERROR 1')

    def test_frame_from_approach_cannot_trigger_at_point(self):
        now = [10.0]
        camera = Mock()
        camera.get_latest.side_effect = lambda: SimpleNamespace(
            frame='old' if now[0] < 10.02 else 'new',
            frame_id=int(now[0]*1000),
            timestamp=9.99 if now[0] < 10.02 else now[0])
        detector = Mock()
        detector.detect.return_value = SimpleNamespace(valid=False)
        grab = Mock()
        self.assertFalse(recognize_point(camera, detector, DiscRfidGate(1), grab,
            lambda _: None, stale_ms=250, clock=lambda: now[0],
            sleep=lambda dt: now.__setitem__(0, now[0]+dt)))
        self.assertTrue(detector.detect.called)
        self.assertTrue(all(c.args[0] == 'new' for c in detector.detect.call_args_list))
        grab.assert_not_called()

    def test_pillar_and_stair_leases_warm_only_once(self):
        factory = Mock()
        session = SharedTaskCamera(factory)
        config = load_config(ROOT / 'rdk_vision/pillar_runtime.yaml').camera
        for _ in range(9):
            camera = session.borrow(config)
            camera.start()
            camera.stop()
        factory.assert_called_once_with(config)
        factory.return_value.start.assert_called_once()
        factory.return_value.stop.assert_not_called()
        session.close()
        session.close()
        factory.return_value.stop.assert_called_once()

    def test_incompatible_stream_rejected_without_restarting(self):
        config = load_config(ROOT / 'rdk_vision/stair_low.yaml').camera
        session = SharedTaskCamera(Mock())
        session.borrow(config).start()
        with self.assertRaises(ValueError):
            session.borrow(replace(config, width=1280))
        session.close()

    def test_stair_reuses_camera_but_loads_each_detector(self):
        session = SharedTaskCamera(Mock())
        with patch('stair_task.HiwonderActionBoard'), patch('stair_task.recognize_point', return_value=False) as recognize:
            for point, level in [(1, 'low'), (3, 'high'), (7, 'mid')]:
                config = load_config(ROOT / 'rdk_vision' / f'stair_{level}.yaml')
                run_stair_point(ROOT, point, rfid_gate=DiscRfidGate(1),
                                on_action_complete=lambda _: None,
                                camera=session.borrow(config.camera))
                self.assertEqual(recognize.call_args.args[1].config, config)
        session.close()

    def test_failed_start_releases_camera_and_can_retry(self):
        factory = Mock()
        factory.return_value.start.side_effect = [RuntimeError('camera failed'), None]
        session = SharedTaskCamera(factory)
        config = load_config(ROOT / 'rdk_vision/stair_low.yaml').camera
        with self.assertRaises(RuntimeError):
            session.borrow(config).start()
        factory.return_value.stop.assert_called_once()
        session.borrow(config).start()
        self.assertEqual(factory.call_count, 2)
        session.close()
