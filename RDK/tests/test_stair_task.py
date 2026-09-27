import sys
import unittest
import threading
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'tools'))
from stair_task import recognize_point, run_stair_point
from disc_rfid_gate import DiscRfidGate
from rdk_stm32_bridge import BridgeCore


class StairRecognitionTests(unittest.TestCase):
    def run_point(self, valid=False, stale=False, confirm=True, cancelled=False, detection_delay=0):
        now = [0.0]
        calls = []
        gate = DiscRfidGate(1)
        if cancelled:
            gate.cancel()
        class Camera:
            def get_latest(self):
                return SimpleNamespace(frame=None, frame_id=int(now[0]*1000),
                                       timestamp=-2 if stale else now[0])
        class Detector:
            def detect(self, frame, color):
                now[0] += detection_delay
                return SimpleNamespace(valid=valid)
        def sleep(dt):
            now[0] += dt
        def complete(index):
            calls.append(('done', index))
            if confirm:
                gate.on_rfid_confirmed(index)
        result = recognize_point(Camera(), Detector(), gate,
            lambda: calls.append(('grab',)), complete,
            clock=lambda: now[0], sleep=sleep, stale_ms=250)
        return result, calls, now[0]

    def test_valid_triggers_once_and_requires_rfid(self):
        result, calls, elapsed = self.run_point(valid=True)
        self.assertTrue(result)
        self.assertEqual(calls, [('grab',), ('done', 1)])
        self.assertLess(elapsed, 1)

    def test_no_ball_waits_one_second(self):
        result, calls, elapsed = self.run_point()
        self.assertFalse(result)
        self.assertFalse(calls)
        self.assertGreaterEqual(elapsed, 1)
        self.assertLess(elapsed, 1.1)

    def test_stale_camera_is_error_not_empty_point(self):
        with self.assertRaises(TimeoutError):
            self.run_point(valid=True, stale=True)

    def test_missing_rfid_stops_instead_of_reporting_success(self):
        with self.assertRaises(TimeoutError):
            self.run_point(valid=True, confirm=False)

    def test_cancel_prevents_action(self):
        with self.assertRaises(RuntimeError):
            self.run_point(valid=True, cancelled=True)

    def test_slow_detection_cannot_grab_from_expired_frame(self):
        with self.assertRaises(TimeoutError):
            self.run_point(valid=True, detection_delay=.3)

    def test_action_dispatch_and_cancel_are_serialized(self):
        gate = DiscRfidGate(1)
        started, release, cancelled = threading.Event(), threading.Event(), threading.Event()
        def dispatch():
            started.set()
            release.wait(1)
        def cancel():
            gate.cancel()
            cancelled.set()
        worker = threading.Thread(target=lambda: gate.dispatch_if_allowed(dispatch))
        worker.start()
        try:
            self.assertTrue(started.wait(1))
            stopper = threading.Thread(target=cancel)
            stopper.start()
            self.assertFalse(cancelled.wait(.02))
        finally:
            release.set()
            worker.join(1)
        stopper.join(1)
        self.assertTrue(cancelled.is_set())
        self.assertFalse(gate.dispatch_if_allowed(lambda: self.fail('dispatched after cancel')))

    def test_stair_uses_independent_roi_without_shared_config(self):
        import tempfile
        from dataclasses import replace
        from rdk_vision.config import load_config, save_config
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / 'rdk_vision').mkdir()
            config = load_config(ROOT / 'rdk_vision/config.yaml')
            config = replace(config, roi=replace(config.roi, height=260))
            save_config(config, root / 'rdk_vision/stair_runtime.yaml')
            with patch('stair_task.LatestFrameCamera'), \
                 patch('stair_task.HiwonderActionBoard'), \
                 patch('stair_task.recognize_point', return_value=False) as recognize:
                run_stair_point(root, 1, rfid_gate=DiscRfidGate(1),
                                on_action_complete=lambda _: None)
                self.assertEqual(recognize.call_args.args[1].config.roi.height, 260)

    def test_point_groups_and_stair_profile(self):
        with patch('stair_task.LatestFrameCamera') as camera, \
             patch('stair_task.HiwonderActionBoard') as board, \
             patch('stair_task.recognize_point') as recognize:
            def found(cam, detector, gate, grab, complete, **kwargs):
                grab()
                kwargs['wait_complete']()
                return True
            recognize.side_effect = found
            for point, group in enumerate((106,106,107,107,107,107,108,108), 1):
                self.assertTrue(run_stair_point(ROOT, point, rfid_gate=DiscRfidGate(1),
                                               on_action_complete=lambda _: None))
                board.return_value.__enter__.return_value.start_group.assert_called_with(group)
                board.return_value.__enter__.return_value.wait_group_complete.assert_called_with(group, timeout_s=30.0)
                self.assertEqual(recognize.call_args.args[1].config.ball.reference_width, 76)
            self.assertEqual(camera.return_value.stop.call_count, 8)


class StairBridgeTests(unittest.TestCase):
    def test_no_ball_and_group_commands(self):
        lines, groups = [], []
        core = BridgeCore(lines.append, lambda **kw: 0,
            run_stair=lambda point, **kw: False, run_group=groups.append)
        for group in (2, 4, 105):
            core.handle(f'GROUP {group}')
            self.assertTrue(core.wait_for_idle(1))
            self.assertIn(f'GROUP_DONE {group}', lines)
        core.handle('STAIR_CHECK 4')
        self.assertTrue(core.wait_for_idle(1))
        self.assertEqual(lines[-2:], ['STAIR_ACK 4', 'STAIR_NONE 4'])
        self.assertEqual(groups, [2, 4, 105])

    def test_only_matching_rfid_releases_point(self):
        lines = []
        waiting, release = threading.Event(), threading.Event()
        def run(point, rfid_gate, on_action_complete):
            self.assertEqual(point, 7)
            rfid_gate.on_action_complete(1)
            on_action_complete(1)
            waiting.set()
            release.wait(1)
            return True
        core = BridgeCore(lines.append, lambda **kw: 0, run_stair=run)
        try:
            core.handle('STAIR_CHECK 7')
            self.assertTrue(waiting.wait(1))
            core.handle('STAIR_RFID_OK 6')
            self.assertFalse(core.gate.is_complete())
            core.handle('DISC_RFID_OK 1')
            self.assertFalse(core.gate.is_complete())
            core.handle('STAIR_RFID_OK 7')
            self.assertTrue(core.gate.is_complete())
        finally:
            release.set()
            core.wait_for_idle(1)
        self.assertEqual(lines, ['STAIR_ACK 7', 'STAIR_ACTION_DONE 7', 'STAIR_DONE 7'])

    def test_cancel_during_action_never_reports_done(self):
        lines = []
        started, release = threading.Event(), threading.Event()
        def run(point, rfid_gate, on_action_complete):
            started.set()
            release.wait(1)
            return False
        core = BridgeCore(lines.append, lambda **kw: 0, run_stair=run)
        try:
            core.handle('STAIR_CHECK 1')
            self.assertTrue(started.wait(1))
            core.handle('DISC_CANCEL')
        finally:
            release.set()
            core.wait_for_idle(1)
        self.assertEqual(lines[-1], 'STAIR_ERROR 1')
