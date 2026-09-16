"""Run with: python -m unittest discover -s tests -p test_path_bridge.py -v."""
import importlib.util
from concurrent.futures import Future
from pathlib import Path
import unittest
from types import SimpleNamespace
from unittest.mock import patch

spec = importlib.util.spec_from_file_location(
    "path_bridge", Path(__file__).resolve().parents[1] / "tools/rdk/path_bridge.py")
bridge = importlib.util.module_from_spec(spec)
spec.loader.exec_module(bridge)


class Worker:
    def __init__(self):
        self.jobs = []

    def submit(self, op, arg, cancel):
        future = Future()
        self.jobs.append((op, arg, cancel, future))
        return future


class ProtocolTests(unittest.TestCase):
    def setUp(self):
        self.worker = Worker()
        self.output = []
        self.engine = bridge.Engine(self.worker.submit, self.output.append)
        self.engine.accept("Q 123 1 HELLO 0")
        self.assertEqual(self.output[-1], "R 123 1 DONE")

    def test_group_requires_session_and_duplicate_does_not_move_twice(self):
        self.engine.accept("Q 456 2 GROUP 101")
        self.assertEqual(self.output[-1], "R 456 2 ERROR")
        self.assertFalse(self.worker.jobs)
        self.engine.accept("Q 123 2 GROUP 101")
        self.engine.accept("Q 123 2 GROUP 101")
        self.assertEqual(len(self.worker.jobs), 1)
        self.assertEqual(self.worker.jobs[0][:2], ("GROUP", 101))
        self.worker.jobs[0][3].set_result("DONE")
        self.engine.poll()
        self.engine.accept("Q 123 2 GROUP 101")
        self.assertEqual(self.output[-1], "R 123 2 DONE")
        self.assertEqual(len(self.worker.jobs), 1)

    def test_busy_rejects_new_group_but_stop_waits_for_action(self):
        self.engine.accept("Q 123 2 DISC 20000")
        self.engine.accept("Q 123 3 GROUP 0")
        self.assertEqual(self.output[-1], "R 123 3 ERROR")
        self.engine.accept("Q 123 4 STOP 0")
        self.assertEqual(self.output[-1], "R 123 4 ACK")
        self.assertTrue(self.worker.jobs[0][2].is_set())
        self.assertNotIn("R 123 4 DONE", self.output)
        self.worker.jobs[0][3].set_result("DONE")
        self.engine.poll()
        self.assertEqual(self.output[-1], "R 123 4 DONE")
        self.assertEqual(len(self.worker.jobs), 1)

    def test_error_latches_unknown_arm_state_and_stop_not_false_done(self):
        self.engine.accept("Q 123 2 GROUP 102")
        self.worker.jobs[0][3].set_exception(TimeoutError("missing servo completion"))
        self.engine.poll()
        self.engine.accept("Q 123 3 STOP 0")
        self.assertEqual(self.output[-1], "R 123 3 ERROR")
        self.engine.accept("Q 789 1 HELLO 0")
        self.assertEqual(self.output[-1], "R 789 1 ERROR")
        self.assertEqual(len(self.worker.jobs), 1)

    def test_bad_arguments_and_conflicting_duplicate_cannot_start(self):
        for line in ("Q 123 2 GROUP 256", "Q 123 2 DISC 0", "Q 123 2 VISION -1",
                     "Q 123 2 GROUP 0 extra", "Q 123 4294967296 GROUP 1"):
            self.engine.accept(line)
        self.assertFalse(self.worker.jobs)
        self.engine.accept("Q 123 3 GROUP 101")
        self.engine.accept("Q 123 3 GROUP 102")
        self.assertEqual(self.output[-1], "R 123 3 ERROR")
        self.assertEqual(len(self.worker.jobs), 1)

    def test_retired_session_and_evicted_sequence_never_reexecute(self):
        for seq in range(2, 25):
            self.engine.accept(f"Q 123 {seq} STOP 0")
        self.engine.accept("Q 123 2 GROUP 102")
        self.assertEqual(self.output[-1], "R 123 2 ERROR")
        self.engine.accept("Q 456 1 HELLO 0")
        self.engine.accept("Q 123 1 HELLO 0")
        self.assertEqual(self.output[-1], "R 123 1 ERROR")
        self.assertFalse(self.worker.jobs)

    def test_line_overflow_invalid_ascii_and_stale_fragment_are_discarded(self):
        lines = bridge.LineReader()
        self.assertEqual(lines.feed(b"X" * 200 + b"Q 1 1 HELLO 0\n", 0), [])
        self.assertEqual(lines.feed(b"Q 1 1 HELLO 0\r\n", 0.1), ["Q 1 1 HELLO 0"])
        self.assertEqual(lines.feed(b"Q 1 2 GR", 0.2), [])
        self.assertEqual(lines.feed(b"OUP 102\n", 2), [])
        self.assertEqual(lines.feed(b"Q 1 3 GR\xffOUP 102\n", 2.1), [])

    def test_session_capacity_is_fail_closed_not_replayable(self):
        for sid in range(1000, 1034):
            self.engine.accept(f"Q {sid} 1 HELLO 0")
        self.engine.accept("Q 123 1 HELLO 0")
        self.assertEqual(self.output[-1], "R 123 1 ERROR")
        self.engine.accept("Q 123 2 GROUP 102")
        self.assertFalse(self.worker.jobs)

    def test_lost_request_lease_cancels_detection_without_false_completion(self):
        with patch.object(bridge.time, "monotonic", return_value=10):
            self.engine.accept("Q 123 2 DISC 20000")
        with patch.object(bridge.time, "monotonic", return_value=11):
            self.engine.accept("Q 123 2 DISC 20000")
        with patch.object(bridge.time, "monotonic", return_value=12.5):
            self.engine.poll()
        self.assertFalse(self.worker.jobs[0][2].is_set())
        with patch.object(bridge.time, "monotonic", return_value=13.1):
            self.engine.poll()
        self.assertTrue(self.worker.jobs[0][2].is_set())
        self.assertNotIn("R 123 2 DONE", self.output)

    def test_old_hello_cannot_renew_active_disc_lease(self):
        with patch.object(bridge.time, "monotonic", return_value=10):
            self.engine.accept("Q 123 2 DISC 20000")
        with patch.object(bridge.time, "monotonic", return_value=11.9):
            self.engine.accept("Q 123 1 HELLO 0")
        with patch.object(bridge.time, "monotonic", return_value=12.1):
            self.engine.poll()
        self.assertTrue(self.worker.jobs[0][2].is_set())


class HardwareTests(unittest.TestCase):
    def make_hardware(self, *, fresh=True, valid=True, event=None, detector_delay=0):
        self.now = 0.0
        self.calls = []
        frame_number = 0
        def frame():
            nonlocal frame_number
            if fresh or frame_number == 0:
                frame_number += 1
            return SimpleNamespace(frame_id=frame_number, timestamp=self.now,
                                   frame=SimpleNamespace(shape=(480, 640, 3)))
        def detect(_image, _color):
            self.now += detector_delay
            return SimpleNamespace(valid=valid, reason="ok" if valid else "edge_margin",
                                   bbox=(350, 80, 94, 120), aspect_ratio=0.783,
                                   fill_ratio=0.7)
        class Cancel:
            def is_set(_self):
                return False
            def wait(_self, _seconds):
                self.now += 0.1
        hardware = bridge.VisionHardware.__new__(bridge.VisionHardware)
        hardware.config = SimpleNamespace(camera=SimpleNamespace(width=640, height=480, stale_ms=250),
                                          roi=SimpleNamespace(x=92, y=29, width=352, height=244))
        hardware.camera = SimpleNamespace(get_latest=frame)
        hardware.detector = SimpleNamespace(detect=detect)
        hardware.gate = SimpleNamespace(update=lambda _result: event)
        hardware.board = SimpleNamespace(run_group=lambda group, **kwargs: self.calls.append(group))
        hardware.color = "red"
        hardware.action_timeout = 30
        return hardware, Cancel()

    def test_frozen_camera_raises_before_long_recognition_deadline(self):
        hardware, cancel = self.make_hardware(fresh=False)
        with patch.object(bridge.time, "monotonic", side_effect=lambda: self.now):
            with self.assertRaisesRegex(TimeoutError, "new frames"):
                hardware.run("DISC", 20000, cancel)
        self.assertLess(self.now, 2)
        self.assertFalse(self.calls)

    def test_detection_that_crosses_deadline_cannot_launch_group(self):
        hardware, cancel = self.make_hardware(event="TRIGGER_VALID", detector_delay=0.2)
        with patch.object(bridge.time, "monotonic", side_effect=lambda: self.now):
            self.assertEqual(hardware.run("DISC", 100, cancel), "NONE")
        self.assertFalse(self.calls)

    def test_false_full_valid_but_edge_trigger_runs_exactly_one_g102(self):
        hardware, cancel = self.make_hardware(valid=False, event="TRIGGER_EARLY")
        with patch.object(bridge.time, "monotonic", side_effect=lambda: self.now):
            self.assertEqual(hardware.run("DISC", 20000, cancel), "DONE")
        self.assertEqual(self.calls, [102])

    def test_vision_reports_found_without_moving_arm(self):
        hardware, cancel = self.make_hardware()
        with patch.object(bridge.time, "monotonic", side_effect=lambda: self.now):
            self.assertEqual(hardware.run("VISION", 1000, cancel), "DONE")
        self.assertFalse(self.calls)


if __name__ == "__main__":
    unittest.main()
