import unittest
import threading
from unittest.mock import patch

import numpy as np

from rdk_vision.camera import LatestFrameCamera
from rdk_vision.config import CameraConfig


class FakeCapture:
    def __init__(self, frames):
        self.frames = list(frames)
        self.released = False
        self.props = []
        self.read_count = 0

    def isOpened(self):
        return True

    def set(self, prop, value):
        self.props.append((prop, value))
        return True

    def read(self):
        self.read_count += 1
        if not self.frames:
            return False, None
        return True, self.frames.pop(0)

    def release(self):
        self.released = True


class CameraTests(unittest.TestCase):
    def test_cancel_before_start_does_not_open_device(self):
        opened = []
        camera = LatestFrameCamera(self.config(), capture_factory=lambda _: opened.append(True))
        camera.abort_start()
        with self.assertRaisesRegex(RuntimeError, 'cancelled'):
            camera.start()
        self.assertEqual(opened, [])

    def test_stop_during_stream_read_does_not_release_until_read_returns(self):
        entered, resume = threading.Event(), threading.Event()
        capture = FakeCapture([])
        def read():
            entered.set()
            resume.wait(4)
            self.assertFalse(capture.released)
            return True, np.zeros((2, 2, 3), dtype=np.uint8)
        capture.read = read
        camera = LatestFrameCamera(self.config(), capture_factory=lambda _: capture)
        camera.start()
        try:
            self.assertTrue(entered.wait(1))
            camera.stop()
            self.assertFalse(capture.released)
            self.assertIsNone(camera.get_latest())
        finally:
            resume.set()
            camera._thread.join(2)
            camera.stop()
        self.assertFalse(camera.is_running)
        self.assertTrue(capture.released)
        self.assertIsNone(camera.get_latest())

    def test_abort_during_warmup_releases_after_read_without_starting_stream(self):
        entered, resume = threading.Event(), threading.Event()
        capture = FakeCapture([np.zeros((2, 2, 3), dtype=np.uint8)])
        original_read = capture.read
        def read():
            entered.set()
            resume.wait(3)
            self.assertFalse(capture.released, 'release raced with read')
            return original_read()
        capture.read = read
        camera = LatestFrameCamera(CameraConfig('/dev/video0', 640, 480, 30, 250, 5000),
                                   capture_factory=lambda _: capture)
        errors = []
        def start():
            try:
                camera.start()
            except RuntimeError as exc:
                errors.append(exc)
        with patch('subprocess.run'):
            worker = threading.Thread(target=start)
            worker.start()
            try:
                self.assertTrue(entered.wait(1))
                camera.abort_start()
                self.assertFalse(capture.released)
            finally:
                resume.set()
                worker.join(3)
                camera.stop()
        self.assertFalse(worker.is_alive())
        self.assertTrue(capture.released)
        self.assertFalse(camera.is_running)
        self.assertIsNone(camera.get_latest())
        self.assertTrue(errors)

    def test_property_failure_releases_capture(self):
        capture = FakeCapture([])
        capture.set = lambda *args: (_ for _ in ()).throw(RuntimeError('property failed'))
        camera = LatestFrameCamera(self.config(), capture_factory=lambda _: capture)
        with self.assertRaisesRegex(RuntimeError, 'property failed'):
            camera.start()
        self.assertTrue(capture.released)

    def config(self):
        return CameraConfig("/dev/fake", 640, 480, 30, 250, 5000)

    def test_latest_snapshot_replaces_older_frames_and_id_increments(self):
        first = np.zeros((480, 640, 3), dtype=np.uint8)
        second = np.ones((480, 640, 3), dtype=np.uint8)
        capture = FakeCapture([first, second])
        times = iter([10.0, 10.033])
        camera = LatestFrameCamera(
            self.config(),
            capture_factory=lambda _: capture,
            clock=lambda: next(times),
        )
        self.assertTrue(camera.capture_once())
        self.assertTrue(camera.capture_once())
        snapshot = camera.get_latest()
        self.assertEqual(snapshot.frame_id, 2)
        self.assertEqual(snapshot.timestamp, 10.033)
        self.assertTrue(np.array_equal(snapshot.frame, second))

    def test_failed_read_does_not_republish_old_frame(self):
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        capture = FakeCapture([frame])
        camera = LatestFrameCamera(self.config(), capture_factory=lambda _: capture, clock=lambda: 1.0)
        self.assertTrue(camera.capture_once())
        before = camera.get_latest()
        self.assertFalse(camera.capture_once())
        after = camera.get_latest()
        self.assertEqual(after.frame_id, before.frame_id)


    def test_video_device_primes_auto_white_balance_then_locks_before_publishing(self):
        warmup = [np.full((480, 640, 3), i, dtype=np.uint8) for i in range(45)]
        production = np.full((480, 640, 3), 99, dtype=np.uint8)
        capture = FakeCapture(warmup + [production])
        config = CameraConfig("/dev/video0", 640, 480, 30, 250, 5000)

        with patch("subprocess.run") as run:
            run.return_value.stdout = ""
            camera = LatestFrameCamera(
                config,
                capture_factory=lambda _: capture,
                clock=lambda: 3.0,
            )
            camera._open_capture()
            self.assertEqual(capture.read_count, 45)
            self.assertTrue(camera.capture_once())

        snapshot = camera.get_latest()
        self.assertTrue(np.array_equal(snapshot.frame, production))

        commands = [call.args[0] for call in run.call_args_list]
        joined = [" ".join(command) for command in commands]
        self.assertTrue(
            any("white_balance_automatic=1" in command for command in joined),
            joined,
        )
        self.assertTrue(
            any("white_balance_automatic=0" in command for command in joined),
            joined,
        )
        self.assertTrue(
            any("brightness=0" in command for command in joined),
            joined,
        )
        self.assertTrue(
            any("exposure_time_absolute=157" in command for command in joined),
            joined,
        )

    def test_start_wait_ready_and_stop_releases_capture(self):
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        capture = FakeCapture([frame])
        camera = LatestFrameCamera(
            self.config(),
            capture_factory=lambda _: capture,
            clock=lambda: 2.0,
        )
        camera.start()
        try:
            self.assertTrue(camera.wait_until_ready(500))
            self.assertIsNotNone(camera.get_latest())
        finally:
            camera.stop()
        self.assertTrue(capture.released)


if __name__ == "__main__":
    unittest.main()
