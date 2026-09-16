import sys
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
TOOLS = ROOT / "tools"
for p in (ROOT, TOOLS):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

from vision_servo_direct_test import run_preparation_with_camera_warmup


class BlockingCamera:
    def __init__(self):
        self.start_entered = threading.Event()
        self.allow_start_finish = threading.Event()
        self.wait_ready_calls = []

    def start(self):
        self.start_entered.set()
        if not self.allow_start_finish.wait(timeout=1.0):
            raise RuntimeError("test camera was not released by G101")

    def wait_until_ready(self, timeout_ms):
        self.wait_ready_calls.append(timeout_ms)
        return True


class PrepBoard:
    def __init__(self, camera):
        self.camera = camera
        self.calls = []

    def run_group(self, group, *, repeat_count, timeout_s):
        # This assertion is the point of the regression test: the camera/AWB
        # startup must already be running when G101 begins.
        if not self.camera.start_entered.wait(timeout=0.5):
            raise AssertionError("camera startup did not overlap G101")
        self.calls.append((group, repeat_count, timeout_s))
        self.camera.allow_start_finish.set()
        return SimpleNamespace(raw=b"prep-ok")


class FailingCamera:
    def start(self):
        raise RuntimeError("awb startup failed")

    def wait_until_ready(self, timeout_ms):
        raise AssertionError("wait_until_ready must not run after startup failure")


class SimpleBoard:
    def __init__(self):
        self.calls = []

    def run_group(self, group, *, repeat_count, timeout_s):
        self.calls.append((group, repeat_count, timeout_s))
        return SimpleNamespace(raw=b"prep-ok")


class ParallelCameraPrepTests(unittest.TestCase):
    def test_camera_awb_startup_overlaps_g101_and_is_ready_after_prep(self):
        camera = BlockingCamera()
        board = PrepBoard(camera)

        prep = run_preparation_with_camera_warmup(
            board=board,
            camera=camera,
            prep_group=101,
            repeat_count=1,
            servo_timeout_s=30.0,
            camera_ready_timeout_ms=5000,
        )

        self.assertEqual(prep.raw, b"prep-ok")
        self.assertEqual(board.calls, [(101, 1, 30.0)])
        self.assertEqual(camera.wait_ready_calls, [5000])

    def test_camera_startup_failure_is_propagated_after_prep(self):
        camera = FailingCamera()
        board = SimpleBoard()

        with self.assertRaisesRegex(RuntimeError, "awb startup failed"):
            run_preparation_with_camera_warmup(
                board=board,
                camera=camera,
                prep_group=101,
                repeat_count=1,
                servo_timeout_s=30.0,
                camera_ready_timeout_ms=5000,
            )

        self.assertEqual(board.calls, [(101, 1, 30.0)])


if __name__ == "__main__":
    unittest.main()
