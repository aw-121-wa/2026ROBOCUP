import unittest

import numpy as np

from rdk_vision.ball_detector import DetectionResult
from rdk_vision.camera import FrameSnapshot
from rdk_vision.config import load_config
from rdk_vision.main import RecognitionSession, VisionService
from rdk_vision.serial_link import SerialLinkError


class RecognitionSessionTests(unittest.TestCase):
    def test_requires_frames_after_request(self):
        session = RecognitionSession(window_frames=3, required_hits=2)
        session.arm("red", request_frame_id=10)
        self.assertFalse(session.accept_frame(10, True))
        self.assertFalse(session.accept_frame(11, True))
        self.assertTrue(session.accept_frame(12, True))

    def test_two_hits_in_three_processed_frames_confirms(self):
        session = RecognitionSession(3, 2)
        session.arm("red", 5)
        self.assertFalse(session.accept_frame(6, True))
        self.assertFalse(session.accept_frame(7, False))
        self.assertTrue(session.accept_frame(8, True))

    def test_new_request_clears_old_history_and_switches_color(self):
        session = RecognitionSession(3, 2)
        session.arm("red", 5)
        self.assertFalse(session.accept_frame(6, True))
        session.arm("blue", 6)
        self.assertEqual(session.color, "blue")
        self.assertFalse(session.accept_frame(7, True))
        self.assertTrue(session.accept_frame(8, True))


class FakeCamera:
    def __init__(self, snapshot=None):
        self.snapshot = snapshot

    def get_latest(self):
        return self.snapshot


class FakeLink:
    def __init__(self):
        self.requests = []
        self.found_count = 0
        self.reconnect_count = 0

    def poll_requests(self):
        requests, self.requests = self.requests, []
        return requests

    def send_found(self):
        self.found_count += 1

    def reconnect(self):
        self.reconnect_count += 1
        return True


class FailOnceLink(FakeLink):
    def __init__(self):
        super().__init__()
        self.failed = False

    def poll_requests(self):
        if not self.failed:
            self.failed = True
            raise SerialLinkError("synthetic UART read failure")
        return super().poll_requests()


class SequenceDetector:
    def __init__(self, hits):
        self.hits = list(hits)
        self.colors = []

    def detect(self, frame, color):
        self.colors.append(color)
        hit = self.hits.pop(0)
        return DetectionResult(hit, "ok" if hit else "no_candidate")


class VisionServiceTests(unittest.TestCase):
    def setUp(self):
        self.cfg = load_config("rdk_vision/config.yaml")
        self.frame = np.zeros((480, 640, 3), dtype=np.uint8)
        self.now = [10.000]

    def service(self, hits):
        camera = FakeCamera(FrameSnapshot(self.frame, 100, 10.000))
        link = FakeLink()
        detector = SequenceDetector(hits)
        service = VisionService(
            camera, link, detector, self.cfg, clock=lambda: self.now[0]
        )
        return camera, link, detector, service

    def test_request_frame_is_skipped_then_two_new_hits_send_once(self):
        camera, link, detector, service = self.service([True, True])
        link.requests = ["red"]
        service.step()
        self.assertEqual(detector.colors, [])

        camera.snapshot = FrameSnapshot(self.frame, 101, 10.033)
        self.now[0] = 10.034
        service.step()
        self.assertEqual(link.found_count, 0)

        camera.snapshot = FrameSnapshot(self.frame, 102, 10.066)
        self.now[0] = 10.067
        service.step()
        self.assertEqual(link.found_count, 1)

        camera.snapshot = FrameSnapshot(self.frame, 103, 10.099)
        self.now[0] = 10.100
        service.step()
        self.assertEqual(link.found_count, 1)

    def test_blue_request_uses_blue_detector_mode(self):
        camera, link, detector, service = self.service([False])
        link.requests = ["blue"]
        service.step()
        camera.snapshot = FrameSnapshot(self.frame, 101, 10.033)
        self.now[0] = 10.034
        service.step()
        self.assertEqual(detector.colors, ["blue"])

    def test_stale_frame_is_not_evaluated(self):
        camera, link, detector, service = self.service([True])
        link.requests = ["red"]
        service.step()
        camera.snapshot = FrameSnapshot(self.frame, 101, 10.033)
        self.now[0] = 10.400
        service.step()
        self.assertEqual(detector.colors, [])
        self.assertEqual(link.found_count, 0)

    def test_uart_recovery_clears_active_request(self):
        camera = FakeCamera(FrameSnapshot(self.frame, 100, 10.000))
        link = FailOnceLink()
        detector = SequenceDetector([])
        service = VisionService(
            camera, link, detector, self.cfg, clock=lambda: self.now[0], sleep_fn=lambda _seconds: None
        )
        service.session.arm("red", 99)
        service.run(max_steps=1)
        self.assertEqual(link.reconnect_count, 1)
        self.assertFalse(service.session.armed)

    def test_two_requests_in_one_process_each_emit_once(self):
        cfg = load_config("rdk_vision/config.yaml")
        camera = FakeCamera(FrameSnapshot(np.zeros((480, 640, 3), dtype=np.uint8), 100, 10.000))
        link = FakeLink()
        detector = SequenceDetector([True, True, True, True])
        now = [10.000]
        service = VisionService(camera, link, detector, cfg, clock=lambda: now[0])

        link.requests = ["red"]
        service.step()

        camera.snapshot = FrameSnapshot(camera.snapshot.frame, 101, 10.033)
        now[0] = 10.034
        service.step()
        camera.snapshot = FrameSnapshot(camera.snapshot.frame, 102, 10.066)
        now[0] = 10.067
        service.step()
        self.assertEqual(link.found_count, 1)

        camera.snapshot = FrameSnapshot(camera.snapshot.frame, 103, 10.099)
        now[0] = 10.100
        service.step()
        self.assertEqual(link.found_count, 1)

        link.requests = ["red"]
        service.step()
        camera.snapshot = FrameSnapshot(camera.snapshot.frame, 104, 10.132)
        now[0] = 10.133
        service.step()
        camera.snapshot = FrameSnapshot(camera.snapshot.frame, 105, 10.165)
        now[0] = 10.166
        service.step()
        self.assertEqual(link.found_count, 2)

    def test_stale_frame_never_advances_temporal_history(self):
        cfg = load_config("rdk_vision/config.yaml")
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        camera = FakeCamera(FrameSnapshot(frame, 200, 20.000))
        link = FakeLink()
        detector = SequenceDetector([True, True])
        now = [20.000]
        service = VisionService(camera, link, detector, cfg, clock=lambda: now[0])

        link.requests = ["red"]
        service.step()
        camera.snapshot = FrameSnapshot(frame, 201, 20.033)
        now[0] = 20.034
        service.step()
        self.assertEqual(link.found_count, 0)

        camera.snapshot = FrameSnapshot(frame, 202, 20.050)
        now[0] = 20.400
        service.step()
        service.step()
        self.assertEqual(link.found_count, 0)

        camera.snapshot = FrameSnapshot(frame, 203, 20.433)
        now[0] = 20.434
        service.step()
        self.assertEqual(link.found_count, 1)


if __name__ == "__main__":
    unittest.main()
