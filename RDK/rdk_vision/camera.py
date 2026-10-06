from dataclasses import dataclass
import subprocess
import threading
import time
from typing import Optional

import cv2
import numpy as np

# Validated on the RDK X5 + Sonix USB camera used by this project.
# The camera can power up with an uninitialized internal color-gain state even
# when white_balance_temperature reads 4600.  A short AWB streaming warm-up
# followed by locking AWB prevents the severe magenta cast seen after cold boot.
_UVC_BASE_CONTROLS = {
    "brightness": 0,
    "contrast": 32,
    "saturation": 64,
    "hue": 0,
    "gamma": 100,
    "gain": 0,
    "auto_exposure": 1,
    "exposure_time_absolute": 157,
}
_AWB_WARMUP_FRAMES = 45


@dataclass(frozen=True)
class FrameSnapshot:
    frame: np.ndarray
    frame_id: int
    timestamp: float


class LatestFrameCamera:
    def __init__(self, config, capture_factory=None, clock=None):
        self.config = config
        self._capture_factory = capture_factory or (
            lambda device: cv2.VideoCapture(device, cv2.CAP_V4L2)
        )
        self._clock = clock or time.monotonic
        self._capture = None
        self._latest: Optional[FrameSnapshot] = None
        self._frame_id = 0
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread = None
        self._io_lock = threading.RLock()
        self._aborted = threading.Event()

    def _check_cancelled(self):
        if self._stop_event.is_set() or self._aborted.is_set():
            raise RuntimeError('camera startup cancelled')

    def abort_start(self):
        # A failed startup instance must never be revived by a late helper.
        self._aborted.set()
        self.stop()

    def _release_capture(self):
        capture, self._capture = self._capture, None
        if capture is not None:
            capture.release()

    @property
    def is_running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def _set_v4l2_controls(self, controls) -> None:
        if not str(self.config.device).startswith(("/dev/video", "/dev/v4l/")):
            return
        assignment = ",".join(f"{name}={value}" for name, value in controls.items())
        subprocess.run(
            [
                "v4l2-ctl",
                "-d",
                str(self.config.device),
                f"--set-ctrl={assignment}",
            ],
            check=True,
            capture_output=True,
            text=True,
            timeout=2.0,
        )

    def _read_v4l2_controls(self) -> str:
        if not str(self.config.device).startswith(("/dev/video", "/dev/v4l/")):
            return ""
        names = list(_UVC_BASE_CONTROLS) + [
            "white_balance_automatic",
            "white_balance_temperature",
        ]
        result = subprocess.run(
            [
                "v4l2-ctl",
                "-d",
                str(self.config.device),
                f"--get-ctrl={','.join(names)}",
            ],
            check=False,
            capture_output=True,
            text=True,
            timeout=2.0,
        )
        if result.returncode != 0:
            return ""
        return result.stdout.strip()

    def _prime_auto_white_balance(self, capture) -> None:
        if not str(self.config.device).startswith(("/dev/video", "/dev/v4l/")):
            return

        # Apply the known-good baseline first, then explicitly enable AWB.
        self._set_v4l2_controls(_UVC_BASE_CONTROLS)
        self._set_v4l2_controls({"white_balance_automatic": 1})
        print(
            f"CAMERA: AWB warm-up started; discarding {_AWB_WARMUP_FRAMES} frames",
            flush=True,
        )

        valid_frames = 0
        for _ in range(_AWB_WARMUP_FRAMES):
            self._check_cancelled()
            ok, frame = capture.read()
            self._check_cancelled()
            if ok and frame is not None:
                valid_frames += 1
            else:
                time.sleep(0.01)

        if valid_frames == 0:
            raise RuntimeError("camera produced no valid frame during AWB warm-up")

        # Lock the internal gains reached by AWB.  Do not write temperature here:
        # this camera reports 4600 in both the bad-magenta and good states.
        self._set_v4l2_controls({"white_balance_automatic": 0})
        print(
            f"CAMERA: AWB locked after {valid_frames}/{_AWB_WARMUP_FRAMES} "
            "valid warm-up frames",
            flush=True,
        )
        controls = self._read_v4l2_controls()
        if controls:
            print(f"CAMERA: controls after lock:\n{controls}", flush=True)

    def _open_capture(self) -> None:
        if self._capture is not None:
            return
        capture = self._capture_factory(self.config.device)
        try:
            self._check_cancelled()
            if capture is None or not capture.isOpened():
                raise RuntimeError(f"failed to open camera {self.config.device}")
            # Keep the validated stream settings and their order unchanged.
            capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"YUYV"))
            capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.config.width)
            capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.config.height)
            capture.set(cv2.CAP_PROP_FPS, self.config.fps)
            capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            self._prime_auto_white_balance(capture)
            self._check_cancelled()
        except BaseException:
            if capture is not None:
                capture.release()
            raise
        self._capture = capture

    def capture_once(self) -> bool:
        with self._io_lock:
            if self._stop_event.is_set():
                return False
            if self._capture is None:
                self._open_capture()
            ok, frame = self._capture.read()
            if self._stop_event.is_set() or not ok or frame is None:
                return False
            timestamp = self._clock()
            with self._lock:
                self._frame_id += 1
                self._latest = FrameSnapshot(frame=frame, frame_id=self._frame_id, timestamp=timestamp)
            return True

    def get_latest(self) -> Optional[FrameSnapshot]:
        with self._lock:
            return self._latest

    def _capture_loop(self) -> None:
        try:
            while not self._stop_event.is_set():
                if not self.capture_once():
                    self._stop_event.wait(0.01)
        finally:
            with self._io_lock:
                self._release_capture()
            with self._lock:
                self._latest = None

    def start(self) -> None:
        with self._io_lock:
            if self._aborted.is_set():
                raise RuntimeError('camera startup cancelled')
            if self.is_running:
                if self._stop_event.is_set():
                    raise RuntimeError('camera is still stopping')
                return
            self._stop_event.clear()
            try:
                self._open_capture()
                self._check_cancelled()
                self._thread = threading.Thread(
                    target=self._capture_loop, name="rdk-camera", daemon=True)
                self._thread.start()
            except BaseException:
                self._release_capture()
                raise

    def wait_until_ready(self, timeout_ms: int) -> bool:
        deadline = time.monotonic() + timeout_ms / 1000.0
        while time.monotonic() < deadline:
            if self._stop_event.is_set():
                return False
            if self.get_latest() is not None:
                return True
            time.sleep(0.005)
        return self.get_latest() is not None

    def stop(self) -> None:
        self._stop_event.set()
        thread = self._thread
        if thread is not None and thread.is_alive() and thread is not threading.current_thread():
            thread.join(timeout=1.0)
        # Never release concurrently with an OpenCV read/open. The owner will
        # observe cancellation and release when the driver call returns.
        if self._io_lock.acquire(blocking=False):
            try:
                if thread is None or not thread.is_alive():
                    self._release_capture()
            finally:
                self._io_lock.release()
        with self._lock:
            self._latest = None
