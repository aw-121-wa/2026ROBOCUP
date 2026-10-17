"""Camera ownership across pillar and stationary stair workers.

Only the serial bridge's single task worker uses a lease. The bridge closes
the owner after that worker exits, never while detection is reading a frame.
"""
import threading
import time
from rdk_vision.camera import LatestFrameCamera


class SharedTaskCamera:
    def __init__(self, factory=LatestFrameCamera):
        self._factory = factory
        self._camera = None
        self._config = None

    def borrow(self, config, *, cancelled=None, on_wait=None):
        if self._config is not None:
            keys = ('device', 'width', 'height', 'fps')
            if any(getattr(config, key) != getattr(self._config, key) for key in keys):
                raise ValueError('pillar/stair camera stream settings differ')
        return _CameraLease(self, config, cancelled=cancelled, on_wait=on_wait)

    def _start(self, config):
        if self._camera is not None:
            return
        self._camera = self._factory(config)
        self._config = config
        try:
            self._camera.start()
        except BaseException:
            self.close()
            raise
        print('SHARED CAMERA: opened; retained through pillar and stair', flush=True)

    def close(self):
        camera, self._camera = self._camera, None
        self._config = None
        if camera is not None:
            camera.stop()
            print('SHARED CAMERA: closed', flush=True)


class _CameraLease:
    def __init__(self, owner, config, *, cancelled=None, on_wait=None):
        self.owner, self.config = owner, config
        self._aborted = False
        self._wake = threading.Event()
        self._cancelled = cancelled or (lambda: False)
        self._on_wait = on_wait
        self.retry_start = on_wait is not None

    def start(self):
        while True:
            if self._aborted or self._cancelled():
                raise RuntimeError('camera startup cancelled')
            try:
                if self.retry_start:
                    self._on_wait()
                self.owner._start(self.config)
                if self._aborted or self._cancelled():
                    raise RuntimeError('camera startup cancelled')
                if not self.retry_start:
                    return
                # A retained handle alone is not readiness: require a fresh frame.
                deadline = time.monotonic() + self.config.startup_timeout_ms / 1000.0
                while time.monotonic() < deadline:
                    if self._aborted or self._cancelled():
                        raise RuntimeError('camera startup cancelled')
                    frame = self.get_latest()
                    if frame is not None and 0 <= time.monotonic()-frame.timestamp <= self.config.stale_ms/1000.0:
                        return
                    self._wake.wait(0.05)
                raise TimeoutError('camera produced no fresh frame during startup')
            except Exception as exc:
                failed_camera = self.owner._camera
                self.owner.close()
                if not self.retry_start or self._aborted or self._cancelled():
                    raise
                # A blocked capture.read may outlive stop(). Never open a second
                # handle until that thread has released the old USB stream.
                while getattr(failed_camera, 'is_running', False) is True:
                    if self._aborted or self._cancelled():
                        raise RuntimeError('camera startup cancelled')
                    self._on_wait()
                    self._wake.wait(1.0)
                print(f'CAMERA: startup failed, retrying in 1 s: {exc!r}', flush=True)
                self._on_wait()
                for _ in range(20):
                    if self._aborted or self._cancelled():
                        raise RuntimeError('camera startup cancelled')
                    self._wake.wait(0.05)

    def abort_start(self):
        self._aborted = True
        self._wake.set()
        camera = self.owner._camera
        if camera is not None:
            camera.abort_start()

    def wait_until_ready(self, timeout_ms):
        return self.owner._camera.wait_until_ready(timeout_ms)

    def get_latest(self):
        return self.owner._camera.get_latest()

    def stop(self):
        # Existing task finalizers release a lease, not the physical camera.
        pass
