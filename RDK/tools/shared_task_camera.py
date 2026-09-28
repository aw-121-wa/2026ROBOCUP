"""Camera ownership across pillar and stationary stair workers.

Only the serial bridge's single task worker uses a lease. The bridge closes
the owner after that worker exits, never while detection is reading a frame.
"""
from rdk_vision.camera import LatestFrameCamera


class SharedTaskCamera:
    def __init__(self, factory=LatestFrameCamera):
        self._factory = factory
        self._camera = None
        self._config = None

    def borrow(self, config):
        if self._config is not None:
            keys = ('device', 'width', 'height', 'fps')
            if any(getattr(config, key) != getattr(self._config, key) for key in keys):
                raise ValueError('pillar/stair camera stream settings differ')
        return _CameraLease(self, config)

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
    def __init__(self, owner, config):
        self.owner, self.config = owner, config

    def start(self):
        self.owner._start(self.config)

    def wait_until_ready(self, timeout_ms):
        return self.owner._camera.wait_until_ready(timeout_ms)

    def get_latest(self):
        return self.owner._camera.get_latest()

    def stop(self):
        # Existing task finalizers release a lease, not the physical camera.
        pass
