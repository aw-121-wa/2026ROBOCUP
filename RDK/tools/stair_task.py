"""One stationary stair observation; movement and the two-ball limit belong to STM32."""
import time

from rdk_vision.ball_detector import BallDetector
from rdk_vision.camera import LatestFrameCamera
from rdk_vision.config import load_config
from hiwonder_action import HiwonderActionBoard


def recognize_point(camera, detector, gate, grab, on_action_complete, *,
                    stale_ms, wait_complete=lambda: None,
                    clock=time.monotonic, sleep=time.sleep):
    # Count the observation second only after the first post-request fresh frame.
    requested = clock()
    first = last_fresh = None
    last_id = None
    while True:
        if gate.is_cancelled():
            raise RuntimeError('STAIR cancelled')
        now = clock()
        if first is None and now - requested >= 2.0:
            raise TimeoutError('STAIR no fresh camera frame')
        if last_fresh is not None and now - last_fresh > stale_ms / 1000.0:
            raise TimeoutError('STAIR camera stalled')
        if first is not None and now - first >= 1.0:
            return False
        snapshot = camera.get_latest()
        if (snapshot is not None and snapshot.frame_id != last_id
                and requested <= snapshot.timestamp <= now
                and now - snapshot.timestamp <= stale_ms / 1000.0):
            last_id = snapshot.frame_id
            last_fresh = snapshot.timestamp
            if first is None:
                first = now
            result = detector.detect(snapshot.frame, 'red')
            if result.valid:
                def dispatch():
                    if clock() - snapshot.timestamp > stale_ms / 1000.0:
                        raise TimeoutError('STAIR detection frame expired before action')
                    grab()
                if not gate.dispatch_if_allowed(dispatch, snapshot.timestamp):
                    raise RuntimeError('STAIR action cancelled')
                wait_complete()
                if not gate.on_action_complete(1):
                    raise RuntimeError('STAIR cancelled during action')
                on_action_complete(1)
                deadline = clock() + 30.0
                while not gate.is_complete():
                    if gate.is_cancelled():
                        raise RuntimeError('STAIR cancelled while waiting for RFID')
                    if clock() >= deadline:
                        raise TimeoutError('STAIR RFID confirmation timeout (30s)')
                    sleep(.01)
                return True
        sleep(.005)


def run_stair_point(project_root, point, *, rfid_gate, on_action_complete):
    if not 1 <= point <= 8:
        raise ValueError('STAIR point must be 1..8')

    if point <= 2:
        level = 'low'
        group = 106
    elif point <= 6:
        level = 'high'
        group = 107
    else:
        level = 'mid'
        group = 108

    config_path = project_root / 'rdk_vision' / f'stair_{level}.yaml'
    config = load_config(config_path)
    print(
        f'STAIR CONFIG: point={point} level={level} group={group} '
        f'file={config_path} ROI={config.roi} BALL={config.ball}',
        flush=True,
    )
    camera = LatestFrameCamera(config.camera)
    try:
        camera.start()
        if not camera.wait_until_ready(config.camera.startup_timeout_ms):
            raise TimeoutError('STAIR camera startup timeout')
        with HiwonderActionBoard('/dev/ttyS1', 9600) as board:
            print(f'STAIR point={point} group={group} observation=1s', flush=True)
            return recognize_point(
                camera, BallDetector(config), rfid_gate,
                lambda: board.start_group(group), on_action_complete,
                wait_complete=lambda: board.wait_group_complete(group, timeout_s=30.0),
                stale_ms=config.camera.stale_ms,
            )
    finally:
        camera.stop()
