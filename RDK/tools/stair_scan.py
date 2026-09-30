"""Continuous level scan; STM32 owns distance, boundaries and stop permission."""
import time
from rdk_vision.config import load_config
from rdk_vision.ball_detector import BallDetector
from hiwonder_action import HiwonderActionBoard


def scan_loop(camera, detector, board, group, gate, on_action_complete,
              before_action, should_finish, *, stale_ms, clock=time.monotonic,
              sleep=time.sleep):
    requested=last_fresh=clock()
    last_id=None
    count=0
    clear_required=0
    while True:
        if gate.is_cancelled(): return 1
        if should_finish(): return 0
        now=clock()
        if now-last_fresh>stale_ms/1000.0:
            raise TimeoutError('STAIR_SCAN camera stalled')
        snapshot=camera.get_latest()
        if (snapshot is None or snapshot.frame_id==last_id or
                not requested<=snapshot.timestamp<=now or
                now-snapshot.timestamp>stale_ms/1000.0):
            sleep(.005); continue
        last_id=snapshot.frame_id;last_fresh=snapshot.timestamp
        result=detector.detect(snapshot.frame,'red')
        if clear_required:
            clear_required=clear_required-1 if not result.valid else 3
            continue
        if not result.valid: continue
        if clock()-snapshot.timestamp>stale_ms/1000.0: continue
        if not before_action(count+1): return 0 if should_finish() else 1
        if should_finish(): return 0
        # STOPPED authorizes the latched detection; braking may take longer than
        # stale_ms. Never dispatch without that explicit stationary handshake.
        if not gate.dispatch_if_allowed(lambda: board.start_group(group)):
            return 1
        board.wait_group_complete(group,timeout_s=30.0)
        count+=1
        if not gate.on_action_complete(count): return 1
        on_action_complete(count)
        deadline=clock()+30
        while not gate.can_execute_action():
            if gate.is_cancelled(): return 1
            if clock()>=deadline: raise TimeoutError('STAIR_SCAN RFID timeout')
            sleep(.01)
        requested=last_fresh=clock()
        last_id=None
        clear_required=3


def run_stair_scan(project_root, level, *, camera_session, rfid_gate,
                   on_action_complete, on_ready, before_action, should_finish):
    if level not in (1,2,3): raise ValueError('STAIR_SCAN level must be 1..3')
    name=('low','high','mid')[level-1]
    config=load_config(project_root/'rdk_vision'/f'stair_{name}.yaml')
    camera=camera_session.borrow(config.camera)
    camera.start()
    if not camera.wait_until_ready(config.camera.startup_timeout_ms):
        raise TimeoutError('STAIR_SCAN camera startup timeout')
    with HiwonderActionBoard('/dev/ttyS1',9600) as board:
        print(f'STAIR_SCAN level={name} group={105+level} continuous',flush=True)
        on_ready()
        return scan_loop(camera,BallDetector(config),board,105+level,rfid_gate,
                         on_action_complete,before_action,should_finish,
                         stale_ms=config.camera.stale_ms)
