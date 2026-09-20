import sys
import threading
import time
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from rdk_stm32_bridge import BridgeCore
from disc_rfid_gate import DiscRfidGate
from vision_servo_direct_test import run_disc_task
from test_disc_task_loop import FakeCamera, FakeDetector, FakeTrigger, FakeBoard, snapshot, args, config

class PathActions(unittest.TestCase):
    def test_shared_vision_loop_obeys_stop_permission_before_g104(self):
        for permission in (False, True):
            board=FakeBoard(); camera=FakeCamera([snapshot(i) for i in range(4)])
            settings=args(1); settings.prep_group=103; settings.trigger_group=104
            gate=DiscRfidGate(1); order=[]
            def before(index):
                self.assertEqual(board.started, [])
                order.append(('stop', index))
                return permission
            def complete(index):
                order.append(('rfid',index))
                self.assertTrue(gate.on_rfid_confirmed(index))
            rc=run_disc_task(settings, config=config(), detector=FakeDetector(),
                trigger=FakeTrigger(['TRIGGER_VALID']), camera=camera, board=board,
                rfid_gate=gate, on_ready=lambda: order.append('ready'), before_action=before,
                on_action_complete=complete, clock=lambda: float(camera.index-1),
                sleep_fn=lambda seconds: None, max_idle_iterations=3)
            self.assertEqual(board.started, [(104,1)] if permission else [])
            self.assertEqual(order, ['ready',('stop',1),('rfid',1)] if permission else ['ready',('stop',1)])
            self.assertEqual(rc, 0 if permission else 1)

    def test_pillar_waits_for_stopped_then_rfid_before_resuming(self):
        tx, action, waiting = [], threading.Event(), threading.Event()
        def pillar(**kw):
            kw['on_ready']()
            waiting.set()
            if not kw['before_action'](1): return 1
            gate=kw['rfid_gate']
            self.assertTrue(gate.on_action_complete(1))
            kw['on_action_complete'](1); action.set()
            deadline=time.monotonic()+1
            while not kw['should_finish']() and not gate.is_cancelled():
                if time.monotonic()>deadline: return 1
                time.sleep(.001)
            return 0
        core=BridgeCore(tx.append, lambda **kw: 1, run_pillar=pillar)
        core.handle('PILLAR_START'); self.assertTrue(waiting.wait(.5))
        self.assertFalse(action.wait(.02))
        core.handle('PILLAR_STOPPED 2'); self.assertFalse(action.wait(.02))
        core.handle('PILLAR_STOPPED 1'); self.assertTrue(action.wait(.5))
        self.assertFalse(core.gate.can_execute_action())
        core.handle('PILLAR_RFID_OK 2'); self.assertNotIn('PILLAR_RESUME 1', tx)
        core.handle('PILLAR_RFID_OK 1'); self.assertIn('PILLAR_RESUME 1', tx)
        core.handle('PILLAR_END'); self.assertTrue(core.wait_for_idle(1))
        self.assertEqual(tx, ['PILLAR_ACK','PILLAR_READY','PILLAR_BALL 1',
                              'PILLAR_ACTION_DONE 1','PILLAR_RESUME 1','PILLAR_DONE'])

    def test_cancel_while_waiting_for_stop_never_executes_action(self):
        tx, entered, actions=[], threading.Event(), []
        def pillar(**kw):
            kw['on_ready'](); entered.set()
            if kw['before_action'](1): actions.append(104)
            return 1
        core=BridgeCore(tx.append, lambda **kw: 1, run_pillar=pillar)
        core.handle('PILLAR_START'); self.assertTrue(entered.wait(.5))
        core.handle('PILLAR_CANCEL'); core.handle('PILLAR_STOPPED 1')
        self.assertTrue(core.wait_for_idle(1)); self.assertEqual(actions, [])
        self.assertEqual(tx[-1], 'PILLAR_ERROR')

    def test_group_waits_for_completion_and_excludes_other_workers(self):
        tx, entered, release, groups = [], threading.Event(), threading.Event(), []
        def group(number):
            groups.append(number); entered.set(); release.wait(1)
        core = BridgeCore(tx.append, lambda **kw: 1, run_group=group)
        core.handle('GROUP 0')
        self.assertTrue(entered.wait(.5))
        self.assertEqual(tx, ['GROUP_ACK 0'])
        core.handle('GROUP 100'); core.handle('DISC_START'); core.handle('PING')
        self.assertEqual(groups, [0]); self.assertIn('PONG', tx)
        release.set(); self.assertTrue(core.wait_for_idle(1))
        self.assertEqual(tx[-1], 'GROUP_DONE 0')

    def test_group_failure_does_not_report_done(self):
        tx=[]
        def group(number): raise TimeoutError('servo missing')
        core=BridgeCore(tx.append, lambda **kw: 1, run_group=group)
        core.handle('GROUP 0'); self.assertTrue(core.wait_for_idle(1))
        self.assertEqual(tx, ['GROUP_ACK 0', 'GROUP_ERROR 0'])

if __name__ == '__main__': unittest.main()
