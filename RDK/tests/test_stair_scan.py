import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'tools')]
from stair_scan import scan_loop, run_stair_scan
from disc_rfid_gate import DiscRfidGate
from rdk_stm32_bridge import BridgeCore

class ScanTests(unittest.TestCase):
    def test_grab_waits_for_rfid_and_same_ball_requires_clear_frames(self):
        now=[0.0];events=[];done=[False]; detections=[True,True,True,False,False,False,True]
        camera=Mock()
        def frame():
            now[0]+=.01
            return SimpleNamespace(frame=None,frame_id=int(now[0]*10000),timestamp=now[0]-.01)
        camera.get_latest.side_effect=frame
        detector=Mock();detector.detect.side_effect=lambda *_:SimpleNamespace(valid=detections.pop(0))
        gate=DiscRfidGate(59);board=Mock()
        board.start_group.side_effect=lambda g:events.append(('grab',g))
        def sleep(dt):
            now[0]+=dt
            if gate.waiting_rfid_index:
                events.append(('rfid',gate.waiting_rfid_index))
                gate.on_rfid_confirmed(gate.waiting_rfid_index,confirmed_at=now[0])
        def complete(n):
            events.append(('done',n))
            if n==2: done[0]=True
        rc=scan_loop(camera,detector,board,107,gate,complete,
            lambda n:events.append(('stopped',n)) or True,lambda:done[0],
            stale_ms=250,clock=lambda:now[0],sleep=sleep)
        self.assertEqual(rc,0)
        self.assertEqual(events,[('stopped',1),('grab',107),('done',1),('rfid',1),
                                 ('stopped',2),('grab',107),('done',2),('rfid',2)])

    def test_stalled_camera_errors_without_action(self):
        now=[1.0];camera=Mock();camera.get_latest.return_value=None;board=Mock()
        with self.assertRaises(TimeoutError):
            scan_loop(camera,Mock(),board,108,DiscRfidGate(59),lambda _:None,
                lambda _:True,lambda:False,stale_ms=250,clock=lambda:now[0],
                sleep=lambda dt:now.__setitem__(0,now[0]+dt))
        board.start_group.assert_not_called()

    def test_each_level_loads_own_profile_and_group_without_prep(self):
        session=Mock()
        with patch('stair_scan.HiwonderActionBoard') as board, patch('stair_scan.scan_loop',return_value=0) as loop:
            for level,name in enumerate(('low','high','mid'),1):
                self.assertEqual(run_stair_scan(ROOT,level,camera_session=session,
                    rfid_gate=DiscRfidGate(59),on_action_complete=Mock(),on_ready=Mock(),
                    before_action=Mock(),should_finish=Mock()),0)
                from rdk_vision.config import load_config
                expected=load_config(ROOT/'rdk_vision'/f'stair_{name}.yaml')
                self.assertEqual(loop.call_args.args[1].config,expected)
                self.assertEqual(loop.call_args.args[3],105+level)
            board.return_value.__enter__.return_value.run_group.assert_not_called()

    def test_no_action_without_stopped_authorization(self):
        now=[0.0]; events=[]
        camera=Mock(); camera.get_latest.side_effect=lambda: SimpleNamespace(
            frame=None,frame_id=int(now[0]*1000),timestamp=now[0])
        detector=Mock();detector.detect.return_value=SimpleNamespace(valid=True)
        board=Mock(); gate=DiscRfidGate(59)
        rc=scan_loop(camera,detector,board,106,gate,lambda _:None,
            lambda n: events.append(n) or False,lambda:False,
            stale_ms=250,clock=lambda:now[0],sleep=lambda dt:now.__setitem__(0,now[0]+dt))
        self.assertEqual(rc,1);self.assertEqual(events,[1]);board.start_group.assert_not_called()

    def test_boundary_finish_cancels_detected_ball_before_grab(self):
        now=[0.0];finished=[False]
        camera=Mock();camera.get_latest.side_effect=lambda:SimpleNamespace(frame=None,frame_id=1,timestamp=now[0])
        detector=Mock();detector.detect.return_value=SimpleNamespace(valid=True)
        board=Mock()
        def stop(n): finished[0]=True; return False
        self.assertEqual(scan_loop(camera,detector,board,106,DiscRfidGate(59),lambda _:None,
            stop,lambda:finished[0],stale_ms=250,clock=lambda:now[0]),0)
        board.start_group.assert_not_called()

    def test_bridge_routes_level_without_pillar_preparation(self):
        calls=[]; lines=[]
        def scan(level,**kw):
            calls.append(level);kw['on_ready']();return 1
        core=BridgeCore(lines.append,lambda **kw:0,run_scan=scan)
        core.handle('STAIR_SCAN 2');self.assertTrue(core.wait_for_idle(1))
        self.assertEqual(calls,[2]);self.assertEqual(lines,['PILLAR_ACK','PILLAR_READY','PILLAR_ERROR'])
