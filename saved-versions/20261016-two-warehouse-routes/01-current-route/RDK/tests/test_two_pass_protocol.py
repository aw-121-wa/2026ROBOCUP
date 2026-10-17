import sys, unittest, threading
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'tools')]
from rdk_stm32_bridge import BridgeCore
from rdk_vision.warehouse_digits import NumberConfig, recognize_number, load_number_config

class TwoPassTests(unittest.TestCase):
 def test_stop_ack_follows_worker_exit_and_allows_group(self):
  tx=[]; entered=threading.Event(); exited=threading.Event()
  def number(**kw):
   kw['on_ready']();entered.set();kw['cancel'].wait(1);exited.set();return 2
  core=BridgeCore(tx.append,lambda **kw:0,run_number=number,run_group=lambda n:None)
  core.handle('WAREHOUSE_CHECK 7 0');self.assertTrue(entered.wait(1))
  core.handle('WAREHOUSE_STOP 7')
  self.assertTrue(exited.is_set());self.assertEqual(tx[-1],'WAREHOUSE_STOPPED 7')
  self.assertNotIn('WAREHOUSE_DIGIT 7 2',tx)
  core.handle('GROUP 111');self.assertTrue(core.wait_for_idle(1));self.assertIn('GROUP_DONE 111',tx)
  core.handle('WAREHOUSE_STOP 8');self.assertEqual(tx[-1],'WAREHOUSE_STOPPED 8')
 def test_zero_timeout_recognizes_beyond_old_deadline(self):
  now=[0.]; calls=[0]
  class Frames:
   done=threading.Event()
   def snapshot(self):
    now[0]+=20 if now[0]==0 else .01
    return (int(now[0]*1000),now[0],np.zeros((480,640,3),dtype=np.uint8))
  class Detector:
   def detect(self,frame):
    calls[0]+=1
    return (2,1.)
  config=NumberConfig(device='test',timeout_s=0)
  self.assertEqual(recognize_number(config,frames=Frames(),detector=Detector(),clock=lambda:now[0]),2)
  self.assertGreater(now[0],8);self.assertEqual(calls[0],3)
  self.assertEqual(load_number_config(ROOT/'rdk_vision/warehouse_number.yaml').timeout_s,0)
if __name__=='__main__':unittest.main()
