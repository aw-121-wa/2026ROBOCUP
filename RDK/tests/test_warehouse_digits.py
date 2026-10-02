import sys, unittest, time
from pathlib import Path
import cv2
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'tools')]
from rdk_vision.warehouse_digits import DigitDetector, DigitConfirmation, recognize_number, NumberConfig
from rdk_stm32_bridge import BridgeCore

class DigitTests(unittest.TestCase):
 def test_number_camera_is_lazy_and_released_after_request(self):
  from unittest.mock import patch, MagicMock
  from rdk_vision.warehouse_digits import NumberCameraSession
  with patch('rdk_vision.warehouse_digits._NumberFrames') as frames, patch('rdk_vision.warehouse_digits.recognize_number',return_value=2):
   session=NumberCameraSession(NumberConfig(device="fake"))
   frames.assert_not_called()
   self.assertEqual(session.recognize(),2)
   frames.return_value.thread.start.assert_called_once()
   frames.return_value.close.assert_called_once()
   self.assertIsNone(session.frames)
   self.assertEqual(session.recognize(),2)
   self.assertEqual(frames.call_count,2)

 def test_prepared_number_camera_is_reused_without_early_detection(self):
  from unittest.mock import patch
  from rdk_vision.warehouse_digits import NumberCameraSession
  with patch('rdk_vision.warehouse_digits._NumberFrames') as frames, patch('rdk_vision.warehouse_digits.recognize_number',return_value=2) as detect:
   session=NumberCameraSession(NumberConfig(device="fake"))
   session.start(); session.start()
   frames.assert_called_once(); detect.assert_not_called()
   self.assertEqual(session.recognize(),2)
   frames.assert_called_once(); frames.return_value.close.assert_called_once()
 def test_stair_exit_switches_camera_before_group_three(self):
  events=[]
  core=BridgeCore(send_line=lambda line:events.append(line),run_disc=lambda **kw:0,
   close_camera=lambda:events.append('close-ball'),prepare_number=lambda:events.append('open-number'),
   close_number=lambda:events.append('close-number'),run_group=lambda group:events.append(group))
  core._group_main(3)
  self.assertEqual(events[:3],['close-ball','open-number',3])
  events.clear(); core._group_main(0)
  self.assertEqual(events[:3],['close-ball','close-number',0])
 def test_persistent_request_skips_repeated_frame_and_does_not_close(self):
  import threading
  class Frames:
   done=threading.Event()
   def snapshot(self): return (1,time.monotonic(),np.zeros((480,640,3),np.uint8))
  class Detector:
   calls=0
   def detect(self,frame): self.calls+=1; return 0,0
  detector=Detector()
  self.assertEqual(recognize_number(NumberConfig(device="fake",timeout_s=.03),frames=Frames(),detector=detector),0)
  self.assertEqual(detector.calls,1)
 def test_persistent_camera_retries_failed_open(self):
  from rdk_vision.warehouse_digits import _NumberFrames
  calls=[]
  frames=_NumberFrames(NumberConfig(device="fake"),lambda d:calls.append(d),time.monotonic,float("inf"))
  frames.thread.start()
  try:
   deadline=time.monotonic()+1.2
   while len(calls)<2 and time.monotonic()<deadline: time.sleep(.01)
   self.assertGreaterEqual(len(calls),2)
  finally: frames.stop.set();frames.thread.join(1)
  self.assertTrue(frames.done.is_set())
 def test_templates_at_different_sizes(self):
  detector=DigitDetector()
  for d in (1,2,3):
   template=cv2.imread(str(ROOT/'rdk_vision'/'digit_templates'/f'{d}.png'),0)
   for height in (42,80,130):
    width=round(template.shape[1]*height/template.shape[0])
    glyph=cv2.resize(template,(width,height))
    frame=np.full((240,320,3),255,np.uint8)
    frame[50:50+height,100:100+width]=cv2.cvtColor(glyph,cv2.COLOR_GRAY2BGR)
    self.assertEqual(detector.detect(frame)[0],d)
 def test_actual_split_clipped_three(self):
  frame=cv2.imread(str(Path(__file__).with_name('fixtures')/'warehouse_partial_3.png'))
  self.assertEqual(DigitDetector().detect(frame)[0],3)
 def test_actual_clipped_one(self):
  frame=cv2.imread(str(Path(__file__).with_name('fixtures')/'warehouse_partial_1.png'))
  self.assertEqual(DigitDetector().detect(frame)[0],1)
 def test_actual_bottom_clipped_two(self):
  frame=cv2.imread(str(Path(__file__).with_name('fixtures')/'warehouse_partial_2.png'))
  self.assertEqual(DigitDetector().detect(frame)[0],2)
 def test_partial_digits_do_not_require_complete_glyph(self):
  detector=DigitDetector()
  for digit in (1,2,3):
   image=cv2.imread(str(ROOT/'rdk_vision'/'digit_templates'/f'{digit}.png'),0)
   ys,xs=np.where(image<128)
   glyph=image[ys.min():ys.max()+1,xs.min():xs.max()+1]
   glyph=cv2.resize(glyph,(glyph.shape[1]*2,glyph.shape[0]*2))
   visible=glyph[:int(glyph.shape[0]*.85)]
   frame=np.full((visible.shape[0]+30,320,3),255,np.uint8)
   frame[30:,80:80+visible.shape[1]]=cv2.cvtColor(visible,cv2.COLOR_GRAY2BGR)
   self.assertEqual(detector.detect(frame)[0],digit)
 def test_white_label_on_dark_background(self):
  detector=DigitDetector()
  for digit in (1,2,3):
   image=cv2.imread(str(ROOT/'rdk_vision'/'digit_templates'/f'{digit}.png'))
   frame=np.zeros((480,640,3),np.uint8)
   frame[130:340,200:380]=image
   self.assertEqual(detector.detect(frame)[0],digit)
 def test_blank_and_clipped_rejected(self):
  detector=DigitDetector()
  frame=np.full((240,320,3),255,np.uint8)
  self.assertEqual(detector.detect(frame)[0],0)
  frame[:,0:30]=0
  self.assertEqual(detector.detect(frame)[0],0)
 def test_fresh_three_frames_and_exclusions(self):
  gate=DigitConfirmation(excluded=1<<2)
  self.assertEqual(gate.update(2,1,1,1),0)
  self.assertEqual(gate.update(1,2,1,1),0)
  self.assertEqual(gate.update(1,2,1,1),0)
  self.assertEqual(gate.update(1,3,1.01,1.01),0)
  self.assertEqual(gate.update(1,4,0,1),0)
  self.assertEqual(gate.update(1,5,1.02,1.02),0)
  self.assertEqual(gate.update(1,6,1.03,1.03),0)
  self.assertEqual(gate.update(1,7,1.04,1.04),1)
 def test_camera_open_is_bounded_by_request_deadline(self):
  def slow_open(device): time.sleep(.3); return None
  start=time.monotonic()
  self.assertEqual(recognize_number(NumberConfig(device='number-camera',timeout_s=.05),capture_factory=slow_open),0)
  self.assertLess(time.monotonic()-start,.2)
 def test_stream_returns_confirmed_digit_and_releases_camera(self):
  image=cv2.imread(str(ROOT/'rdk_vision'/'digit_templates'/'2.png'))
  frame=np.full((480,640,3),255,np.uint8); frame[100:310,220:400]=image
  import threading
  released=threading.Event()
  class Capture:
   def isOpened(self): return True
   def set(self,*args): pass
   def read(self): time.sleep(.01); return True,frame.copy()
   def release(self): time.sleep(.12); released.set()
  self.assertEqual(recognize_number(NumberConfig(device='number-camera'),capture_factory=lambda _:Capture()),2)
  self.assertTrue(released.is_set())
 def test_missing_device_is_none(self):
  self.assertEqual(recognize_number(NumberConfig(device='/does/not/exist'), excluded=0),0)

class BridgeTests(unittest.TestCase):
 def test_number_request_echo_and_camera_error_fallback(self):
  tx=[]
  def runner(excluded, cancel, on_ready):
   self.assertEqual(excluded,8); on_ready(); return 1
  core=BridgeCore(tx.append,lambda **kw:0,run_number=runner)
  core.handle('WAREHOUSE_CHECK 42 8')
  self.assertTrue(core.wait_for_idle(1))
  self.assertEqual(tx,['WAREHOUSE_READY 42','WAREHOUSE_DIGIT 42 1'])
  def broken(excluded,cancel,on_ready): raise OSError('camera unplugged')
  core=BridgeCore(tx.append,lambda **kw:0,run_number=broken)
  core.handle('WAREHOUSE_CHECK 43 0'); self.assertTrue(core.wait_for_idle(1))
  self.assertEqual(tx[-1],'WAREHOUSE_DIGIT 43 0')
 def test_cancel_number_suppresses_late_digit(self):
  tx=[]
  def runner(excluded,cancel,on_ready):
   cancel.wait(.5); return 2
  core=BridgeCore(tx.append,lambda **kw:0,run_number=runner)
  core.handle('WAREHOUSE_CHECK 7 0'); core.cancel_active()
  self.assertTrue(core.wait_for_idle(1)); self.assertEqual(tx,[])
if __name__=='__main__': unittest.main()
