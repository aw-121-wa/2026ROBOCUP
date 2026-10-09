import sys, unittest, tempfile, threading, time
from pathlib import Path
from types import SimpleNamespace
import cv2
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'tools')]
from rdk_vision.block_digits import BlockDetector, confirm_loop, EMPTY, UNKNOWN
from rdk_stm32_bridge import BridgeCore

class BlockTests(unittest.TestCase):
 def setUp(self):
  self.temp=tempfile.TemporaryDirectory();self.root=Path(self.temp.name)
  self.settings=dict(roi=[0,0,160,180],calibrated=True,templates='.',empty_reference='empty.png')
  self.empty=np.full((180,160),180,np.uint8)
  cv2.line(self.empty,(0,5),(159,5),170,2)
  cv2.imwrite(str(self.root/'empty.png'),self.empty)
  self.images={}
  for digit in (1,2,3):
   frame=self.empty.copy();cv2.putText(frame,str(digit),(45,130),cv2.FONT_HERSHEY_SIMPLEX,3,0,6,cv2.LINE_8)
   self.images[digit]=frame
   ys,xs=np.where(frame<128)
   cv2.imwrite(str(self.root/f'{digit}.png'),frame[ys.min():ys.max()+1,xs.min():xs.max()+1])
 def tearDown(self):self.temp.cleanup()
 def test_calibrated_digits_and_empty(self):
  detector=BlockDetector(self.root,self.settings)
  for digit,image in self.images.items():self.assertEqual(detector.detect(image),digit)
  self.assertEqual(detector.detect(self.empty),EMPTY)
  self.assertEqual(detector.detect(np.zeros_like(self.empty)),UNKNOWN)
  obscured=self.empty.copy();cv2.rectangle(obscured,(25,25),(130,155),20,-1)
  self.assertEqual(detector.detect(obscured),UNKNOWN)
 def test_uncalibrated_never_empty(self):
  detector=BlockDetector(self.root,dict(self.settings,calibrated=False))
  self.assertEqual(detector.detect(self.empty),UNKNOWN)
 def test_empty_with_lighting_gradient_and_shift(self):
  shifted=cv2.warpAffine(self.empty,np.float32([[1,0,3],[0,1,3]]),(160,180),borderMode=cv2.BORDER_REPLICATE)
  gradient=np.linspace(-12,12,160)[None,:]
  image=np.uint8(np.clip(shifted.astype(float)+gradient+8,0,255))
  self.assertEqual(BlockDetector(self.root,self.settings).detect(image),EMPTY)
 def test_unreadable_print_and_block_edge_are_not_empty(self):
  detector=BlockDetector(self.root,self.settings)
  for x in (0,60,157):
   image=self.empty.copy();cv2.rectangle(image,(x,35),(min(159,x+2),145),30,-1)
   self.assertEqual(detector.detect(image),UNKNOWN)
  image=self.empty.copy();cv2.putText(image,'?',(45,130),cv2.FONT_HERSHEY_SIMPLEX,3,0,6)
  self.assertEqual(detector.detect(image),UNKNOWN)
 def test_real_empty_column_and_all_digit_samples(self):
  import yaml
  root=ROOT/'rdk_vision'
  settings=yaml.safe_load((root/'block_digits.yaml').read_text())['red']
  for row in (1,2,3):
   detector=BlockDetector(root,settings[row]);folder=root/settings[row]['templates']
   for label,value in [('1',1),('2',2),('3',3),('empty',EMPTY)]:
    self.assertEqual(detector.detect(cv2.imread(str(folder/(label+'-scene.png')))),value)
  image=cv2.imread(str(root/'block_samples/shared/row2/empty-column3-scene.png'))
  self.assertEqual(BlockDetector(root,settings[2]).detect(image),EMPTY)
 def test_missing_sample_and_bad_roi(self):
  (self.root/'2.png').unlink()
  with self.assertRaises(ValueError):BlockDetector(self.root,self.settings)
  with self.assertRaises(ValueError):BlockDetector(self.root,dict(self.settings,roi=[-1,0,160,180]))
 def test_frame_confirmation_empty_slower_than_digit(self):
  for result,minimum in ((2,.15),(EMPTY,.55)):
   now=[0.0];serial=[0]
   def get():
    now[0]+=.05;serial[0]+=1
    return SimpleNamespace(frame_id=serial[0],timestamp=now[0],frame=None)
   camera=SimpleNamespace(get_latest=get);detector=SimpleNamespace(detect=lambda frame:result)
   self.assertEqual(confirm_loop(camera,detector,threading.Event(),clock=lambda:now[0],sleep=lambda t:None),result)
   self.assertGreaterEqual(now[0]+1e-6,minimum)
 def test_repeated_frame_cannot_confirm(self):
  now=[0.0]
  def get():
   now[0]+=.05
   return SimpleNamespace(frame_id=1,timestamp=.05,frame=None)
  result=confirm_loop(SimpleNamespace(get_latest=get),SimpleNamespace(detect=lambda _:1),threading.Event(),
                     timeout_s=.5,clock=lambda:now[0],sleep=lambda _:None)
  self.assertEqual(result,UNKNOWN)
 def test_stale_frames_raise_not_empty(self):
  now=[1.0]
  def get():
   now[0]+=.1
   return SimpleNamespace(frame_id=round(now[0]*10),timestamp=0,frame=None)
  with self.assertRaises(TimeoutError):
   confirm_loop(SimpleNamespace(get_latest=get),SimpleNamespace(detect=lambda _:EMPTY),threading.Event(),
                timeout_s=.5,clock=lambda:now[0],sleep=lambda _:None)
 def test_two_digits_ambiguous(self):
  image=self.empty.copy()
  cv2.putText(image,'12',(10,120),cv2.FONT_HERSHEY_SIMPLEX,2,0,4,cv2.LINE_8)
  self.assertEqual(BlockDetector(self.root,self.settings).detect(image),UNKNOWN)

class CalibrationTests(unittest.TestCase):
 def test_import_requires_all_samples_and_validates_before_enabling(self):
  from calibrate_block_digits import main
  import yaml
  with tempfile.TemporaryDirectory() as temporary:
   root=Path(temporary);config=root/'block_digits.yaml'
   settings=dict(roi=[100,100,160,180],calibrated=False,templates='samples',empty_reference='samples/empty.png')
   config.write_text(yaml.safe_dump({'red':{3:settings}}))
   base=['--config',str(config),'--side','red','--row','3']
   for label in ('empty','1','2','3'):
    image=np.full((480,640,3),180,np.uint8)
    if label!='empty':cv2.putText(image,label,(145,230),cv2.FONT_HERSHEY_SIMPLEX,3,(0,0,0),6,cv2.LINE_8)
    path=root/(label+'-input.png');cv2.imwrite(str(path),image)
    args=base+['--input',str(path),'--label',label]
    if label!='empty':
     ys,xs=np.where(cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)<128)
     args+=['--glyph',str(xs.min()),str(ys.min()),str(xs.max()-xs.min()+1),str(ys.max()-ys.min()+1)]
    main(args)
    self.assertFalse(yaml.safe_load(config.read_text())['red'][3]['calibrated'])
   main(base+['--enable'])
   self.assertTrue(yaml.safe_load(config.read_text())['red'][3]['calibrated'])

class BlockBridgeTests(unittest.TestCase):
 def wait(self,core):
  deadline=time.monotonic()+2
  while core.disc_active and time.monotonic()<deadline:time.sleep(.005)
  self.assertFalse(core.disc_active)
 def test_results_and_camera_switch(self):
  for result in range(5):
   tx=[];events=[]
   def run(row,**kwargs):events.append(row);return result
   core=BridgeCore(tx.append,lambda **kw:0,run_block=run,close_number=lambda:events.append('close-number'))
   core.handle('BLOCK_CHECK 123 3');self.wait(core)
   self.assertEqual(events,['close-number',3]);self.assertEqual(tx,['BLOCK_RESULT 123 '+str(result)])
 def test_error_not_empty(self):
  tx=[]
  def run(*args,**kwargs):raise OSError('camera unavailable')
  core=BridgeCore(tx.append,lambda **kw:0,run_block=run)
  core.handle('BLOCK_CHECK 1 2');self.wait(core);self.assertEqual(tx,['BLOCK_RESULT 1 5'])
 def test_cancel_and_busy(self):
  tx=[];entered=threading.Event();release=threading.Event();groups=[]
  def run(*args,**kwargs):entered.set();release.wait(1);return EMPTY
  core=BridgeCore(tx.append,lambda **kw:0,run_block=run,run_group=groups.append)
  core.handle('BLOCK_CHECK 1 1');self.assertTrue(entered.wait(1))
  core.handle('GROUP 113');core.handle('BLOCK_CHECK 2 2');core.handle('DISC_CANCEL');release.set();self.wait(core)
  self.assertEqual(tx,[]);self.assertEqual(groups,[])
 def test_new_action_groups(self):
  for group in range(112,121):
   tx=[];groups=[]
   core=BridgeCore(tx.append,lambda **kw:0,run_group=groups.append)
   core.handle(f'GROUP {group}');self.wait(core)
   self.assertEqual(groups,[group]);self.assertEqual(tx,[f'GROUP_ACK {group}',f'GROUP_DONE {group}'])

if __name__=='__main__':unittest.main()
