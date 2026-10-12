import unittest
import time
import threading
from pathlib import Path
import cv2
import numpy as np
from rdk_vision.warehouse_stream import StableLocation, locate, locate_number
from rdk_vision.block_digits import BlockDetector
import yaml
from rdk_vision.warehouse_digits import _normalize
from rdk_stm32_bridge import BridgeCore

class StreamTests(unittest.TestCase):
    def test_fresh_confirmation(self):
        g=StableLocation()
        self.assertEqual(g.update(2,5,1,1,1),(0,0))
        self.assertIsNone(g.update(2,5,1,1,1))
        self.assertEqual(g.update(2,4,2,1.03,1.03),(0,0))
        self.assertEqual(g.update(2,3,3,1.06,1.06),(2,3))
        self.assertIsNone(g.update(2,0,4,1.1,2))
        self.assertEqual(g.update(2,0,5,2.01,2.01),(0,0))
    def test_location_and_ambiguity(self):
        templates=[]
        for digit in (1,2,3):
            mask=np.zeros((100,70),np.uint8)
            cv2.putText(mask,str(digit),(5,80),cv2.FONT_HERSHEY_SIMPLEX,2.3,255,4)
            templates.append((digit,_normalize(mask)))
        frame=np.full((480,640),255,np.uint8)
        cv2.putText(frame,'2',(285,280),cv2.FONT_HERSHEY_SIMPLEX,2.3,0,4)
        digit,offset=locate(frame,templates,(0,0,640,480))
        self.assertEqual(digit,2);self.assertLess(abs(offset),20)
        cv2.putText(frame,'3',(100,280),cv2.FONT_HERSHEY_SIMPLEX,2.3,0,4)
        self.assertEqual(locate(frame,templates,(0,0,640,480)),(0,0))
    def test_recorded_scenes(self):
        root=Path(__file__).resolve().parents[1]/'rdk_vision'
        cfg=yaml.safe_load((root/'block_digits.yaml').read_text(encoding='utf-8'))
        for color in ('red','blue'):
            for row in (1,2,3):
                settings=cfg[color][row];detector=BlockDetector(root,settings)
                roi=(0,settings['roi'][1],640,settings['roi'][3])
                if row==2: roi=(0,max(0,roi[1]-20),640,roi[3]+40)
                for image in (root/settings['templates']).glob('*scene.png'):
                    expected=0 if image.name.startswith('empty') else int(image.name[0])
                    with self.subTest(color=color,row=row,image=image.name):
                        result=locate(cv2.imread(str(image)),detector.templates,
                            roi,
                            settings['min_score'],settings['min_margin'],score_floor=.73 if row==1 else .70)
                        self.assertEqual(result[0],expected)

    def test_number_central_retry(self):
        class Detector:
            def detect(self,image):
                self.last_center=165 if image.shape[1]==320 else None
                return (2,.8) if self.last_center is not None else (0,0)
        self.assertEqual(locate_number(np.full((480,640),255,np.uint8),Detector(),(0,0,640,480)),(2,5))

    def test_number_retry_recorded_and_empty(self):
        from rdk_vision.warehouse_digits import DigitDetector
        detector=DigitDetector(.58,.08)
        root=Path(__file__).resolve().parent/'fixtures'
        for digit in (1,2,3):
            frame=cv2.imread(str(root/f'warehouse_partial_{digit}.png'))
            self.assertEqual(locate_number(frame,detector,(0,0,frame.shape[1],frame.shape[0]))[0],digit)
        self.assertEqual(locate_number(np.full((480,640),255,np.uint8),detector,(0,0,640,480)),(0,0))

    def test_bridge_stop_before_arm(self):
        sent=[];started=threading.Event()
        def runner(row,cancel,emit):
            emit(1,2,0);started.set();cancel.wait(1)
        core=BridgeCore(sent.append,lambda **kw:0,run_stream=runner,run_group=lambda g:None)
        core.handle('WH_SCAN 42 3');self.assertTrue(started.wait(1))
        core.handle('GROUP 113');self.assertNotIn('GROUP_ACK 113',sent)
        core.handle('WH_STOP 41');self.assertFalse(core._cancel.is_set())
        worker=core._worker;core.handle('WH_STOP 42');worker.join(1)
        self.assertIn('WH_DONE 42',sent)
        core.handle('GROUP 113')
        if core._worker: core._worker.join(1)
        self.assertIn('GROUP_DONE 113',sent)

if __name__=='__main__': unittest.main()
