import sys
import tempfile
import threading
import unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
import cv2

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from rdk_vision.cell_capture import CellCaptureWriter


class CaptureTests(unittest.TestCase):
    def test_background_write_owns_frame_copy(self):
        writer=CellCaptureWriter();entered=threading.Event();release=threading.Event();seen=[]
        frame=np.zeros((12,16,3),np.uint8)
        def write(path,image):
            entered.set();release.wait(2);seen.append(image.copy());return True
        with tempfile.TemporaryDirectory() as folder,patch('rdk_vision.cell_capture.cv2.imwrite',side_effect=write):
            try:
                self.assertTrue(writer.submit(Path(folder)/'round'/'col3-row2.png',frame))
                self.assertTrue(entered.wait(1));frame[:]=255
            finally:
                release.set();writer.jobs.join()
        self.assertTrue(np.all(seen[0]==0))

    def test_full_frame_written(self):
        writer=CellCaptureWriter();frame=np.full((480,640,3),123,np.uint8)
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'round'/'red-col3-row3.png'
            self.assertTrue(writer.submit(path,frame));writer.jobs.join()
            self.assertTrue(np.array_equal(cv2.imread(str(path)),frame))
