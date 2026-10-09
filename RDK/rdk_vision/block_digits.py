"""Row-specific block digits and calibrated empty-cell recognition.

Wire results: UNKNOWN=0, digits=1..3, EMPTY=4. Exceptions are bridge ERROR=5.
No uncalibrated, stale, dark or unreadable frame can report EMPTY.
"""
from pathlib import Path
import time
import cv2
import numpy as np
import yaml
from .config import load_config
from .warehouse_digits import _normalize

UNKNOWN, EMPTY = 0, 4

class BlockDetector:
    def __init__(self, root, settings):
        self.settings = settings
        self.roi = tuple(settings['roi'])
        if len(self.roi)!=4 or any(type(v) is not int for v in self.roi):
            raise ValueError('block ROI must contain four integers')
        x,y,w,h=self.roi
        if x<0 or y<0 or w<=0 or h<=0 or x+w>640 or y+h>480:
            raise ValueError('invalid block ROI')
        self.templates = []
        self.empty = None
        self.ready = False
        if not settings.get('calibrated', False): return
        root = Path(root)
        # Samples must come from this camera/pose, not bottom-camera templates.
        for digit in (1, 2, 3):
            path = root / settings['templates'] / f'{digit}.png'
            image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
            if image is None: raise ValueError(f'missing block glyph sample: {path}')
            glyph = _normalize((image < 128).astype(np.uint8))
            if glyph is None: raise ValueError(f'empty glyph sample: {path}')
            self.templates.append((digit, glyph))
        image = cv2.imread(str(root / settings['empty_reference']), cv2.IMREAD_GRAYSCALE)
        if image is None: raise ValueError('missing empty-cell reference')
        x,y,w,h = self.roi
        if image.shape != (h,w): raise ValueError('empty reference must exactly match ROI')
        self.empty = image.astype(np.float32)
        self.ready = True

    def detect(self, frame):
        if not self.ready: return UNKNOWN
        x,y,w,h = self.roi
        if frame is None or y+h>frame.shape[0] or x+w>frame.shape[1]: return UNKNOWN
        crop = frame[y:y+h,x:x+w]
        gray = cv2.cvtColor(crop,cv2.COLOR_BGR2GRAY) if crop.ndim==3 else crop
        if not 25<float(np.mean(gray))<245: return UNKNOWN
        _,mask = cv2.threshold(gray,0,255,cv2.THRESH_BINARY_INV|cv2.THRESH_OTSU)
        contours,_ = cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        found = []
        for contour in contours:
            a,b,c,d = cv2.boundingRect(contour)
            if (a<2 or b<2 or a+c>w-2 or b+d>h-2 or
                    d<self.settings.get('min_digit_height',20) or not .12<c/d<1.3): continue
            glyph = _normalize(mask[b:b+d,a:a+c])
            if glyph is None: continue
            scores = sorted(((float(np.count_nonzero(glyph & template))/max(1,np.count_nonzero(glyph | template)),digit)
                             for digit,template in self.templates), reverse=True)
            if scores[0][0]>=self.settings.get('min_score',.75) and scores[0][0]-scores[1][0]>=.15:
                found.append(scores[0][1])
        if len(found)==1: return found[0]
        if found: return UNKNOWN
        # Only a close positive match to a verified empty cell can mean EMPTY.
        current = gray.astype(np.float32)
        delta = current-self.empty
        delta -= np.median(delta)  # tolerate small uniform illumination drift
        mean_error = float(np.mean(np.abs(delta)))
        changed = float(np.mean(np.abs(delta)>25))
        if mean_error<=self.settings.get('empty_mean_error',4.0) and changed<=self.settings.get('empty_changed_fraction',.01):
            return EMPTY
        return UNKNOWN


def confirm_loop(camera, detector, cancel, *, timeout_s=3.0, clock=time.monotonic, sleep=time.sleep):
    requested = clock(); last_id = None; last_time = requested
    candidate = UNKNOWN; count = 0; first = requested; saw_fresh = False
    while not cancel.is_set() and clock()-requested<timeout_s:
        snapshot = camera.get_latest(); now = clock()
        if (snapshot is None or snapshot.frame_id==last_id or
                not requested<=snapshot.timestamp<=now or now-snapshot.timestamp>.25 or
                (saw_fresh and snapshot.timestamp<=last_time)):
            sleep(.005); continue
        if snapshot.timestamp-last_time>.25: count=0
        last_id=snapshot.frame_id;last_time=snapshot.timestamp;saw_fresh=True
        result=detector.detect(snapshot.frame)
        if clock()-snapshot.timestamp>.25:
            candidate=UNKNOWN;count=0;continue
        if result==UNKNOWN: candidate=UNKNOWN;count=0;continue
        if candidate!=result: candidate=result;count=0;first=now
        count+=1
        if count>=(10 if result==EMPTY else 3) and (result!=EMPTY or now-first>=.5): return result
    if cancel.is_set(): return UNKNOWN
    if not saw_fresh: raise TimeoutError('block camera produced no fresh frame')
    return UNKNOWN


def run_block_check(project_root, row, camera_session, *, color='red', cancel):
    if row not in (1,2,3) or color not in ('red','blue'): raise ValueError('invalid block row/side')
    root=Path(project_root)/'rdk_vision'
    settings=yaml.safe_load((root/'block_digits.yaml').read_text(encoding='utf-8'))
    entry=settings[color][row]
    detector=BlockDetector(root,entry)
    config=load_config(root/'stair_low.yaml')  # Same ball USB device and capture settings only.
    camera=camera_session.borrow(config.camera,cancelled=cancel.is_set)
    camera.start()
    if not camera.wait_until_ready(config.camera.startup_timeout_ms): raise TimeoutError('block camera startup failed')
    return confirm_loop(camera,detector,cancel)
