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
        if image.shape == (480,640): image=image[y:y+h,x:x+w]
        if image.shape != (h,w): raise ValueError('empty reference must exactly match ROI')
        self.empty = image.astype(np.float32)
        self.empty_samples = [self.empty]
        for name in settings.get('empty_references', []):
            reference = cv2.imread(str(root / name), cv2.IMREAD_GRAYSCALE)
            if reference is not None and reference.shape == (480,640):
                reference=reference[y:y+h,x:x+w]
            if reference is None or reference.shape != (h, w):
                raise ValueError(f'invalid additional empty reference: {name}')
            self.empty_samples.append(reference.astype(np.float32))
        self.ready = True

    def _matches_empty(self, gray):
        """Positive empty match, with bounded translation and smooth lighting drift.

        Never infer EMPTY merely from failure to recognize a digit. Compare the
        whole ROI (including its edges); do not crop away an entering block.
        """
        current = gray.astype(np.float32)
        h, w = gray.shape
        yy, xx = np.mgrid[-1:1:complex(h), -1:1:complex(w)]
        basis = np.stack([np.ones_like(xx), xx, yy], axis=-1).reshape(-1, 3)
        inverse = np.linalg.pinv(basis)
        for reference in self.empty_samples:
            for dy in (0, -3, 3, -6, 6):
                for dx in (0, -3, 3, -6, 6):
                    aligned = cv2.warpAffine(reference, np.float32([[1, 0, dx], [0, 1, dy]]),
                                             (w, h), borderMode=cv2.BORDER_REPLICATE)
                    delta = current - aligned
                    lighting = inverse @ delta.ravel()
                    if abs(lighting[0]) > 35 or np.max(np.abs(lighting[1:])) > 20:
                        continue
                    residual = delta - (basis @ lighting).reshape(h, w)
                    if np.mean(np.abs(residual)) > self.settings.get('empty_mean_error', 4.0):
                        continue
                    if np.mean(np.abs(residual) > 25) > self.settings.get('empty_changed_fraction', .01):
                        continue
                    # A new vertical contour can be a block side or unreadable
                    # print, even when it covers few pixels in the empty ROI.
                    edges = cv2.Canny(gray, 25, 60)
                    known = cv2.Canny(np.uint8(np.clip(aligned, 0, 255)), 25, 60)
                    unexplained = edges & ~cv2.dilate(known, np.ones((5, 5), np.uint8))
                    contours, _ = cv2.findContours(unexplained, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
                    if any(cv2.boundingRect(c)[3] >= 15 for c in contours):
                        continue
                    return True
        return False

    def _has_block_evidence(self, gray):
        """Detect a card/glyph silhouette even when its digit is unreadable."""
        window = self.settings.get('digit_window')
        if not window:
            return True
        ww, hh = window
        h, w = gray.shape
        for top in sorted(set(range(0, h - hh + 1, 8)) | {max(0, h - hh)}):
            for left in sorted(set(range(0, w - ww + 1, 8)) | {max(0, w - ww)}):
                crop = gray[top:top + hh, left:left + ww]
                mask = cv2.adaptiveThreshold(
                    crop, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                    cv2.THRESH_BINARY_INV, 31, 7)
                contours, _ = cv2.findContours(
                    mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                for contour in contours:
                    a, b, c, d = cv2.boundingRect(contour)
                    if (cv2.contourArea(contour) >= 35 and d >= self.settings.get(
                            'block_evidence_min_height', 30) and c >= 5 and
                            .10 < c / d < 1.6):
                        return True
        return False

    def _digits(self, gray):
        h, w = gray.shape
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
            if (scores[0][0]>=self.settings.get('min_score',.75) and
                    scores[0][0]-scores[1][0]>=self.settings.get('min_margin',.15)):
                found.append(scores[0][1])
        return found

    def detect(self, frame):
        if not self.ready: return UNKNOWN
        x,y,w,h = self.roi
        if frame is None or y+h>frame.shape[0] or x+w>frame.shape[1]: return UNKNOWN
        crop = frame[y:y+h,x:x+w]
        gray = cv2.cvtColor(crop,cv2.COLOR_BGR2GRAY) if crop.ndim==3 else crop
        if not 25<float(np.mean(gray))<245: return UNKNOWN
        found = self._digits(gray)
        window = self.settings.get('digit_window')
        if window:
            ww, hh = window
            found = []
            for top in sorted(set(range(0, h-hh+1, 8)) | set(range(10, h-hh+1, 8)) | {h-hh}):
                for left in sorted(set(range(0, w-ww+1, 8)) | set(range(10, w-ww+1, 8)) | {w-ww}):
                    found.extend(self._digits(gray[top:top+hh,left:left+ww]))
            found = sorted(set(found))
        if len(found)==1: return found[0]
        if found: return UNKNOWN
        # A blank cell has no tall central glyph.  If there is no card-shaped
        # evidence, accept EMPTY even when the shelf background moved.
        if not self._has_block_evidence(gray): return EMPTY
        if self._matches_empty(gray):
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
    return confirm_loop(camera,detector,cancel,timeout_s=1.5)
