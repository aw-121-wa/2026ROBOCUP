"""Fresh, token-scoped digit locations for continuous warehouse travel.

Only horizontal alignment is controllable by warehouse travel. Arm row poses
supply the height; center tolerance is enforced again by STM32 after braking.
"""
from pathlib import Path
import time
import cv2
import numpy as np
import yaml
from .warehouse_digits import _normalize
from .block_digits import BlockDetector
from .config import load_config


def locate(frame, templates, roi, min_score=.72, min_margin=.12, min_height=20):
    x,y,w,h=roi
    if frame is None or x+w>frame.shape[1] or y+h>frame.shape[0]: return 0,0
    crop=frame[y:y+h,x:x+w]
    gray=cv2.cvtColor(crop,cv2.COLOR_BGR2GRAY) if crop.ndim==3 else crop
    if float(gray.mean())<8 or int(gray.max())-int(gray.min())<20: return 0,0
    regions=[(0,0,w,h)]
    # Local threshold windows retain glyphs beside bright shelf rails.
    ww,hh=min(110,w),min(100,h)
    regions.extend((left,top,ww,hh) for top in sorted(set(range(0,h-hh+1,8))|{h-hh})
                   for left in sorted(set(range(0,w-ww+1,16))|{w-ww}))
    found=[]
    for left,top,ww,hh in regions:
        part=gray[top:top+hh,left:left+ww]
        _,mask=cv2.threshold(part,0,255,cv2.THRESH_BINARY_INV|cv2.THRESH_OTSU)
        contours,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            a,b,c,d=cv2.boundingRect(contour)
            if a<2 or b<2 or a+c>ww-2 or b+d>hh-2 or d<min_height or not .12<c/d<1.3: continue
            glyph=_normalize(mask[b:b+d,a:a+c])
            if glyph is None: continue
            scores={i:0.0 for i in (1,2,3)}
            for digit,template in templates:
                score=float(np.count_nonzero(glyph&template))/max(1,np.count_nonzero(glyph|template))
                scores[digit]=max(scores[digit],score)
            ranked=sorted(((score,digit) for digit,score in scores.items()),reverse=True)
            if ranked[0][0]<max(.73,min_score) or ranked[0][0]-ranked[1][0]<min_margin: continue
            center=x+left+a+c/2
            found.append((ranked[0][1],center,ranked[0][0]))
    if not found: return 0,0
    found.sort(key=lambda item:item[2],reverse=True)
    digit,center,_=found[0]
    # Multiple visible glyphs are not allowed to masquerade as one centered glyph.
    if any(other!=digit or abs(cx-center)>12 for other,cx,_ in found[1:]): return 0,0
    return digit,round(center-frame.shape[1]/2)


class StableLocation:
    def __init__(self):
        self.last_id=None;self.last_time=None;self.digit=0;self.count=0
    def update(self, digit, offset, frame_id, stamp, now):
        if frame_id==self.last_id: return None
        self.last_id=frame_id
        if not 0<=now-stamp<=.20 or (self.last_time is not None and stamp<=self.last_time):
            self.count=0;return None
        if self.last_time is None or stamp-self.last_time>.20: self.count=0
        self.last_time=stamp
        self.count=self.count+1 if digit and digit==self.digit else 1
        self.digit=digit
        return (digit,offset) if digit and self.count>=3 else (0,0)


def run_stream(project_root,row,camera_session,number_session,*,color,cancel,emit):
    root=Path(project_root)/'rdk_vision'
    camera=None
    if row:
        number_session.close()
        settings=yaml.safe_load((root/'block_digits.yaml').read_text(encoding='utf-8'))[color][row]
        detector=BlockDetector(root,settings)
        if not detector.ready: raise ValueError('uncalibrated block camera')
        config=load_config(root/'stair_low.yaml')
        camera=camera_session.borrow(config.camera,cancelled=cancel.is_set)
        camera.start()
        roi=(0,settings['roi'][1],640,settings['roi'][3])
        templates=detector.templates
        score=settings.get('min_score',.72);margin=settings.get('min_margin',.12)
    else:
        camera_session.close();number_session.start()
        roi=number_session.config.roi
        templates=[(digit,mask) for digit,mask,_ in number_session.detector.templates]
        score=number_session.config.min_score;margin=number_session.config.min_margin
    gate=StableLocation();requested=time.monotonic();sequence=0
    while not cancel.wait(.005):
        if row:
            snap=camera.get_latest()
            snapshot=None if snap is None else (snap.frame_id,snap.timestamp,snap.frame)
        else:
            snapshot=number_session.frames.snapshot() if number_session.frames else None
        if snapshot is None:
            if time.monotonic()-requested>2: raise TimeoutError('warehouse camera has no frames')
            continue
        frame_id,stamp,frame=snapshot
        if time.monotonic()-stamp>2: raise TimeoutError('warehouse camera stalled')
        if stamp<requested or frame_id==gate.last_id: continue
        if row:
            digit,offset=locate(frame,templates,roi,score,margin)
        else:
            x,y,w,h=roi
            digit,_=number_session.detector.detect(frame[y:y+h,x:x+w])
            center=number_session.detector.last_center
            offset=round(x+center-frame.shape[1]/2) if center is not None else 0
        result=gate.update(digit,offset,frame_id,stamp,time.monotonic())
        if result is not None:
            sequence+=1;emit(sequence,*result)


