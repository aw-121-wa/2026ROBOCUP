"""Single visible warehouse digit; no pixel-to-distance or yaw estimation."""
from dataclasses import dataclass
from pathlib import Path
import time
import threading
import cv2
import numpy as np
import yaml

@dataclass(frozen=True)
class NumberConfig:
    device: str = ''  # Empty intentionally selects first-column default fallback.
    width: int = 640
    height: int = 480
    fps: int = 30
    roi: tuple = (0, 0, 640, 480)
    timeout_s: float = 8.0
    min_score: float = 0.62
    min_margin: float = 0.10
    confirm_frames: int = 3
    stale_s: float = 0.25

def load_number_config(path):
    data = yaml.safe_load(Path(path).read_text(encoding='utf-8')) or {}
    config = NumberConfig(**data)
    x,y,w,h = config.roi
    if (config.width<=0 or config.height<=0 or config.fps<=0 or
        x<0 or y<0 or w<=0 or h<=0 or x+w>config.width or y+h>config.height or
        not 0<config.timeout_s<=10 or not 0<config.min_score<=1 or
        not 0<config.min_margin<1 or config.confirm_frames<3 or not 0<config.stale_s<=0.5):
        raise ValueError('invalid warehouse number camera configuration')
    return config

def _normalize(mask):
    ys,xs=np.where(mask>0)
    if not len(xs): return None
    crop=mask[ys.min():ys.max()+1,xs.min():xs.max()+1]
    return cv2.resize(crop,(64,96),interpolation=cv2.INTER_NEAREST)>0

class DigitDetector:
    def __init__(self,min_score=0.62,min_margin=0.10):
        self.min_score,self.min_margin=min_score,min_margin
        self.templates=[]
        self.masks={}
        self.variant_cache={}
        for digit in (1,2,3):
            path=Path(__file__).with_name('digit_templates')/f'{digit}.png'
            image=cv2.imread(str(path),cv2.IMREAD_GRAYSCALE)
            if image is None: raise RuntimeError(f'missing digit template {path}')
            mask=(image<128).astype(np.uint8)*255
            ys,xs=np.where(mask>0)
            self.masks[digit]=mask[ys.min():ys.max()+1,xs.min():xs.max()+1]
            self.templates.append((digit,_normalize(mask),(xs.max()-xs.min()+1)/(ys.max()-ys.min()+1)))

    def _variants(self,top,bottom):
        key=(top,bottom)
        if key not in self.variant_cache:
            variants=[]
            for digit,mask in self.masks.items():
                for cut_top in (range(15) if top else (0,)):
                    for cut_bottom in (range(15) if bottom else (0,)):
                        if cut_top+cut_bottom>14: continue  # Retain at least 65% height.
                        first=round(mask.shape[0]*cut_top*.025)
                        last=round(mask.shape[0]*(1-cut_bottom*.025))
                        part=mask[first:last]
                        ys,xs=np.where(part>0)
                        if not len(xs): continue
                        aspect=(xs.max()-xs.min()+1)/(ys.max()-ys.min()+1)
                        variants.append((digit,_normalize(part),aspect))
            self.variant_cache[key]=variants
        return self.variant_cache[key]

    def detect(self,frame):
        gray=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY) if frame.ndim==3 else frame
        if gray.size==0: return 0,0.0
        _,mask=cv2.threshold(gray,0,255,cv2.THRESH_BINARY_INV|cv2.THRESH_OTSU)
        contours,_=cv2.findContours(mask,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
        boxes=[]
        for contour in contours:
            x,y,w,h=cv2.boundingRect(contour)
            if x<=1 or x+w>=gray.shape[1]-1 or w<5 or h<20 or cv2.contourArea(contour)<25: continue
            boxes.append((x,y,w,h))
        regions=[(box,frozenset((i,))) for i,box in enumerate(boxes)]
        # Clipping may disconnect two strokes which join just outside the frame.
        # Merge only nearby fragments touching the SAME top/bottom image edge.
        for i,(x,y,w,h) in enumerate(boxes):
            for j in range(i+1,len(boxes)):
                a,b,c,d=boxes[j]
                same_edge=(y<=1 and b<=1) or (y+h>=gray.shape[0]-1 and b+d>=gray.shape[0]-1)
                gap=max(0,max(x,a)-min(x+w,a+c))
                if not same_edge or gap>max(h,d)*.25: continue
                left,top=min(x,a),min(y,b)
                right,bottom=max(x+w,a+c),max(y+h,b+d)
                regions.append(((left,top,right-left,bottom-top),frozenset((i,j))))
        candidates=[]
        for (x,y,w,h),sources in regions:
            top,bottom=y<=1,y+h>=gray.shape[0]-1
            partial=top or bottom
            if not 0.15<w/h<1.2: continue
            glyph=_normalize(mask[y:y+h,x:x+w])
            best={1:0.0,2:0.0,3:0.0}
            for digit,template,aspect in (self._variants(top,bottom) if partial else self.templates):
                score=float(np.count_nonzero(glyph & template))/max(1,np.count_nonzero(glyph | template))
                if not 0.60<(w/h)/aspect<1.65: score=0.0
                best[digit]=max(best[digit],score)
            scores=sorted(((score,digit) for digit,score in best.items()),reverse=True)
            minimum=max(.72,self.min_score) if partial else self.min_score
            margin=max(.16,self.min_margin) if partial else self.min_margin
            if scores[0][0]>=minimum and scores[0][0]-scores[1][0]>=margin:
                candidates.append((scores[0][1],scores[0][0],sources))
        selected=[]
        for digit,score,sources in sorted(candidates,key=lambda item:item[1],reverse=True):
            overlaps=[other for other in selected if sources & other[2]]
            if any(other[0]!=digit for other in overlaps): return 0,0.0
            if not overlaps: selected.append((digit,score,sources))
        return selected[0][:2] if len(selected)==1 else (0,0.0)

class DigitConfirmation:
    def __init__(self,excluded=0,frames=3,stale_s=.25):
        self.excluded,self.frames,self.stale_s=excluded,frames,stale_s
        self.last_frame=None; self.candidate=0; self.count=0; self.last_timestamp=None
    def update(self,digit,frame_id,timestamp,now):
        if frame_id==self.last_frame: return 0
        self.last_frame=frame_id
        if (digit not in (1,2,3) or self.excluded & (1<<digit) or
            not 0<=now-timestamp<=self.stale_s or
            (self.last_timestamp is not None and timestamp<=self.last_timestamp)):
            self.candidate=0; self.count=0; return 0
        if self.last_timestamp is not None and timestamp-self.last_timestamp>self.stale_s:
            self.candidate=0; self.count=0
        self.last_timestamp=timestamp
        self.count=self.count+1 if digit==self.candidate else 1
        self.candidate=digit
        return digit if self.count>=self.frames else 0

CAPTURE_POLL_S = 0.005
CAMERA_RECONNECT_S = 0.5
CAMERA_CLOSE_S = 0.4


class _NumberFrames:
    """The capture thread owns release; a stuck driver cannot block serial/arm work."""
    def __init__(self,config,factory,clock,deadline):
        self.config,self.factory,self.clock,self.deadline=config,factory,clock,deadline
        self.stop=threading.Event(); self.done=threading.Event(); self.opened=threading.Event(); self.lock=threading.Lock()
        self.latest=None; self.frame_id=0
        self.thread=threading.Thread(target=self._run,name='warehouse-number-camera',daemon=True)
    def _run(self):
        try:
            while not self.stop.is_set() and self.clock()<self.deadline:
                self._capture_once()
                if self.deadline!=float("inf"): break
                with self.lock: self.latest=None
                self.opened.clear()
                self.stop.wait(CAMERA_RECONNECT_S)
        finally: self.done.set()
    def _capture_once(self):
        capture=None
        try:
            capture=self.factory(self.config.device)
            if capture is None or not capture.isOpened(): return
            self.opened.set()
            capture.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc(*'MJPG'))
            capture.set(cv2.CAP_PROP_FRAME_WIDTH,self.config.width)
            capture.set(cv2.CAP_PROP_FRAME_HEIGHT,self.config.height)
            capture.set(cv2.CAP_PROP_FPS,self.config.fps)
            capture.set(cv2.CAP_PROP_BUFFERSIZE,1)
            frame_id=0
            while not self.stop.is_set() and self.clock()<self.deadline:
                timestamp=self.clock(); ok,frame=capture.read()
                if not ok or frame is None: return
                frame_id+=1
                if frame_id<=3: continue  # Discard buffers after each camera open.
                with self.lock:
                    self.frame_id+=1
                    self.latest=(self.frame_id,timestamp,frame)
        except Exception as exc:
            print(f'WAREHOUSE camera capture failed: {exc!r}',flush=True)
        finally:
            if capture is not None: capture.release()
    def snapshot(self):
        with self.lock:
            return self.latest

    def close(self):
        """Only the capture thread releases its driver; joining is bounded."""
        self.stop.set()
        if self.thread.ident is not None:
            self.thread.join(timeout=CAMERA_CLOSE_S if self.opened.is_set() else .05)

def recognize_number(config,excluded=0,cancel=None,*,capture_factory=None,clock=time.monotonic,frames=None,detector=None,on_ready=None):
    """Confirm new frames after a request; borrowed capture stays open.

    A standalone caller owns capture and cleanup. Service requests borrow the
    persistent camera and detector, but create a fresh confirmation gate.
    """
    if not config.device or (cancel is not None and cancel.is_set()): return 0
    deadline=clock()+config.timeout_s
    owned=frames is None
    requested_at=clock()
    camera=frames or _NumberFrames(config,capture_factory or (lambda d:cv2.VideoCapture(d,cv2.CAP_V4L2)),clock,deadline)
    detector=detector or DigitDetector(config.min_score,config.min_margin)
    gate=DigitConfirmation(excluded,config.confirm_frames,config.stale_s)
    if owned: camera.thread.start()
    last_frame=None
    ready_sent=False
    try:
        while clock()<deadline and not (cancel is not None and cancel.is_set()):
            snapshot=camera.snapshot()
            if snapshot is not None:
                frame_id,timestamp,frame=snapshot
                if frame_id==last_frame or timestamp < requested_at:
                    camera.done.wait(CAPTURE_POLL_S); continue
                last_frame=frame_id
                x,y,w,h=config.roi
                if frame.shape[1]<x+w or frame.shape[0]<y+h: return 0
                if not ready_sent:
                    if on_ready is not None: on_ready()
                    ready_sent=True
                digit,score=detector.detect(frame[y:y+h,x:x+w])
                found=gate.update(digit,frame_id,timestamp,clock())
                if found:
                    print(f'WAREHOUSE: digit={found}, score={score:.3f}',flush=True)
                    return found
            if camera.done.wait(CAPTURE_POLL_S): break
        return 0
    finally:
        if owned:
            camera.close()

class NumberCameraSession:
    """Warm after stairs, after the bridge closes the ball camera.

    Reuse templates, but release USB bandwidth when each check finishes so a
    later mission can open the ball camera without restarting this service.
    """
    def __init__(self, config):
        self.config = config
        self.detector = DigitDetector(config.min_score, config.min_margin)
        self.frames = None

    def start(self):
        """Warm capture without classifying; safe to call again at the request."""
        if self.config.device and self.frames is None:
            self.frames = _NumberFrames(
                self.config, lambda device: cv2.VideoCapture(device, cv2.CAP_V4L2),
                time.monotonic, float("inf"))
            self.frames.thread.start()

    def recognize(self, **kwargs):
        if not self.config.device:
            return 0
        self.start()
        try:
            return recognize_number(self.config, frames=self.frames,
                                    detector=self.detector, **kwargs)
        finally:
            self.close()

    def close(self):
        if self.frames is not None:
            self.frames.close()
            self.frames = None
