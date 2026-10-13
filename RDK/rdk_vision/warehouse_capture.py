"""Bounded 5 Hz raw camera capture, independent of recognition latency."""
import json
import shutil
import threading
import time
from pathlib import Path

class Recorder:
    def __init__(self, root, row, color, snapshot):
        self.path=Path(root)/f"{time.time_ns()}-{color}-row{row}"
        self.row=row;self.color=color;self.snapshot=snapshot
        self.done=threading.Event();self.thread=None
    def start(self):
        self.path.mkdir(parents=True,exist_ok=True)
        self.thread=threading.Thread(target=self._run,daemon=True)
        self.thread.start()
    def stop(self):
        self.done.set()
        if self.thread: self.thread.join()
    def _run(self):
        import cv2
        last=None
        try:
            with (self.path/'frames.jsonl').open('w',encoding='utf-8') as log:
                while not self.done.wait(.2):
                    snap=self.snapshot()
                    if snap is None or snap[0]==last: continue
                    frame_id,stamp,frame=snap
                    if shutil.disk_usage(self.path).free<512*1024*1024:
                        raise OSError('less than 512 MiB free; capture stopped')
                    name=f'{frame_id:08d}.jpg'
                    if not cv2.imwrite(str(self.path/name),frame,[cv2.IMWRITE_JPEG_QUALITY,95]):
                        raise OSError('image write failed')
                    log.write(json.dumps(dict(file=name,frame_id=frame_id,monotonic=stamp,
                        wall_time=time.time(),row=self.row,color=self.color))+"\n")
                    log.flush();last=frame_id
        except Exception as exc:
            print(f'WAREHOUSE_CAPTURE_ERROR {self.path}: {exc}',flush=True)
