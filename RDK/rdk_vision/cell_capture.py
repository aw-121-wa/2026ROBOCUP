"""Bounded background snapshot writer; disk failures never interrupt a task."""
from pathlib import Path
import queue
import threading
import cv2


class CellCaptureWriter:
    def __init__(self):
        self.jobs=queue.Queue(maxsize=12)
        threading.Thread(target=self._write,daemon=True,name='cell-snapshots').start()

    def submit(self,path,frame):
        try:
            self.jobs.put_nowait((Path(path),frame.copy()))
            return True
        except (queue.Full,AttributeError) as exc:
            print(f'CELL SNAPSHOT skipped: {exc!r}',flush=True)
            return False

    def _write(self):
        while True:
            path,frame=self.jobs.get()
            try:
                path.parent.mkdir(parents=True,exist_ok=True)
                if not cv2.imwrite(str(path),frame): raise OSError('image write failed')
                print(f'CELL SNAPSHOT saved: {path}',flush=True)
            except Exception as exc:
                print(f'CELL SNAPSHOT failed: {exc!r}',flush=True)
            finally:
                self.jobs.task_done()


writer=CellCaptureWriter()
