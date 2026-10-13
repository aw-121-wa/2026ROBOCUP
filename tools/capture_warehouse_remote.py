import argparse,time,cv2
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--mode',choices=['destack','bottom'],required=True);p.add_argument('--row',type=int,default=0);p.add_argument('--digit',type=int,required=True);p.add_argument('--out',required=True);p.add_argument('--count',type=int,default=15);p.add_argument('--pose',action='store_true');p.add_argument('--pose-only',action='store_true');a=p.parse_args()
if a.pose or a.pose_only:
 from hiwonder_action import HiwonderActionBoard
 with HiwonderActionBoard('/dev/ttyS1',9600) as board:board.run_group(121-3*a.row,timeout_s=30)
if a.pose_only: raise SystemExit(0)
device='/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera_SN0001-video-index0' if a.mode=='destack' else '/dev/v4l/by-id/usb-RYS_RGB_RGB_Camera_200901010001-video-index0'
folder=Path(a.out)/('row'+str(a.row) if a.mode=='destack' else 'bottom')/str(a.digit);folder.mkdir(parents=True,exist_ok=True)
cap=cv2.VideoCapture(device,cv2.CAP_V4L2);cap.set(cv2.CAP_PROP_FRAME_WIDTH,640);cap.set(cv2.CAP_PROP_FRAME_HEIGHT,480)
try:
 for _ in range(30):
  ok,frame=cap.read()
  if not ok:raise RuntimeError('Camera read failed: '+device)
 for i in range(a.count):
  ok,frame=cap.read()
  if not ok:raise RuntimeError('Camera stopped')
  if not cv2.imwrite(str(folder/f'{i:03d}.png'),frame):raise RuntimeError('Image write failed')
  time.sleep(.1)
finally:cap.release()
print('Saved',a.count,'images to',folder,flush=True)
