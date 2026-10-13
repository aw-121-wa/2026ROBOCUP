"""Validate timestamped destack samples against both continuous row profiles."""
import argparse,sys,time,json
from pathlib import Path
sys.path[:0]=[str(Path(__file__).resolve().parents[1]/'RDK')]
import cv2,yaml
from rdk_vision.block_digits import BlockDetector
from rdk_vision.warehouse_stream import locate
p=argparse.ArgumentParser();p.add_argument('samples',type=Path);args=p.parse_args()
root=Path(__file__).resolve().parents[1]/'RDK/rdk_vision';cfg=yaml.safe_load((root/'block_digits.yaml').read_text());report=[];failed=False
for side in ('red','blue'):
 for row in (1,2,3):
  s=cfg[side][row];d=BlockDetector(root,s);y=s['roi'][1]-(20 if row==2 else 0);h=s['roi'][3]+(40 if row==2 else 0)
  for label in ('1','2','3','empty'):
   files=sorted((args.samples/f'row{row}'/label).glob('20261012*/*.png'));expected=0 if label=='empty' else int(label);results=[];times=[]
   for f in files:
    image=cv2.imread(str(f));t=time.monotonic();results.append(locate(image,d.templates,(0,y,640,h),s['min_score'],s['min_margin'],score_floor=.73 if row==1 else .70)[0]);times.append(time.monotonic()-t)
   confirmed=expected==0 or any(results[i:i+3]==[expected]*3 for i in range(len(results)-2))
   wrong=sum(v not in (0,expected) for v in results)
   item=dict(side=side,row=row,label=label,total=len(results),correct=results.count(expected),wrong=wrong,confirmed=confirmed,max_ms=round(max(times,default=0)*1000))
   print(json.dumps(item),flush=True);report.append(item);failed|=bool(wrong or not confirmed or not files)
print('PASS' if not failed else 'FAIL');sys.exit(int(failed))
